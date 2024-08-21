#include "Interpolation.h"
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <set>
#include <unordered_map>
#include <sstream>
#include <cxxabi.h>


// Function to find the lower bound in a sorted vector
double findLowerBound(const std::vector<double>& vec, double value) {
    auto it = std::lower_bound(vec.begin(), vec.end(), value);
    if (it == vec.end() || (it != vec.begin() && *it > value)) {
        --it;
    }
    return *it;
}

std::vector<double> quantizeVector(const std::vector<double>& coords, double precision = 1e-6) {
    std::vector<double> quantizedCoords = coords;
    for (auto& coord : quantizedCoords) {
        coord = std::round(coord / precision) * precision;
    }
    return quantizedCoords;
}

#include <iomanip>  // for setting precision

#include <limits>  // for bounds checking

// Function to get the value at a specific point in the point cloud
double getValueAtPoint(const PointCloud& points, const std::vector<double>& queryCoords) {
    // Adjust query coordinates within epsilon and quantize
    std::vector<double> adjustedQueryCoords = quantizeVector(queryCoords);

    // Print the original query coordinates
    std::cerr << "Original query coordinates: (";
    for (size_t i = 0; i < queryCoords.size(); ++i) {
        std::cerr << std::fixed << std::setprecision(10) << queryCoords[i];
        if (i < queryCoords.size() - 1) std::cerr << ", ";
    }
    std::cerr << ")" << std::endl;

    // Print the adjusted (quantized) query coordinates
    std::cerr << "Adjusted (quantized) query coordinates: (";
    for (size_t i = 0; i < adjustedQueryCoords.size(); ++i) {
        std::cerr << std::fixed << std::setprecision(10) << adjustedQueryCoords[i];
        if (i < adjustedQueryCoords.size() - 1) std::cerr << ", ";
    }
    std::cerr << ")" << std::endl;

    // Bounds checking: Determine if the query point is within the bounds of the point cloud
    bool outOfBounds = false;
    for (size_t dim = 0; dim < adjustedQueryCoords.size(); ++dim) {
        double minVal = std::numeric_limits<double>::max();
        double maxVal = std::numeric_limits<double>::lowest();

        if (!points.uniqueValues[dim].empty()) {
            minVal = points.uniqueValues[dim].front();
            maxVal = points.uniqueValues[dim].back();
        }

        if (adjustedQueryCoords[dim] < minVal || adjustedQueryCoords[dim] > maxVal) {
            std::cerr << "Query coordinate out of bounds in dimension " << dim << ": "
                      << adjustedQueryCoords[dim] << " (min: " << minVal << ", max: " << maxVal << ")" << std::endl;
            outOfBounds = true;
        }
    }

    if (outOfBounds) {
        std::cerr << "Error: One or more query coordinates are out of bounds." << std::endl;
    } else {
        std::cerr << "All query coordinates are within bounds." << std::endl;
    }

    // Attempt to find the quantized query point in the pointMap
    auto it = points.pointMap.find(adjustedQueryCoords);
    if (it != points.pointMap.end()) {
        std::cerr << "Value found for adjusted query point: " << it->second << std::endl;
        return it->second;
    }

    // If not found, print the size of the pointMap
    std::cerr << "Point map size: " << points.pointMap.size() << std::endl;

    // Print out all points in the pointMap (for debugging purposes)
    std::cerr << "Contents of pointMap:" << std::endl;
    for (const auto& pair : points.pointMap) {
        std::cerr << "Point: (";
        for (size_t i = 0; i < pair.first.size(); ++i) {
            std::cerr << std::fixed << std::setprecision(10) << pair.first[i];
            if (i < pair.first.size() - 1) std::cerr << ", ";
        }
        std::cerr << ") -> Value: " << pair.second << std::endl;
    }

    // Check for near misses: print points close to the query point
    std::cerr << "Checking for near misses (points close to query):" << std::endl;
    for (const auto& pair : points.pointMap) {
        bool nearMiss = true;
        for (size_t i = 0; i < pair.first.size(); ++i) {
            if (std::abs(pair.first[i] - adjustedQueryCoords[i]) > 1e-6) {
                nearMiss = false;
                break;
            }
        }
        if (nearMiss) {
            std::cerr << "Near miss found: (";
            for (size_t i = 0; i < pair.first.size(); ++i) {
                std::cerr << std::fixed << std::setprecision(10) << pair.first[i];
                if (i < pair.first.size() - 1) std::cerr << ", ";
            }
            std::cerr << ") -> Value: " << pair.second << std::endl;
        }
    }

    // Prepare error message with query point details
    std::ostringstream errorMsg;
    errorMsg << "Value not found for query point: (";
    for (size_t i = 0; i < queryCoords.size(); ++i) {
        errorMsg << std::fixed << std::setprecision(10) << queryCoords[i];
        if (i < queryCoords.size() - 1) errorMsg << ", ";
    }
    errorMsg << ")";

    // Log error details with stack trace
    std::cerr << "Error in getValueAtPoint: " << errorMsg.str() << std::endl;

    // Throw exception with detailed message
    throw std::runtime_error(errorMsg.str());
}



// Recursive function to perform interpolation
double interpolateRecursive(const std::vector<double>& queryPoint, const PointCloud& points, size_t dim) {
    try {
        if (dim == 0) {
            return getValueAtPoint(points, queryPoint);
        }

        double lower = findLowerBound(points.uniqueValues[dim - 1], queryPoint[dim - 1]);
        double upper = *std::upper_bound(points.uniqueValues[dim - 1].begin(), points.uniqueValues[dim - 1].end(), lower);

        std::vector<double> lowerCoords = queryPoint;
        std::vector<double> upperCoords = queryPoint;
        lowerCoords[dim - 1] = lower;
        upperCoords[dim - 1] = upper;

        double lowerValue = interpolateRecursive(lowerCoords, points, dim - 1);
        double upperValue = interpolateRecursive(upperCoords, points, dim - 1);

        return (upper - queryPoint[dim - 1]) / (upper - lower) * lowerValue + (queryPoint[dim - 1] - lower) / (upper - lower) * upperValue;
    } catch (const std::exception& e) {
        // Log the parameters if an error has occurred
        std::cerr << "interpolateRecursive called with queryPoint: (";
        for (size_t i = 0; i < queryPoint.size(); ++i) {
            std::cerr << queryPoint[i];
            if (i < queryPoint.size() - 1) std::cerr << ", ";
        }
        std::cerr << "), dim: " << dim << std::endl;
        // Rethrow the exception to propagate it up the call stack
        throw;
    }
}

// Function to clamp a value between a minimum and maximum
double clamp(double value, double min, double max) {
    return std::max(min, std::min(value, max));
}

// Function to perform interpolation
double interpolate(const std::vector<double>& queryPoint, const PointCloud& points) {
    std::vector<double> clampedQueryPoint = queryPoint;
    
    // Clamp the query point coordinates to the valid range
    for (size_t i = 0; i < points.numDimensions; ++i) {
        clampedQueryPoint[i] = clamp(queryPoint[i], 
                                     points.uniqueValues[i].front(), 
                                     points.uniqueValues[i].back());
    }
    
    return interpolateRecursive(clampedQueryPoint, points, points.numDimensions);
}