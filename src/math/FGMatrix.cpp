#include "FGMatrix.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <algorithm>
#include <set>

using namespace std;

FGMatrix::FGMatrix(JSBSim::Element* el) : name("Matrix") {
    std::vector<std::string> data_lines;
    std::string line;
    unsigned int i = 0;

    // Collect all data lines
    while (!(line = el->GetDataLine(i++)).empty() && line != "end") {
        data_lines.push_back(line);
    }
    if (data_lines.empty()) {
        throw std::runtime_error("Empty matrix data");
    }

    // Parse the data lines into a 2D vector of doubles
    size_t total_rows = data_lines.size();
    std::vector<std::vector<double>> temp_matrix;

    for (const auto& data_line : data_lines) {
        std::istringstream iss(data_line);
        std::vector<double> row;
        double value;
        while (iss >> value) {
            row.push_back(value);
        }

        if (!temp_matrix.empty() && row.size() != temp_matrix[0].size()) {
            std::cout << "Current row size: " << row.size() << std::endl;
            std::cout << "Expected row size: " << temp_matrix[0].size() << std::endl;

            throw std::runtime_error("Inconsistent number of columns in matrix");
        }

        temp_matrix.push_back(std::move(row));
    }

    matrix = std::move(temp_matrix);
    num_dimensions = matrix[0].size() - 1;
    populatePointCloud();  // Populate the PointCloud
}

double FGMatrix::GetValue() const {
    return 0.0;  // This method might not be used directly for a matrix
}

std::string FGMatrix::GetName() const {
    return name;
}

bool FGMatrix::IsConstant() const {
    return true;  // Assuming the matrix doesn't change after initialization
}

const std::vector<std::vector<double>>& FGMatrix::GetMatrix() const {
    return matrix;
}

size_t FGMatrix::GetNumDimensions() const {
    return num_dimensions;
}

void FGMatrix::Print() const {
    std::cout << "Matrix: " << name << std::endl;
    std::cout << "Dimensions: " << num_dimensions << std::endl;
    std::cout << "Data:" << std::endl;

    // Find the maximum width needed for formatting
    size_t max_width = 0;
    for (const auto& row : matrix) {
        for (const auto& val : row) {
            std::ostringstream temp;
            temp << std::setprecision(FGMatrix::PRINT_PRECISION) << val;  // Set precision to 8 decimal places
            max_width = std::max(max_width, static_cast<size_t>(temp.str().length()));
        }
    }

    // Print the matrix with aligned columns
    for (const auto& row : matrix) {
        for (const auto& val : row) {
            std::cout << std::setw(max_width + 2) << std::setprecision(FGMatrix::PRINT_PRECISION) << val << " ";  // Set precision to 8 decimal places
        }
        std::cout << std::endl;
    }
}

void FGMatrix::populatePointCloud() {
    pointCloud.points.clear();
    pointCloud.pointMap.clear();
    pointCloud.numDimensions = num_dimensions;

    for (const auto& row : matrix) {
        std::vector<double> coords(row.begin(), row.end() - 1);
        double value = row.back();
        pointCloud.points.emplace_back(coords, value);
        pointCloud.pointMap[coords] = value;
    }

    // Extract unique values for each dimension
    size_t dimensions = pointCloud.points[0].coordinates.size();
    pointCloud.uniqueValues.resize(dimensions);

    for (size_t dim = 0; dim < dimensions; ++dim) {
        std::set<double> valueSet;
        for (const auto& point : pointCloud.points) {
            valueSet.insert(point.coordinates[dim]);
        }
        pointCloud.uniqueValues[dim].assign(valueSet.begin(), valueSet.end());
    }
}