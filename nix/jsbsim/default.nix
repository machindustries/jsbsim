{ pkgs ? import <nixpkgs> {} }:
pkgs.stdenv.mkDerivation {
  src = ../../.;
  name = "jsbsim";
  version = "0.0.1";
  buildInputs = [
    pkgs.cmake
    pkgs.pkg-config
  ];
  cmakeFlagsArray = [
    ''-DCMAKE_CXX_FLAGS_RELEASE=-O3 -march=native -mtune=native''
    ''-DCMAKE_C_FLAGS_RELEASE=-O3 -march=native -mtune=native''
    "-DBUILD_SHARED_LIBS=ON" 
    "-DCMAKE_INSTALL_PREFIX=$out"
  ];
}
