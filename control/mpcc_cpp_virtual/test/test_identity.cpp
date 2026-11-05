#include "mpcc_cpp_virtual/identity.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  mpcc_cpp_virtual::verify_runtime_artifacts(argv[1], argv[2]);
  const std::string altered = "/tmp/mpcc_identity_test_" + std::to_string(getpid());
  {
    std::ifstream original(argv[1], std::ios::binary);
    std::ofstream copy(altered, std::ios::binary);
    copy << original.rdbuf() << '\n';
  }
  bool rejected = false;
  try {
    mpcc_cpp_virtual::verify_runtime_artifacts(altered, argv[2]);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  std::remove(altered.c_str());
  if (!rejected) return 3;
  std::cout << "matched artifacts accepted; changed track rejected\n";
}
