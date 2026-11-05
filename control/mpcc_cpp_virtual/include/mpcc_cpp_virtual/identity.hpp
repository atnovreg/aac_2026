#pragma once

#include <string>

namespace mpcc_cpp_virtual {

// Check frozen inputs and the acados library actually loaded by this process.
void verify_runtime_artifacts(const std::string& track_path,
                              const std::string& runtime_path);

}  // namespace mpcc_cpp_virtual
