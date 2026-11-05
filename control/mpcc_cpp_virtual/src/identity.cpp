#include "mpcc_cpp_virtual/identity.hpp"

#include "acados_solver_slash_mpcc.h"
#include "identity_constants.hpp"

#include <dlfcn.h>
#include <openssl/evp.h>

#include <array>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace mpcc_cpp_virtual {
namespace {
std::string sha256(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open artifact: " + path);
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(
      EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("SHA256 initialization failed");
  std::array<char, 65536> buffer{};
  while (input) {
    input.read(buffer.data(), buffer.size());
    const auto count = input.gcount();
    if (count > 0 && EVP_DigestUpdate(context.get(), buffer.data(), count) != 1)
      throw std::runtime_error("SHA256 update failed");
  }
  if (!input.eof()) throw std::runtime_error("cannot read artifact: " + path);
  unsigned char digest[EVP_MAX_MD_SIZE]{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest, &digest_size) != 1 ||
      digest_size != 32) throw std::runtime_error("SHA256 finalization failed");
  std::ostringstream result;
  for (unsigned int i = 0; i < digest_size; ++i)
    result << std::hex << std::setfill('0') << std::setw(2)
           << static_cast<unsigned int>(digest[i]);
  return result.str();
}

void require_hash(const std::string& path, const char* expected) {
  if (sha256(path) != expected)
    throw std::runtime_error("MPCC artifact hash mismatch: " + path);
}
}  // namespace

void verify_runtime_artifacts(const std::string& track_path,
                              const std::string& runtime_path) {
  require_hash(track_path, kExpectedTrackSha);
  require_hash(runtime_path, kExpectedRuntimeSha);
  Dl_info solver{};
  if (!dladdr(reinterpret_cast<const void*>(&slash_mpcc_acados_create_capsule),
              &solver) || !solver.dli_fname)
    throw std::runtime_error("cannot identify loaded MPCC solver library");
  require_hash(solver.dli_fname, kExpectedSolverSha);
}
}  // namespace mpcc_cpp_virtual
