#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>
#include <cstdlib>
#include <system_error>

namespace myapp {
namespace fs = std::filesystem;

inline std::string GetEnvOr(const char* k, std::string_view defv){ const char* v = std::getenv(k); return v && *v ? std::string(v) : std::string(defv); }

inline std::string ReadAllText(const fs::path& p){ std::ifstream ifs(p, std::ios::binary); return std::string{ std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>()}; }

inline std::string GetDeviceUid(){
  const char* paths[] = {
    "/sys/firmware/devicetree/base/serial-number",
    "/proc/device-tree/serial-number",
    "/sys/devices/soc0/serial_number",
  };
  for (auto* c : paths){
    std::error_code ec; fs::path p(c); if (!fs::exists(p, ec)) continue;
    std::string s = ReadAllText(p);
    if (!s.empty()){
      // strip NULs and whitespace
      std::string out; out.reserve(s.size());
      for (unsigned char ch : s){ if (ch != '\0') out.push_back(static_cast<char>(ch)); }
      // trim spaces and newlines
      while (!out.empty() && (out.back()=='\n' || out.back()=='\r' || out.back()==' ' || out.back()=='\t')) out.pop_back();
      return out;
    }
  }
  throw std::runtime_error("No UID found in standard paths");
}

inline int RunCmd(const std::string& cmd){
#ifdef _WIN32
  (void)cmd; return -1;
#else
  int rc = std::system((cmd+" > /dev/null 2>&1").c_str());
  if (rc == -1) return -1; if (WIFEXITED(rc)) return WEXITSTATUS(rc); return -1;
#endif
}

inline bool VerifyUidLicense(std::string& error_out){
  const std::string uid_dir = GetEnvOr("MYAPP_UID_DIR", "/etc/myapp/uid");
  fs::path pub = fs::path(uid_dir) / "public.pem";
  fs::path lic = fs::path(uid_dir) / "license.uid";
  fs::path sig = fs::path(uid_dir) / "license.sig";

  std::error_code ec;
  if (!fs::is_directory(uid_dir, ec)) { error_out = "UID directory not found: "+uid_dir; return false; }
  if (!fs::exists(pub) || !fs::exists(lic) || !fs::exists(sig)) { error_out = "Missing UID license files in "+uid_dir; return false; }

  std::string uid_actual;
  try { uid_actual = GetDeviceUid(); } catch (const std::exception& e){ error_out = e.what(); return false; }

  std::string uid_licensed = ReadAllText(lic);
  // trim
  while (!uid_licensed.empty() && (uid_licensed.back()=='\n' || uid_licensed.back()=='\r' || uid_licensed.back()==' ' || uid_licensed.back()=='\t')) uid_licensed.pop_back();

  if (uid_actual != uid_licensed){ error_out = std::string("UID mismatch: actual=")+uid_actual+" licensed="+uid_licensed; return false; }

  // Verify signature via openssl
  std::string cmd = std::string("openssl dgst -sha256 -verify ") + pub.string() +
                    " -signature " + sig.string() + " " + lic.string();
  int rc = RunCmd(cmd);
  if (rc != 0){ error_out = "Signature verify failed (openssl)"; return false; }
  return true;
}

inline bool EnforceUidLicense(std::string& error_out){ return VerifyUidLicense(error_out); }

} // namespace myapp

// usage
// #include <iostream>
// #include "tpm/uid_check.hpp"
// int main() {
//   std::string err;
//   if (!myapp::EnforceUidLicense(err)) {
//     std::cerr << "Not licensed for this machine: " << err << std::endl;
//     return 1;
//   }
//   std::cout << "License OK" << std::endl;
//   return 0;
// }
