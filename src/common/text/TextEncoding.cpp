// Text encoding helpers for values that arrive from outside the library.
// Implementation note: see the header for the contract.

#include "1q/foundation/TextEncoding.h"

#include <vector>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace oneq {
namespace foundation {

bool IsValidUtf8(const std::string& text) {
#if defined(_WIN32)
  if (text.empty()) {
    return true;
  }
  const int wide_len = ::MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(),
                                             static_cast<int>(text.size()), nullptr, 0);
  return wide_len > 0;
#else
  (void)text;
  return true;
#endif
}

#if defined(_WIN32)
static bool ConvertFromCodePage(const std::string& text, UINT code_page, std::string* out) {
  if (out == nullptr) {
    return false;
  }
  const int wide_len = ::MultiByteToWideChar(code_page, 0, text.data(),
                                             static_cast<int>(text.size()), nullptr, 0);
  if (wide_len <= 0) {
    return false;
  }
  std::vector<wchar_t> wide(static_cast<std::size_t>(wide_len), L'\0');
  if (::MultiByteToWideChar(code_page, 0, text.data(), static_cast<int>(text.size()), &wide[0],
                            wide_len) <= 0) {
    return false;
  }
  const int utf8_len = ::WideCharToMultiByte(CP_UTF8, 0, &wide[0], wide_len, nullptr, 0, nullptr,
                                             nullptr);
  if (utf8_len <= 0) {
    return false;
  }
  std::string utf8(static_cast<std::size_t>(utf8_len), '\0');
  if (::WideCharToMultiByte(CP_UTF8, 0, &wide[0], wide_len, &utf8[0], utf8_len, nullptr, nullptr) <=
      0) {
    return false;
  }
  *out = utf8;
  return true;
}
#endif

std::string ToSystemAnsiText(const std::string& text) {
  if (text.empty()) {
    return text;
  }
#if defined(_WIN32)
  const int wide_len = ::MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, text.data(),
                                             static_cast<int>(text.size()), nullptr, 0);
  if (wide_len <= 0) {
    return text;  // 不是合法 UTF-8：可能本来就是 ANSI，原样返回。
  }
  std::vector<wchar_t> wide(static_cast<std::size_t>(wide_len), L'\0');
  if (::MultiByteToWideChar(CP_UTF8, 0, text.data(), static_cast<int>(text.size()), &wide[0],
                            wide_len) <= 0) {
    return text;
  }
  const int ansi_len = ::WideCharToMultiByte(CP_ACP, 0, &wide[0], wide_len, nullptr, 0, nullptr,
                                             nullptr);
  if (ansi_len <= 0) {
    return text;
  }
  std::string ansi(static_cast<std::size_t>(ansi_len), '\0');
  if (::WideCharToMultiByte(CP_ACP, 0, &wide[0], wide_len, &ansi[0], ansi_len, nullptr, nullptr) <=
      0) {
    return text;
  }
  return ansi;
#else
  return text;
#endif
}

std::string NormalizeExternalText(const std::string& text) {
  if (text.empty()) {
    return text;
  }
  if (IsValidUtf8(text)) {
    return text;
  }
#if defined(_WIN32)
  std::string converted;
  if (ConvertFromCodePage(text, CP_ACP, &converted)) {
    return converted;
  }
  if (ConvertFromCodePage(text, 54936U, &converted)) {
    return converted;
  }
  std::string fallback;
  fallback.reserve(text.size());
  for (std::size_t i = 0U; i < text.size(); ++i) {
    const unsigned char octet = static_cast<unsigned char>(text[i]);
    fallback += (octet < 0x80U) ? static_cast<char>(octet) : '?';
  }
  return fallback;
#else
  return text;
#endif
}

std::string TruncateUtf8(const std::string& text, std::size_t max_bytes) {
  if (text.size() <= max_bytes) {
    return text;
  }
  std::size_t cut = max_bytes;
  while (cut > 0U) {
    const unsigned char octet = static_cast<unsigned char>(text[cut]);
    if ((octet & 0xC0U) != 0x80U) {
      break;
    }
    cut = cut - 1U;
  }
  return text.substr(0U, cut);
}

}  // namespace foundation
}  // namespace oneq