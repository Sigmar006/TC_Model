// Text encoding helpers for values that arrive from outside the library.
// Implementation note: see the header for the contract.

#include "1q/foundation/TextEncoding.h"

#include <cstdint>
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
  // 非 Windows 平台按 RFC 3629 逐字节校验：拒绝非法首字节、被截断的续字节、超长编码、
  // 代理区 U+D800..U+DFFF 以及超出 U+10FFFF 的码位。
  // 此前这里恒返回 true，导致 GBK/其它本地编码的字节流被当成合法 UTF-8 直接写入
  // FlatBuffers 与验收日志——而头文件承诺的是"非 Windows 也给出真实判定"。
  const std::size_t size = text.size();
  std::size_t index = 0U;
  while (index < size) {
    const unsigned char lead = static_cast<unsigned char>(text[index]);
    if (lead < 0x80U) {  // ASCII 单字节
      ++index;
      continue;
    }
    std::size_t extra = 0U;
    std::uint32_t code_point = 0U;
    if ((lead & 0xE0U) == 0xC0U) {  // 2 字节序列
      extra = 1U;
      code_point = static_cast<std::uint32_t>(lead & 0x1FU);
    } else if ((lead & 0xF0U) == 0xE0U) {  // 3 字节序列
      extra = 2U;
      code_point = static_cast<std::uint32_t>(lead & 0x0FU);
    } else if ((lead & 0xF8U) == 0xF0U) {  // 4 字节序列
      extra = 3U;
      code_point = static_cast<std::uint32_t>(lead & 0x07U);
    } else {
      return false;  // 0x80..0xBF 作首字节，或 0xF8..0xFF
    }
    if (index + extra >= size) {
      return false;  // 多字节序列在串尾被截断
    }
    for (std::size_t offset = 1U; offset <= extra; ++offset) {
      const unsigned char continuation = static_cast<unsigned char>(text[index + offset]);
      if ((continuation & 0xC0U) != 0x80U) {
        return false;
      }
      code_point = (code_point << 6U) | static_cast<std::uint32_t>(continuation & 0x3FU);
    }
    if ((extra == 1U && code_point < 0x80U) || (extra == 2U && code_point < 0x800U) ||
        (extra == 3U && code_point < 0x10000U) ||
        (code_point >= 0xD800U && code_point <= 0xDFFFU) || code_point > 0x10FFFFU) {
      return false;  // 超长编码 / 代理区 / 超上限
    }
    index += extra + 1U;
  }
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
  // 非 Windows 平台没有 CP_ACP / GBK 转码能力（iconv 属额外依赖，国产化离线环境
  // 不宜新增）；退化为与 Windows 兜底一致的降级：非 ASCII 字节替换为 '?'，
  // 保证绝不把坏字节透传给 FlatBuffers / 验收日志。
  std::string fallback;
  fallback.reserve(text.size());
  for (std::size_t i = 0U; i < text.size(); ++i) {
    const unsigned char octet = static_cast<unsigned char>(text[i]);
    fallback += (octet < 0x80U) ? static_cast<char>(octet) : '?';
  }
  return fallback;
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