/**
 * @file ReplayTrace.h
 * @brief 定义可复现仿真用的回放 trace（Replay Trace）写入、读取与回放公开 API。
 *
 * Trace 以目录形式存储：manifest.json 描述元信息，events 以 JSONL 分 chunk 落盘，
 * 失败标记与最近事件窗口单独成文件。读取侧支持扫描完整性、兼容性校验、报告生成
 * 与逐事件回放回调。
 */

#ifndef ONEQ_REPLAY_REPLAY_TRACE_H_
#define ONEQ_REPLAY_REPLAY_TRACE_H_

#include <cstdint>
#include <memory>
#include <string>

#include "1q/api.hpp"

namespace oneq {
namespace replay {

/**
 * @brief 回放 trace 的清单（manifest）元信息，落盘为 manifest.json。
 */
struct ONEQ_API ReplayTraceManifest {
  std::string trace_id{};
  std::string module{};
  std::string scenario_id{};
  int schema_version{1};
  std::string serializer_version{"replay-flatbuffers-v1"};
  std::string git_commit{};
  bool git_dirty{false};
  std::string build_type{};
  std::string compiler{};
  std::string compiler_version{};
  std::string platform{};
  std::string cpu_arch{};
  std::string library_version{};
  std::string dependency_versions_payload{"{}"};
  std::string float_policy{};
  std::string default_tolerances_payload{"{}"};
  std::uint32_t checkpoint_interval_cycles{0U};
  std::uint32_t event_chunk_size{10000U};
  std::uint32_t failure_window_event_count{128U};
  bool compress_closed_chunks{false}; /**< 若为 true，Writer 关闭 chunk 后将其压缩为 .gz
                                         并删除原文件；Reader 自动透明解压 */
};

/**
 * @brief 写入侧的单条 trace 事件描述（待序列化为 JSONL）。
 */
struct ONEQ_API ReplayTraceEvent {
  std::string module{};
  std::string event_type{};
  std::string payload_type{};
  std::string payload_encoding{"flatbuffers"};
  std::string payload_inline{"{}"};
  std::string payload_bytes{};
  bool has_cycle_index{false};
  std::uint32_t cycle_index{0U};
  bool has_sim_time_sec{false};
  double sim_time_sec{0.0};
};

/**
 * @brief 读取侧解析出的单条事件，包含序列号、哈希校验与 payload。
 */
struct ONEQ_API ReplayTraceReadEvent {
  std::string raw_event_json{};
  std::int32_t schema_version{0};
  std::string trace_id{};
  std::uint64_t sequence{0U};
  std::string module{};
  std::string event_type{};
  bool has_cycle_index{false};
  std::uint32_t cycle_index{0U};
  bool has_sim_time_sec{false};
  double sim_time_sec{0.0};
  std::string payload_type{};
  std::string payload_encoding{};
  std::string payload_inline{};
  std::string payload_bytes{};
  std::string payload_hash{};
  std::string previous_event_hash{};
  bool payload_hash_matches{false};
  std::string event_hash{};
  bool previous_event_hash_matches{false};
};

/**
 * @brief 扫描 trace 目录得到的完整性校验结果。
 */
struct ONEQ_API ReplayTraceScanResult {
  std::uint64_t event_count{0U};
  bool payload_hashes_ok{true};
  bool event_chain_ok{true};
  bool sequences_contiguous{true};
  bool ok{true};
  std::string first_error{};
};

/**
 * @brief 调用方对 trace 兼容性的期望条件（schema/serializer/git commit/module）。
 */
struct ONEQ_API ReplayTraceCompatibilityExpectation {
  std::int32_t schema_version{1};
  std::string serializer_version{"replay-flatbuffers-v1"};
  std::string git_commit{};
  bool require_git_commit_match{false};
  std::string module{};
  bool require_module_match{false};
};

/**
 * @brief trace 相对期望条件的兼容性校验结果，含 manifest 实际值镜像。
 */
struct ONEQ_API ReplayTraceCompatibilityResult {
  bool compatible{true};
  bool schema_version_matches{true};
  bool serializer_version_matches{true};
  bool git_commit_matches{true};
  bool module_matches{true};
  bool manifest_git_dirty{false};
  std::string manifest_trace_id{};
  std::string manifest_module{};
  std::int32_t manifest_schema_version{0};
  std::string manifest_serializer_version{};
  std::string manifest_git_commit{};
  std::string first_error{};
  std::string warning{};
};

/**
 * @brief 供报告输出使用的回放 trace 综合摘要（兼容性 + 扫描 + 事件计数）。
 */
struct ONEQ_API ReplayTraceReplayReport {
  ReplayTraceCompatibilityResult compatibility{};
  ReplayTraceScanResult scan{};
  bool replay_ready{false};
  bool has_session_config{false};
  bool has_failure_marker{false};
  std::uint64_t session_config_count{0U};
  std::uint64_t cycle_input_count{0U};
  std::uint64_t decision_input_count{0U};
  std::uint64_t scene_state_count{0U};
  std::uint64_t runtime_config_patch_count{0U};
  std::uint64_t cycle_output_count{0U};
  std::uint64_t failure_marker_count{0U};
  std::uint64_t warning_event_count{0U};
  std::uint64_t unsupported_event_count{0U};
  std::uint64_t first_failure_sequence{0U};
  std::string first_failure_payload_base64{};
  std::string first_failure_payload_encoding{};
  std::string first_failure_payload_type{};
  std::string first_error{};
  std::string warning{};
};

/**
 * @brief 失败标记事件，记录一次仿真失败的错误码、定位与诊断 payload。
 */
struct ONEQ_API ReplayTraceFailure {
  std::string error_code{};
  std::string message{};
  std::string location{};
  bool has_cycle_index{false};
  std::uint32_t cycle_index{0U};
  bool has_sim_time_sec{false};
  double sim_time_sec{0.0};
  std::string diagnostics_payload{"{}"};
};

/**
 * @brief 单条事件的回放回调签名。
 * @param[in] event 当前读取到的事件。
 * @param[in] user_data 调用方透传的上下文指针。
 * @param[out] error 失败时写入错误描述。
 * @return 继续回放返回 true，停止回放返回 false（并填充 error）。
 */
using ReplayTraceEventCallback = bool (*)(const ReplayTraceReadEvent& event, void* user_data,
                                          std::string* error);

/**
 * @brief cycle_output 回调返回的结构化比较状态。
 *
 * 替代旧的 bool 返回值，让模块在回调中显式区分三类结果，避免回放框架靠解析
 * `error` 文本字符串来判断是否发生输出分叉（见 `docs/common/contract.md` 的
 * “Replay 与 trace 语义”）。
 *
 * - `kHandledByModule`：模块已逐字段比较，输出一致。`actual_output_payload` 应留空
 *   （表示比较由模块内部完成），或填入模块自行生成的诊断 JSON。
 * - `kDivergence`：模块检测到输出分叉（逐字段比较不一致）。模块应同时通过
 *   `actual_output_payload` 携带实际输出摘要，供消费方结构化读取。
 * - `kOtherFailure`：解码失败、入口 payload 类型不匹配、cycle 执行失败等。
 *   这类失败**不**视为输出分叉，仅记为回放失败。
 */
enum class ReplayTraceOutputStatus {
  kHandledByModule = 0, /**< 模块已比较且输出一致 */
  kDivergence = 1,      /**< 模块检测到输出分叉 */
  kOtherFailure = 2     /**< 解码/执行/类型不匹配等失败，非分叉 */
};

/** @brief Replay trace 写入/刷新操作的结构化结果。 */
enum class ONEQ_API ReplayTraceWriteStatus {
  kSuccess = 0, /**< 操作成功 */
  kError = 1    /**< 操作失败；通过 Writer::first_error() 获取首错 */
};

/** @brief Replay trace 单步读取的结构化结果。 */
enum class ONEQ_API ReplayTraceReadStatus {
  kEvent = 0,      /**< 成功读取一条事件 */
  kEndOfTrace = 1, /**< 正常到达 trace 末尾 */
  kError = 2       /**< 读取失败；通过 Reader::first_error() 获取首错 */
};

/**
 * @brief cycle_output 的结构化比较回调签名。
 * @param[in] event 当前读取到的 cycle_output 事件。
 * @param[in] user_data 调用方透传的上下文指针。
 * @param[out] actual_output_payload 实际输出摘要（分叉时由模块填充）。
 * @param[out] error 失败时写入错误描述。
 * @return 比较结果状态（见 ReplayTraceOutputStatus）。
 */
using ReplayTraceOutputCallback = ReplayTraceOutputStatus (*)(const ReplayTraceReadEvent& event,
                                                              void* user_data,
                                                              std::string* actual_output_payload,
                                                              std::string* error);

/**
 * @brief 回放过程中按事件类型分发的回调集合；未设置的字段表示该类事件无需处理。
 */
struct ONEQ_API ReplayTracePlaybackCallbacks {
  void* user_data{nullptr};
  ReplayTraceEventCallback on_session_config{nullptr};
  ReplayTraceEventCallback on_cycle_input{nullptr};
  ReplayTraceEventCallback on_decision_input{nullptr};
  ReplayTraceEventCallback on_scene_state{nullptr};
  ReplayTraceEventCallback on_runtime_config_patch{nullptr};
  ReplayTraceOutputCallback on_cycle_output{nullptr};
  ReplayTraceEventCallback on_failure_marker{nullptr};
};

/**
 * @brief 回放行为开关。
 */
struct ONEQ_API ReplayTracePlaybackOptions {
  bool stop_on_first_divergence{true};
  bool stop_on_failure_marker{false};
  bool require_output_callback{false};
};

/**
 * @brief 回放执行结果，包含事件计数、分叉定位与首个错误。
 */
struct ONEQ_API ReplayTracePlaybackResult {
  bool ok{true};
  std::uint64_t processed_event_count{0U}; /**< 已读取的 trace event 数，使用 64-bit 以支持长回放 */
  std::uint64_t applied_input_count{0U};   /**< 已应用的 cycle_input 数，使用 64-bit 以支持长回放 */
  std::uint64_t applied_decision_input_count{0U}; /**< 已应用的 decision_input 数 */
  std::uint64_t applied_scene_state_count{0U};   /**< 已应用的 scene_state 数 */
  std::uint64_t applied_runtime_patch_count{0U}; /**< 已应用的 runtime_config_patch 数 */
  std::uint64_t compared_output_count{
      0U};                                /**< 已比较的 cycle_output 数，使用 64-bit 以支持长回放 */
  std::uint64_t skipped_output_count{0U}; /**< 未要求输出回调时跳过的 cycle_output 数 */
  std::uint64_t failure_marker_count{0U}; /**< 已遇到的 failure_marker 数 */
  bool divergence_found{false};
  std::uint64_t divergence_sequence{0U};
  std::string expected_output_payload{};
  std::string actual_output_payload{};
  std::string first_error{};
};

/**
 * @brief 回放 trace 写入器，负责落盘 manifest、分 chunk 的 JSONL 事件与失败标记。
 *
 * 构造时创建 trace 目录并写入 manifest；事件按 event_chunk_size 分文件，
 * 析构时自动 flush。不可拷贝。
 */
class ONEQ_API ReplayTraceWriter final {
 public:
  /**
   * @brief 构造写入器并初始化 trace 目录。
   * @param[in] trace_dir trace 输出目录。
   * @param[in] manifest 清单元信息。
   * @param[in] overwrite 是否允许覆盖既有 replay trace 工件（默认 false）。为 false 时，
   *            若目录内已有 manifest、事件 chunk 或周期索引，Writer 将禁用且不修改既有文件。
   */
  ReplayTraceWriter(std::string trace_dir, ReplayTraceManifest manifest, bool overwrite = false);
  ~ReplayTraceWriter();

  ReplayTraceWriter(const ReplayTraceWriter&) = delete;
  ReplayTraceWriter& operator=(const ReplayTraceWriter&) = delete;

  /** @brief 追加一条事件并返回结构化写入状态。 */
  ReplayTraceWriteStatus WriteEvent(const ReplayTraceEvent& event);
  /** @brief 写入失败标记并返回结构化写入状态。 */
  ReplayTraceWriteStatus WriteFailureMarker(const ReplayTraceFailure& failure);
  /** @brief 写入带 payload 的失败标记并返回结构化写入状态。 */
  ReplayTraceWriteStatus WriteFailureMarker(const ReplayTraceFailure& failure,
                                            const std::string& payload_bytes);
  /** @brief 刷新所有缓冲并返回结构化写入状态。 */
  ReplayTraceWriteStatus Flush();
  /** @return Writer 是否仍可写。 */
  bool ok() const;
  /** @return 首个写入错误；无错误时为空。 */
  const std::string& first_error() const;

  /** @return trace 输出目录。 */
  const std::string& trace_dir() const;
  /** @return 构造时传入的 manifest。 */
  const ReplayTraceManifest& manifest() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/**
 * @brief 回放 trace 读取器，按顺序读取 manifest.json 与各 chunk 中的事件。
 *
 * 构造时读取并缓存 manifest；通过 ReadNextEvent 逐条推进。不可拷贝。
 */
class ONEQ_API ReplayTraceReader final {
 public:
  /**
   * @brief 构造读取器，打开 trace_dir 下的 manifest。
   * @param[in] trace_dir trace 目录。
   */
  explicit ReplayTraceReader(std::string trace_dir);
  ~ReplayTraceReader();

  ReplayTraceReader(const ReplayTraceReader&) = delete;
  ReplayTraceReader& operator=(const ReplayTraceReader&) = delete;

  /** @return trace 目录。 */
  const std::string& trace_dir() const;
  /** @return manifest.json 的原始文本。 */
  const std::string& manifest_json() const;
  /**
   * @brief 读取下一条事件。
   * @param[out] event 输出解析得到的事件。
   * @return 结构化读取状态。
   */
  ReplayTraceReadStatus ReadNextEvent(ReplayTraceReadEvent* event);
  /** @return Reader 初始化及最近一次读取是否未失败。 */
  bool ok() const;
  /** @return 首个读取错误；无错误时为空。 */
  const std::string& first_error() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

/**
 * @brief 扫描 trace 目录并校验事件链、序列连续性与 payload 哈希完整性。
 * @param[in] trace_dir trace 目录。
 * @return 扫描结果（含首个错误描述）。
 */
ONEQ_API ReplayTraceScanResult ScanReplayTrace(const std::string& trace_dir);
/**
 * @brief 按 expectation 校验 trace 与期望条件的兼容性。
 * @param[in] trace_dir trace 目录。
 * @param[in] expectation 期望的 schema/serializer/git commit/module 条件。
 * @return 兼容性结果（含 manifest 实际值镜像）。
 */
ONEQ_API ReplayTraceCompatibilityResult CheckReplayTraceCompatibility(
    const std::string& trace_dir, const ReplayTraceCompatibilityExpectation& expectation);
/**
 * @brief 构建可序列化的回放 trace 综合报告（兼容性 + 扫描 + 事件计数）。
 * @param[in] trace_dir trace 目录。
 * @param[in] expectation 兼容性期望条件。
 * @return 综合报告。
 */
ONEQ_API ReplayTraceReplayReport BuildReplayTraceReport(
    const std::string& trace_dir, const ReplayTraceCompatibilityExpectation& expectation);
/**
 * @brief 将回放报告以 JSON 形式写入文件。
 * @param[in] report 待写出的报告。
 * @param[in] report_path 输出文件路径。
 */
ONEQ_API void WriteReplayTraceReport(const ReplayTraceReplayReport& report,
                                     const std::string& report_path);
/**
 * @brief 逐事件回放 trace，按事件类型调用 callbacks；遇到分叉或失败时按 options 决定是否停止。
 * @param[in] trace_dir trace 目录。
 * @param[in] callbacks 按事件类型分发的回调集合。
 * @param[in] options 回放行为开关。
 * @return 回放结果（含事件计数与分叉定位）。
 */
ONEQ_API ReplayTracePlaybackResult
PlaybackReplayTrace(const std::string& trace_dir, const ReplayTracePlaybackCallbacks& callbacks,
                    const ReplayTracePlaybackOptions& options = ReplayTracePlaybackOptions{});

}  // namespace replay
}  // namespace oneq

#endif  // ONEQ_REPLAY_REPLAY_TRACE_H_
