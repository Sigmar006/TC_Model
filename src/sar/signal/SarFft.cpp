#include "sar/signal/SarFft.h"

#include "common/platform/OneqPlatformCompat.h"

#include <algorithm>
#include <map>
#include <memory>
#include <utility>
#include <unsupported/Eigen/FFT>

namespace sar {
namespace signal {

namespace {

bool HasValidShape(const ComplexMatrix& matrix) {
  return matrix.rows > 0U && matrix.cols > 0U && matrix.values.size() == matrix.rows * matrix.cols;
}

/**
 * @brief 按线程缓存的 FFT plan 池。
 *
 * Eigen FFT 对象在首次执行某个长度时惰性构建旋转因子表（O(N) 次 sin/cos + 堆分配）。
 * 旧实现每次 Fft1D 都新建 Eigen::FFT 对象，导致每次 FFT 都重复这笔开销；
 * 这里按（线程, 长度）缓存 plan 复用，且线程局部缓存避免跨线程共享可变状态。
 */
class FftPlanCache {
 public:
  Eigen::FFT<double>& GetPlan(std::size_t size) {
    std::unique_ptr<Eigen::FFT<double> >& slot = plans_[size];
    if (!slot) {
      slot.reset(new Eigen::FFT<double>());
    }
    return *slot;
  }

 private:
  std::map<std::size_t, std::unique_ptr<Eigen::FFT<double> > > plans_;
};

FftPlanCache& GetFftPlanCache() {
#if defined(_MSC_VER) && _MSC_VER < 1900
  // VS2013 无 thread_local，且 __declspec(thread) 不支持非平凡类型，改以线程局部指针托管。
  static ONEQ_THREAD_LOCAL FftPlanCache* cache = nullptr;
  if (cache == nullptr) {
    cache = new FftPlanCache();
  }
  return *cache;
#else
  static thread_local FftPlanCache cache;
  return cache;
#endif
}

/**
 * @brief 分块转置：以固定块遍历源矩阵，兼顾读写两侧的缓存局部性。
 */
void Transpose(const ComplexMatrix& input, ComplexMatrix* output) {
  output->rows = input.cols;
  output->cols = input.rows;
  output->values.resize(input.values.size());
  const std::size_t block = 64U;
  for (std::size_t row_begin = 0U; row_begin < input.rows; row_begin += block) {
    const std::size_t row_end = std::min(row_begin + block, input.rows);
    for (std::size_t col_begin = 0U; col_begin < input.cols; col_begin += block) {
      const std::size_t col_end = std::min(col_begin + block, input.cols);
      for (std::size_t row = row_begin; row < row_end; ++row) {
        for (std::size_t col = col_begin; col < col_end; ++col) {
          (*output)(col, row) = input(row, col);
        }
      }
    }
  }
}

}  // namespace

ComplexSample& ComplexMatrix::operator()(std::size_t row, std::size_t col) {
  return values[row * cols + col];
}

const ComplexSample& ComplexMatrix::operator()(std::size_t row, std::size_t col) const {
  return values[row * cols + col];
}

bool Fft1D(const ComplexVector& input, bool inverse, ComplexVector* output) {
  if (output == nullptr || input.empty()) {
    return false;
  }

  Eigen::FFT<double>& fft = GetFftPlanCache().GetPlan(input.size());
  if (inverse) {
    fft.inv(*output, input);
  } else {
    fft.fwd(*output, input);
  }
  return output->size() == input.size();
}

bool FftRows(const ComplexMatrix& input, bool inverse, ComplexMatrix* output) {
  if (output == nullptr || !HasValidShape(input)) {
    return false;
  }

  output->rows = input.rows;
  output->cols = input.cols;
  output->values.assign(input.values.size(), ComplexSample(0.0, 0.0));

  ComplexVector row_input(input.cols);
  ComplexVector row_output;
  row_output.reserve(input.cols);
  for (std::size_t row = 0; row < input.rows; ++row) {
    const ComplexSample* source = input.values.data() + row * input.cols;
    std::copy(source, source + input.cols, row_input.begin());
    if (!Fft1D(row_input, inverse, &row_output) || row_output.size() != input.cols) {
      return false;
    }
    std::copy(row_output.begin(), row_output.end(), output->values.begin() + row * input.cols);
  }
  return true;
}

bool FftCols(const ComplexMatrix& input, bool inverse, ComplexMatrix* output) {
  if (output == nullptr || !HasValidShape(input)) {
    return false;
  }

  // 转置 → 逐行 FFT（列方向数据变为连续内存）→ 转置回原布局。
  // 与旧实现（逐列 gather/scatter）数值逐位一致：FFT 蝶形顺序与数据序均不变。
  ComplexMatrix transposed;
  Transpose(input, &transposed);
  ComplexMatrix transformed;
  if (!FftRows(transposed, inverse, &transformed)) {
    return false;
  }
  ComplexMatrix result;
  Transpose(transformed, &result);
  *output = std::move(result);
  return true;
}

}  // namespace signal
}  // namespace sar
