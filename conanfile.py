from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, cmake_layout

_BASE_DEPS = {
    "eigen": "eigen/3.4.0",
    "boost": "boost/1.85.0",
    "nanoflann": "nanoflann/1.3.2",
    "flatbuffers": "flatbuffers/1.12.0",
    "zlib": "zlib/1.3.1",
    "sqlite3": "sqlite3/3.53.4",
}

_JSBSIM_DEPS_NON_WINDOWS = {
    "jsbsim": "jsbsim/1.3.1",
}

_LOG_DEPS_NON_WINDOWS = {
    "spdlog": "spdlog/1.12.0",
    "fmt": "fmt/10.2.1",
}

_GTEST_VERSION = "gtest/1.12.1"

# 属性测试库，仅测试期依赖（与 gtest 同门控）。ConanCenter 配方只安装上游
# include/，不安装 extras/，因此 rapidcheck/gtest.h 需仓库自带适配头：
# tests/support/rapidcheck_gtest.h。配方的 enable_gtest 选项保持默认关闭，
# 避免把 rapidcheck 的 gtest 组件耦合进依赖图。
_RAPIDCHECK_VERSION = "rapidcheck/cci.20231215"


class OneQConan(ConanFile):
    name = "1q"
    version = "0.1"
    settings = "os", "compiler", "build_type", "arch"

    options = {
        "enable_testing": [True, False],
    }

    default_options = {
        "enable_testing": False,
        # 显式固定第三方链接形态，避免依赖 recipe 默认值导致构建行为漂移。
        "spdlog/*:shared": False,
        "fmt/*:shared": False,
        "gtest/*:shared": False,
        "zlib/*:shared": False,
        "sqlite3/*:shared": False,
        "boost/*:header_only": True,
        "flatbuffers/*:header_only": True,
    }

    def _is_windows(self):
        return str(self.settings.os) == "Windows"

    def requirements(self):
        self.requires(_BASE_DEPS["eigen"])
        self.requires(_BASE_DEPS["boost"])
        self.requires(_BASE_DEPS["nanoflann"])
        self.requires(_BASE_DEPS["flatbuffers"])
        self.requires(_BASE_DEPS["zlib"])
        self.requires(_BASE_DEPS["sqlite3"])

        # macOS/Linux 保留调试日志能力；Windows 关闭日志第三方依赖（库内由
        # 内置文件日志后端 ProjectFileLog 承载，ONEQ_ENABLE_FILE_LOG 门控）。
        if not self._is_windows():
            self.requires(_LOG_DEPS_NON_WINDOWS["spdlog"])
            self.requires(_LOG_DEPS_NON_WINDOWS["fmt"], override=True)
            # macOS 开发使用 conan 预编译的 JSBSim（Windows/VS2015 从 third_party 源码构建）
            self.requires(_JSBSIM_DEPS_NON_WINDOWS["jsbsim"])

            # SAR HDF5 输出（始终开启；仅 C++17+ 构建实际启用，见 ConanPackages.cmake）。
            # Windows C++11/14 构建（v141 老工具集）不安装：HighFive/HDF5 要求 C++17。
            self.requires("highfive/2.10.0")

    def build_requirements(self):
        if self.options.enable_testing:
            self.test_requires(_GTEST_VERSION)
            # rapidcheck 暂不纳入依赖图：ConanCenter 配方在本机 Windows + VS 生成器下
            # `cmake.configure()` 失败（生成器未被配方转发），一旦开启 enable_testing
            # 即整条依赖安装失败，进而 tests/ 无法配置。
            # 属性测试源（common_property_test / precision_evaluation_metrics_boundary_test）
            # 已按 `__has_include("rapidcheck.h")` 降级：无依赖时属性用例跳过、边界用例照跑
            # （见 docs/review/acceptance_log_remaining_work_guide.md §5）。
            # 若需恢复属性测试，先修配方生成器转发或改走自建包，再恢复下一行。
            # self.test_requires(_RAPIDCHECK_VERSION)

    def generate(self):
        CMakeDeps(self).generate()
        tc = CMakeToolchain(self)
        tc.generate()

    def layout(self):
        cmake_layout(self)
