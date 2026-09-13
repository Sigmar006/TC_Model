# CMake 架构

本目录按 ownership 分层：顶层负责 project 生命周期，`src/` 负责产品装配，
模块目录负责自己的 object target、直接依赖和 replay schema。编译选项始终以
target 为粒度应用，不污染 vendor、tests、examples 或 tools。

## 目录

```text
cmake/
├── compilers/                         编译器族 target option helpers
├── features/                          ccache、format/tidy、coverage、PCH、unity helpers
└── project/
    ├── ProjectOptions.cmake           build/module option boundary
    ├── ProjectSetup.cmake             prepare/configure lifecycle entry points
    ├── ProjectDependencies.cmake      oneq_find_project_dependencies()
    ├── ProjectTargets.cmake           component registry + target policy/public library helpers
    ├── ProjectInstall.cmake           export/package/public-tree installation
    ├── ProjectSummary.cmake           final configuration summary
    ├── codegen/
    │   └── FlatBuffers.cmake           flatc discovery + module-owned schema API
    ├── dependencies/
    │   ├── ConanPackages.cmake        imported package discovery
    │   └── JsbsimProvider.cmake       JSBSim provider resolution
```

## 主调用链

```text
CMakeLists.txt
  ├── oneq_prepare_project() / project() / oneq_configure_project()
  ├── setup owns language/options/tooling/compiler/project APIs and dependency discovery
  ├── add_subdirectory(src)
  │     ├── create 1q_lib / 1q::core
  │     ├── add_subdirectory(<module>)
  │     └── link registered object components into 1q_lib
  ├── oneq_install_project()
  └── tests / examples / tools
```

每个 `src/<module>/CMakeLists.txt` 直接声明 source、component target、direct
dependency 和 schema。`src/CMakeLists.txt` 不维护 module source list、link matrix
或 replay schema manifest。

## 约束

- `1q::core` 和 `1q_lib` 是稳定的产品/导出 target；不得作为普通 cache option 覆写。
- `PACKAGE_MANAGER=conan` 是唯一受支持的 provider。先运行
  `scripts/bootstrap_conan.sh <preset>`，再运行 `cmake --preset <preset>`。
- `ONEQ_ENABLE_FLIGHT_DYNAMIC` 控制机动模块，同时决定是否引入 JSBSim。
  JSBSim 是 flight_dynamic 专属依赖：关闭该模块（默认）时 core/common 完全不依赖
  JSBSim，JsbsimProvider 不会执行 `find_package(jsbsim)`。
- FlatBuffers schema 与其 codec owner target 在同一模块声明；新增 schema 不更新中心清单。
- `include/1q` 是唯一 public header 物理边界。`ProjectInstall.cmake` 镜像安装该目录，
  `check_public_api_boundary` 与 `check_install_manifest` 共同守护它。
- compiler/feature helper 只定义函数或显式工具入口；禁止目录级
  `add_compile_options()` / `add_link_options()`。

## 验证入口

- `cmake --preset llvm-ninja-debug`
- `cmake --build --preset llvm-ninja-debug --target 1q_unit_tests`
- `ctest --test-dir build/llvm-ninja-debug -L contract --output-on-failure`
- `tests/contract/check_cmake_helper_parse.cmake`：两套 compiler helper 均可解析。
- `tests/contract/check_preset_provider_contract.cmake`：public preset 与 bootstrap/provider 一致。
