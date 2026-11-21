include("${CMAKE_CURRENT_LIST_DIR}/TinyMpcPatch.cmake")

# 源码目录被删除后，清理失效的下载标记。
if(NOT EXISTS "${THIRD_PARTY_DIR}/TinyMPC/CMakeLists.txt")
  file(REMOVE_RECURSE "${FETCHCONTENT_BASE_DIR}/tinympc-subbuild")
endif()

FetchContent_Declare(
  tinympc
  GIT_REPOSITORY https://github.com/TinyMPC/TinyMPC.git
  # 按提交固定版本，无法浅克隆，因此不设置 GIT_SHALLOW。
  GIT_TAG ${PINY_TINYMPC_REVISION}
  GIT_PROGRESS TRUE
  SOURCE_DIR "${THIRD_PARTY_DIR}/TinyMPC"
  SOURCE_SUBDIR src/tinympc)

FetchContent_MakeAvailable(tinympc)

# 仅构建求解器，排除仅供主机使用的代码生成工具和示例程序。
get_target_property(TINYMPC_SOURCES tinympcstatic SOURCES)
piny_patch_tinympc("${tinympc_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/tinympc-patched" TINYMPC_PATCHED)
list(REMOVE_ITEM TINYMPC_SOURCES codegen.cpp tiny_api.cpp admm.cpp)
list(APPEND TINYMPC_SOURCES ${TINYMPC_PATCHED})
# 统一使用框架自带的 Eigen（Lib/Eigen3），避免上游捆绑的开发版进入同一编译单元；
# 上游的 src 与 src/tinympc 仍是接口包含目录。
set(PINY_TINYMPC_INCLUDES
  "${CMAKE_SOURCE_DIR}/Lib/Eigen3"
  "${tinympc_SOURCE_DIR}/src"
  "${tinympc_SOURCE_DIR}/src/tinympc")
set_target_properties(tinympcstatic PROPERTIES
  INCLUDE_DIRECTORIES "${PINY_TINYMPC_INCLUDES}"
  INTERFACE_INCLUDE_DIRECTORIES "${PINY_TINYMPC_INCLUDES}")
if(PINY_TINYMPC_ENABLE_SOLVE_LOG)
  target_compile_definitions(tinympcstatic PRIVATE PINY_TINYMPC_ENABLE_SOLVE_LOG)
endif()
set_target_properties(tinympcstatic PROPERTIES
  SOURCES "${TINYMPC_SOURCES}"
  POSITION_INDEPENDENT_CODE OFF)

# 仅对受影响的上游源文件关闭未使用变量警告；
# 上游仍使用已弃用的 Eigen::placeholders::last，一并不再提示。
set_property(SOURCE "${CMAKE_CURRENT_BINARY_DIR}/tinympc-patched/admm.cpp"
  TARGET_DIRECTORY tinympcstatic
  APPEND PROPERTY COMPILE_OPTIONS -Wno-unused-variable -Wno-deprecated-declarations)

# 仅对大量使用 Eigen 的上游源文件关闭 ABI 提示。
set_property(SOURCE
  "${tinympc_SOURCE_DIR}/src/tinympc/rho_benchmark.cpp"
  "${CMAKE_CURRENT_BINARY_DIR}/tinympc-patched/tiny_api.cpp"
  TARGET_DIRECTORY tinympcstatic
  APPEND PROPERTY COMPILE_OPTIONS -Wno-psabi)
