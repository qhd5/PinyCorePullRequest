# pid
file(GLOB_RECURSE UTIL_SOURCES CMAKE_CONFIGURE_DEPENDS
     "../../Src/Utils/Mathematica/PID/*.cpp")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${UTIL_SOURCES} test_pid.cpp)
target_include_directories(${CMAKE_PROJECT_NAME}
                           PRIVATE ../../Src/Utils/Mathematica/PID)

add_test(NAME positional_pid COMMAND ${CMAKE_PROJECT_NAME} "[positional pid]")
add_test(NAME incremental_pid COMMAND ${CMAKE_PROJECT_NAME} "[incremental pid]")

# tinympc：QP 后端、补丁后的上游求解器、参考实现，以及 Matrix.hpp 依赖的
# CMSIS-DSP 矩阵内核，全部按宿主平台构建。
set(TINYMPC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../ThirdParty/TinyMPC")
set(EIGEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Lib/Eigen3")
set(QP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Src/Utils/Mathematica/QP")
set(MATRIX_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Src/Utils/Mathematica/Matrix")
set(CMSIS_DSP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../ThirdParty/CMSIS-DSP")

include("${CMAKE_CURRENT_SOURCE_DIR}/../../tools/cmake/FindLib/TinyMpcPatch.cmake")
piny_patch_tinympc("${TINYMPC_DIR}"
                   "${CMAKE_CURRENT_BINARY_DIR}/tinympc-patched" TINYMPC_PATCHED)

# 上游源文件的唯一归属：封装层与参考实现共用同一份补丁构建，与固件库保持一致。
add_library(tinympc_ref STATIC
            tinympc_ref.cpp
            "${QP_DIR}/TinyMpcBackend.cpp"
            ${TINYMPC_PATCHED}
            "${TINYMPC_DIR}/src/tinympc/rho_benchmark.cpp")
target_include_directories(tinympc_ref
                           PRIVATE "${QP_DIR}" "${TINYMPC_DIR}/src/tinympc")
target_include_directories(tinympc_ref SYSTEM
                           PRIVATE "${TINYMPC_DIR}/src" "${EIGEN_DIR}")
# 上游仍使用已弃用的 Eigen::placeholders::last，保持宿主构建输出干净。
target_compile_options(tinympc_ref PRIVATE -Wno-deprecated-declarations)
if(PINY_TINYMPC_ENABLE_SOLVE_LOG)
  target_compile_definitions(tinympc_ref PUBLIC PINY_TINYMPC_ENABLE_SOLVE_LOG)
endif()

# Matrix.hpp 只有头文件，宿主构建需要它调用的 CMSIS-DSP 内核；
# __GNUC_PYTHON__ 选择其可移植的 C 实现。
set(CMSIS_MATRIX_SOURCES
    "${CMSIS_DSP_DIR}/Source/MatrixFunctions/arm_mat_init_f32.c"
    "${CMSIS_DSP_DIR}/Source/MatrixFunctions/arm_mat_add_f32.c"
    "${CMSIS_DSP_DIR}/Source/MatrixFunctions/arm_mat_sub_f32.c"
    "${CMSIS_DSP_DIR}/Source/MatrixFunctions/arm_mat_scale_f32.c"
    "${CMSIS_DSP_DIR}/Source/MatrixFunctions/arm_mat_mult_f32.c"
    "${CMSIS_DSP_DIR}/Source/MatrixFunctions/arm_mat_trans_f32.c"
    "${CMSIS_DSP_DIR}/Source/MatrixFunctions/arm_mat_inverse_f32.c")

target_sources(${CMAKE_PROJECT_NAME}
               PRIVATE test_tinympc.cpp ${CMSIS_MATRIX_SOURCES})
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE __GNUC_PYTHON__)
target_include_directories(${CMAKE_PROJECT_NAME}
                           PRIVATE "${QP_DIR}" "${MATRIX_DIR}")
target_include_directories(${CMAKE_PROJECT_NAME} SYSTEM
                           PRIVATE "${CMSIS_DSP_DIR}/Include"
                                   "${CMSIS_DSP_DIR}/PrivateInclude")
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE tinympc_ref)

add_test(NAME tinympc COMMAND ${CMAKE_PROJECT_NAME} "[tinympc]")
