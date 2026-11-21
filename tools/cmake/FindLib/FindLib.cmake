include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
set(FETCHCONTENT_BASE_DIR "${CMAKE_SOURCE_DIR}/.deps")

set(THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/ThirdParty")

message("-------------------- find lib -----------------------")

include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/FreeRTOS.cmake")
include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/Segger.cmake")
include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/CMSIS_DSP.cmake")
if(USE_MATHEMATICA AND USE_MATHEMATICA_QP)
  # TinyMPC 只被 QP 模块使用；未启用时不下载、不编译 Eigen 与求解器。
  include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/TinyMPC.cmake")
endif()
include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/etl.cmake")
include("${CMAKE_SOURCE_DIR}/tools/cmake/FindLib/magicEnum.cmake")
