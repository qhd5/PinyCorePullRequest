# 为固定版本的 TinyMPC 提供共用且可复现的源码补丁。
# 修正上游在 Riccati 预计算中重复累加 rho 的问题（上游 main 至今未修复，
# 相关讨论见 https://github.com/TinyMPC/TinyMPC/issues/79），
# 推导见 Src/Utils/Mathematica/QP/TinyMPC论文解读与完整推导.md。
set(PINY_TINYMPC_REVISION 023f36bd5b27267e1a96ed47a774d120b5707f13)
option(PINY_TINYMPC_ENABLE_SOLVE_LOG "Print a message on each successful MPC solve" OFF)

function(piny_patch_tinympc source_dir output_dir output_sources)
  find_package(Git REQUIRED)
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" -C "${source_dir}" rev-parse HEAD
    OUTPUT_VARIABLE revision OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE status)
  if(NOT status EQUAL 0 OR NOT "${revision}" STREQUAL "${PINY_TINYMPC_REVISION}")
    message(FATAL_ERROR
      "TinyMPC 需要固定在提交 ${PINY_TINYMPC_REVISION}，当前为 '${revision}'。\n"
      "已存在的检出不会自动切换版本，请删除 ${source_dir} 后重新配置。")
  endif()

  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${source_dir}/src/tinympc/tiny_api.cpp"
    "${source_dir}/src/tinympc/admm.cpp")
  file(MAKE_DIRECTORY "${output_dir}")
  foreach(name IN ITEMS tiny_api admm)
    file(READ "${source_dir}/src/tinympc/${name}.cpp" content)
    if(name STREQUAL "tiny_api")
      # 保留原始阶段权重；预计算时会添加一次 ADMM 惩罚项。
      set(before "work->Q = (Q + rho * tinyMatrix::Identity(nx, nx)).diagonal();")
      set(after "work->Q = Q.diagonal();")
      string(FIND "${content}" "${before}" position)
      if(position EQUAL -1)
        message(FATAL_ERROR "TinyMPC Q patch no longer applies")
      endif()
      string(REPLACE "${before}" "${after}" content "${content}")
      set(before "work->R = (R + rho * tinyMatrix::Identity(nu, nu)).diagonal();")
      set(after "work->R = R.diagonal();")
    else()
      set(before "            std::cout << \"Solver converged in \" << solver->work->iter << \" iterations\" << std::endl;")
      set(after "#if defined(PINY_TINYMPC_ENABLE_SOLVE_LOG)\n${before}\n#endif")
    endif()
    string(FIND "${content}" "${before}" position)
    if(position EQUAL -1)
      message(FATAL_ERROR "TinyMPC ${name} patch no longer applies")
    endif()
    string(REPLACE "${before}" "${after}" content "${content}")
    # file(CONFIGURE) 仅在内容变化时更新输出文件的时间戳。
    file(CONFIGURE OUTPUT "${output_dir}/${name}.cpp" CONTENT "${content}" @ONLY)
    list(APPEND patched_sources "${output_dir}/${name}.cpp")
  endforeach()
  set(${output_sources} "${patched_sources}" PARENT_SCOPE)
endfunction()
