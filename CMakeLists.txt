cmake_minimum_required(VERSION 3.10)

project(MiniCheetah_rl_deploy)

set(CMAKE_BUILD_TYPE Release)
add_definitions(-w) # warning ignore
add_compile_options(-fPIC)
include(CTest)

# ========== 控制选项 ==========
set(BUILD_PLATFORM "x86" CACHE STRING "select build cpu type")
option(BUILD_SIM OFF)
option(USE_UDP_SIM "Use the UDP simulation adapter; supported with Mini Cheetah MuJoCo Python sim2sim" ON)
option(USE_RAISIM "Legacy simulator option; not validated for Mini Cheetah" OFF)
option(SEND_REMOTE OFF)
# option(USE_ONNX "Use ONNXRuntime" ON)
option(USE_MJCPP "Use Mujoco C++" OFF)
option(USE_CHEETAH_SOFTWARE_BRIDGE "Build real-hardware bridge against Cheetah-Software" OFF)



if (BUILD_PLATFORM STREQUAL arm)
  message("this is arm platform")
  set(BUILD_SIM OFF)
  set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
  set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 
else()
  message("this is x86 platform")
endif()
set(CMAKE_CXX_STANDARD 17)

if (SEND_REMOTE)
  set(BUILD_SIM OFF)
endif()

# ========== 显示参数 ==========
message("BUILD_PLATFORM: ${BUILD_PLATFORM}")
message("BUILD_SIM:      ${BUILD_SIM}")
message("SEND_REMOTE:    ${SEND_REMOTE}")
# message("USE_ONNX:       ${USE_ONNX}")
message("USE_MJCPP:      ${USE_MJCPP}")
message("USE_UDP_SIM:    ${USE_UDP_SIM}")

# 路径设置
get_filename_component(WORKSPACE_DIR ./ ABSOLUTE)
set(THIRD_PARTY ${WORKSPACE_DIR}/third_party)

# torch setting
# set(Torch_DIR "${THIRD_PARTY}/libtorch/${BUILD_PLATFORM}/share/cmake/Torch")
# find_package(Torch REQUIRED)
# message("Torch_DIR:  ${Torch_DIR}")

# ONNX
set(ONNX_RUNTIME_ROOT "${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}")
message("Using ONNXRuntime from: ${ONNX_RUNTIME_ROOT}")


# ONNX setting
# if (USE_ONNX)
#   add_definitions(-DUSE_ONNX)

#   include_directories(${ONNX_RUNTIME_ROOT}/include)
#   link_directories(${ONNX_RUNTIME_ROOT}/lib)
# endif()
include_directories(${ONNX_RUNTIME_ROOT}/include)
link_directories(${ONNX_RUNTIME_ROOT}/lib)


# MuJoCo
set(MUJOCO_ROOT "${THIRD_PARTY}/mujoco/${BUILD_PLATFORM}")
if (USE_MJCPP)
  message("USE_MJCPP enabled, loading MuJoCo headers from ${MUJOCO_ROOT}")
  add_definitions(-DUSE_MJCPP)
  include_directories(${MUJOCO_ROOT}/include)
  link_directories(${MUJOCO_ROOT}/lib)
endif()





# ========== Include 头文件路径 ==========
include_directories(
  ${THIRD_PARTY}/eigen/
  ${THIRD_PARTY}/gamepad/include
  types
  utils
  interface/robot
  interface/user_command
  state_machine
  run_policy
  # ${TORCH_INCLUDE_DIRS}
  ${ONNX_RUNTIME_ROOT}/include
  ${MUJOCO_ROOT}/include
)

# ========== 添加 interface 子模块 ==========
add_subdirectory(interface)

if (BUILD_SIM)
  if(USE_RAISIM)
    message(FATAL_ERROR "USE_RAISIM is a legacy simulator path and is not validated for Mini Cheetah. Use USE_UDP_SIM with mujoco_simulation.py, or USE_MJCPP after validating the C++ MuJoCo interface.")
  elseif(USE_UDP_SIM)
    add_definitions(-DUSE_UDP_SIM)
  elseif(USE_MJCPP)
    set(MUJOCO_ROOT "${THIRD_PARTY}/mujoco/${BUILD_PLATFORM}")
    message("USE_MJCPP enabled, loading MuJoCo headers from ${MUJOCO_ROOT}")
    add_definitions(-DUSE_MJCPP)
    include_directories(${MUJOCO_ROOT}/include)
    link_directories(${MUJOCO_ROOT}/lib)
  else()
    message(FATAL_ERROR "No supported simulation option selected. Use USE_UDP_SIM with mujoco_simulation.py, or USE_MJCPP after validating the C++ MuJoCo interface.")
  endif()
endif()

# ========== 设置部署路径 ==========
# 如果涉及到
if (SEND_REMOTE)
  set(PATH_LIB "../lib")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
  set(CMAKE_INSTALL_RPATH "${PATH_LIB}/onnxruntime/${BUILD_PLATFORM}/lib")
  # set(CMAKE_INSTALL_RPATH ${PATH_LIB}/libtorch/${BUILD_PLATFORM}/lib)
endif()

# ========== 快速链接器 (可选) ==========
# Install mold with: sudo apt install mold
# Then re-run cmake.  Falls back to system default if not found.
option(USE_MOLD_LINKER "Use mold linker for faster linking" OFF)
if(USE_MOLD_LINKER)
    find_program(MOLD_EXECUTABLE mold)
    if(MOLD_EXECUTABLE)
        add_link_options(-fuse-ld=mold)
        message("Using mold linker: ${MOLD_EXECUTABLE}")
    else()
        message(WARNING "USE_MOLD_LINKER=ON but mold not found; using default linker")
    endif()
endif()

# ========== run_policy 独立库 ==========
add_library(policy_metadata STATIC run_policy/policy_metadata.cpp)
target_include_directories(policy_metadata PUBLIC
    run_policy
    ${THIRD_PARTY}/eigen/
)

add_library(policy_entry_gate STATIC state_machine/policy_entry_gate.cpp)
target_include_directories(policy_entry_gate PUBLIC
    state_machine
    types
    ${THIRD_PARTY}/eigen/
)

add_library(simulation_packet_codec STATIC interface/robot/simulation/simulation_packet_codec.cpp)
target_include_directories(simulation_packet_codec PUBLIC
    interface/robot/simulation
    types
    ${THIRD_PARTY}/eigen/
)

add_library(episode_log STATIC logging/episode_log.cpp)
target_include_directories(episode_log PUBLIC
    logging
    types
    ${THIRD_PARTY}/eigen/
)

# Building the policy runner as a separate library means that editing the
# policy implementation only recompiles this one translation unit and
# re-links, without touching the state machine or main.cpp object files.
add_library(run_policy STATIC run_policy/mini_cheetah_policy_runner_onnx.cpp)
target_include_directories(run_policy PUBLIC
    run_policy
    ${THIRD_PARTY}/eigen/
    utils
    types
    ${ONNX_RUNTIME_ROOT}/include
)
target_link_libraries(run_policy policy_metadata onnxruntime)

# ========== 收集源文件 ==========
file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")
list(FILTER STATE_MACHINE_SRC EXCLUDE REGEX ".*/policy_entry_gate\\.cpp$")

add_executable(rl_deploy main.cpp ${STATE_MACHINE_SRC})

# ========== 链接基本库 ==========
if(BUILD_SIM)
    target_link_libraries(rl_deploy run_policy policy_entry_gate simulation_packet_codec interface dw -lpthread -lm -lrt -ldl -lstdc++)
else()
    target_link_libraries(rl_deploy run_policy policy_entry_gate simulation_packet_codec interface -lpthread -lm -lrt -ldl -lstdc++)
endif()

# if (USE_ONNX)
#   target_link_libraries(rl_deploy onnxruntime)
#   # target_link_libraries(rl_deploy torch)
# endif()

# MuJoCo

if (BUILD_SIM)
  if (USE_MJCPP)
    message("build simulation mujoco")
    target_link_libraries(rl_deploy mujoco glfw)
  endif()
endif()

if (USE_CHEETAH_SOFTWARE_BRIDGE)
  message(FATAL_ERROR "USE_CHEETAH_SOFTWARE_BRIDGE is reserved for the Mini Cheetah real-hardware bridge and is not implemented yet.")
endif()

if (BUILD_SIM)
    target_compile_definitions(rl_deploy PRIVATE BUILD_SIMULATION)
endif()

if (SEND_REMOTE)
  add_custom_target(push_to_robot ALL DEPENDS rl_deploy)
  add_custom_command(
    TARGET push_to_robot POST_BUILD
    COMMAND ./scripts/sftp_to_remote.sh
    WORKING_DIRECTORY ..
    COMMENT "run sftp_to_remote.sh"
  )
endif()

if (BUILD_TESTING)
  add_executable(policy_metadata_test tests/policy_metadata_test.cpp)
  target_include_directories(policy_metadata_test PRIVATE run_policy ${THIRD_PARTY}/eigen/)
  target_link_libraries(policy_metadata_test policy_metadata)
  add_test(NAME policy_metadata_test COMMAND policy_metadata_test)

  add_executable(policy_entry_gate_test tests/policy_entry_gate_test.cpp)
  target_include_directories(policy_entry_gate_test PRIVATE state_machine types ${THIRD_PARTY}/eigen/)
  target_link_libraries(policy_entry_gate_test policy_entry_gate)
  add_test(NAME policy_entry_gate_test COMMAND policy_entry_gate_test)

  add_executable(simulation_packet_codec_test tests/simulation_packet_codec_test.cpp)
  target_include_directories(simulation_packet_codec_test PRIVATE interface/robot/simulation types ${THIRD_PARTY}/eigen/)
  target_link_libraries(simulation_packet_codec_test simulation_packet_codec)
  add_test(NAME simulation_packet_codec_test COMMAND simulation_packet_codec_test)

  add_executable(episode_log_test tests/episode_log_test.cpp)
  target_include_directories(episode_log_test PRIVATE logging types ${THIRD_PARTY}/eigen/)
  target_link_libraries(episode_log_test episode_log)
  add_test(NAME episode_log_test COMMAND episode_log_test)
endif()
