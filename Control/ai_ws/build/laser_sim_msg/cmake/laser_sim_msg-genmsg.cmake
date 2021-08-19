# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "laser_sim_msg: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ilaser_sim_msg:/home/labdog/ai_ws/src/laser_sim_msg/msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(laser_sim_msg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/labdog/ai_ws/src/laser_sim_msg/msg/cmd_vel.msg" NAME_WE)
add_custom_target(_laser_sim_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "laser_sim_msg" "/home/labdog/ai_ws/src/laser_sim_msg/msg/cmd_vel.msg" ""
)

get_filename_component(_filename "/home/labdog/ai_ws/src/laser_sim_msg/msg/point_cloud.msg" NAME_WE)
add_custom_target(_laser_sim_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "laser_sim_msg" "/home/labdog/ai_ws/src/laser_sim_msg/msg/point_cloud.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(laser_sim_msg
  "/home/labdog/ai_ws/src/laser_sim_msg/msg/cmd_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/laser_sim_msg
)
_generate_msg_cpp(laser_sim_msg
  "/home/labdog/ai_ws/src/laser_sim_msg/msg/point_cloud.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/laser_sim_msg
)

### Generating Services

### Generating Module File
_generate_module_cpp(laser_sim_msg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/laser_sim_msg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(laser_sim_msg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(laser_sim_msg_generate_messages laser_sim_msg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/labdog/ai_ws/src/laser_sim_msg/msg/cmd_vel.msg" NAME_WE)
add_dependencies(laser_sim_msg_generate_messages_cpp _laser_sim_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/labdog/ai_ws/src/laser_sim_msg/msg/point_cloud.msg" NAME_WE)
add_dependencies(laser_sim_msg_generate_messages_cpp _laser_sim_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(laser_sim_msg_gencpp)
add_dependencies(laser_sim_msg_gencpp laser_sim_msg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS laser_sim_msg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(laser_sim_msg
  "/home/labdog/ai_ws/src/laser_sim_msg/msg/cmd_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/laser_sim_msg
)
_generate_msg_eus(laser_sim_msg
  "/home/labdog/ai_ws/src/laser_sim_msg/msg/point_cloud.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/laser_sim_msg
)

### Generating Services

### Generating Module File
_generate_module_eus(laser_sim_msg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/laser_sim_msg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(laser_sim_msg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(laser_sim_msg_generate_messages laser_sim_msg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/labdog/ai_ws/src/laser_sim_msg/msg/cmd_vel.msg" NAME_WE)
add_dependencies(laser_sim_msg_generate_messages_eus _laser_sim_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/labdog/ai_ws/src/laser_sim_msg/msg/point_cloud.msg" NAME_WE)
add_dependencies(laser_sim_msg_generate_messages_eus _laser_sim_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(laser_sim_msg_geneus)
add_dependencies(laser_sim_msg_geneus laser_sim_msg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS laser_sim_msg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(laser_sim_msg
  "/home/labdog/ai_ws/src/laser_sim_msg/msg/cmd_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/laser_sim_msg
)
_generate_msg_lisp(laser_sim_msg
  "/home/labdog/ai_ws/src/laser_sim_msg/msg/point_cloud.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/laser_sim_msg
)

### Generating Services

### Generating Module File
_generate_module_lisp(laser_sim_msg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/laser_sim_msg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(laser_sim_msg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(laser_sim_msg_generate_messages laser_sim_msg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/labdog/ai_ws/src/laser_sim_msg/msg/cmd_vel.msg" NAME_WE)
add_dependencies(laser_sim_msg_generate_messages_lisp _laser_sim_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/labdog/ai_ws/src/laser_sim_msg/msg/point_cloud.msg" NAME_WE)
add_dependencies(laser_sim_msg_generate_messages_lisp _laser_sim_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(laser_sim_msg_genlisp)
add_dependencies(laser_sim_msg_genlisp laser_sim_msg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS laser_sim_msg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(laser_sim_msg
  "/home/labdog/ai_ws/src/laser_sim_msg/msg/cmd_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/laser_sim_msg
)
_generate_msg_nodejs(laser_sim_msg
  "/home/labdog/ai_ws/src/laser_sim_msg/msg/point_cloud.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/laser_sim_msg
)

### Generating Services

### Generating Module File
_generate_module_nodejs(laser_sim_msg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/laser_sim_msg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(laser_sim_msg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(laser_sim_msg_generate_messages laser_sim_msg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/labdog/ai_ws/src/laser_sim_msg/msg/cmd_vel.msg" NAME_WE)
add_dependencies(laser_sim_msg_generate_messages_nodejs _laser_sim_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/labdog/ai_ws/src/laser_sim_msg/msg/point_cloud.msg" NAME_WE)
add_dependencies(laser_sim_msg_generate_messages_nodejs _laser_sim_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(laser_sim_msg_gennodejs)
add_dependencies(laser_sim_msg_gennodejs laser_sim_msg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS laser_sim_msg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(laser_sim_msg
  "/home/labdog/ai_ws/src/laser_sim_msg/msg/cmd_vel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_sim_msg
)
_generate_msg_py(laser_sim_msg
  "/home/labdog/ai_ws/src/laser_sim_msg/msg/point_cloud.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_sim_msg
)

### Generating Services

### Generating Module File
_generate_module_py(laser_sim_msg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_sim_msg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(laser_sim_msg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(laser_sim_msg_generate_messages laser_sim_msg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/labdog/ai_ws/src/laser_sim_msg/msg/cmd_vel.msg" NAME_WE)
add_dependencies(laser_sim_msg_generate_messages_py _laser_sim_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/labdog/ai_ws/src/laser_sim_msg/msg/point_cloud.msg" NAME_WE)
add_dependencies(laser_sim_msg_generate_messages_py _laser_sim_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(laser_sim_msg_genpy)
add_dependencies(laser_sim_msg_genpy laser_sim_msg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS laser_sim_msg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/laser_sim_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/laser_sim_msg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/laser_sim_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/laser_sim_msg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/laser_sim_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/laser_sim_msg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/laser_sim_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/laser_sim_msg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_sim_msg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_sim_msg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_sim_msg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
