# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "motion_planner: 2 messages, 0 services")

set(MSG_I_FLAGS "-Imotion_planner:/home/vialab/car_ws/src/motion_planner/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(motion_planner_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/vialab/car_ws/src/motion_planner/msg/State.msg" NAME_WE)
add_custom_target(_motion_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motion_planner" "/home/vialab/car_ws/src/motion_planner/msg/State.msg" ""
)

get_filename_component(_filename "/home/vialab/car_ws/src/motion_planner/msg/Local_path.msg" NAME_WE)
add_custom_target(_motion_planner_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "motion_planner" "/home/vialab/car_ws/src/motion_planner/msg/Local_path.msg" "motion_planner/State:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(motion_planner
  "/home/vialab/car_ws/src/motion_planner/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motion_planner
)
_generate_msg_cpp(motion_planner
  "/home/vialab/car_ws/src/motion_planner/msg/Local_path.msg"
  "${MSG_I_FLAGS}"
  "/home/vialab/car_ws/src/motion_planner/msg/State.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motion_planner
)

### Generating Services

### Generating Module File
_generate_module_cpp(motion_planner
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motion_planner
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(motion_planner_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(motion_planner_generate_messages motion_planner_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vialab/car_ws/src/motion_planner/msg/State.msg" NAME_WE)
add_dependencies(motion_planner_generate_messages_cpp _motion_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vialab/car_ws/src/motion_planner/msg/Local_path.msg" NAME_WE)
add_dependencies(motion_planner_generate_messages_cpp _motion_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motion_planner_gencpp)
add_dependencies(motion_planner_gencpp motion_planner_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motion_planner_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(motion_planner
  "/home/vialab/car_ws/src/motion_planner/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motion_planner
)
_generate_msg_eus(motion_planner
  "/home/vialab/car_ws/src/motion_planner/msg/Local_path.msg"
  "${MSG_I_FLAGS}"
  "/home/vialab/car_ws/src/motion_planner/msg/State.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motion_planner
)

### Generating Services

### Generating Module File
_generate_module_eus(motion_planner
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motion_planner
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(motion_planner_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(motion_planner_generate_messages motion_planner_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vialab/car_ws/src/motion_planner/msg/State.msg" NAME_WE)
add_dependencies(motion_planner_generate_messages_eus _motion_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vialab/car_ws/src/motion_planner/msg/Local_path.msg" NAME_WE)
add_dependencies(motion_planner_generate_messages_eus _motion_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motion_planner_geneus)
add_dependencies(motion_planner_geneus motion_planner_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motion_planner_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(motion_planner
  "/home/vialab/car_ws/src/motion_planner/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motion_planner
)
_generate_msg_lisp(motion_planner
  "/home/vialab/car_ws/src/motion_planner/msg/Local_path.msg"
  "${MSG_I_FLAGS}"
  "/home/vialab/car_ws/src/motion_planner/msg/State.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motion_planner
)

### Generating Services

### Generating Module File
_generate_module_lisp(motion_planner
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motion_planner
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(motion_planner_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(motion_planner_generate_messages motion_planner_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vialab/car_ws/src/motion_planner/msg/State.msg" NAME_WE)
add_dependencies(motion_planner_generate_messages_lisp _motion_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vialab/car_ws/src/motion_planner/msg/Local_path.msg" NAME_WE)
add_dependencies(motion_planner_generate_messages_lisp _motion_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motion_planner_genlisp)
add_dependencies(motion_planner_genlisp motion_planner_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motion_planner_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(motion_planner
  "/home/vialab/car_ws/src/motion_planner/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motion_planner
)
_generate_msg_nodejs(motion_planner
  "/home/vialab/car_ws/src/motion_planner/msg/Local_path.msg"
  "${MSG_I_FLAGS}"
  "/home/vialab/car_ws/src/motion_planner/msg/State.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motion_planner
)

### Generating Services

### Generating Module File
_generate_module_nodejs(motion_planner
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motion_planner
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(motion_planner_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(motion_planner_generate_messages motion_planner_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vialab/car_ws/src/motion_planner/msg/State.msg" NAME_WE)
add_dependencies(motion_planner_generate_messages_nodejs _motion_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vialab/car_ws/src/motion_planner/msg/Local_path.msg" NAME_WE)
add_dependencies(motion_planner_generate_messages_nodejs _motion_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motion_planner_gennodejs)
add_dependencies(motion_planner_gennodejs motion_planner_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motion_planner_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(motion_planner
  "/home/vialab/car_ws/src/motion_planner/msg/State.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motion_planner
)
_generate_msg_py(motion_planner
  "/home/vialab/car_ws/src/motion_planner/msg/Local_path.msg"
  "${MSG_I_FLAGS}"
  "/home/vialab/car_ws/src/motion_planner/msg/State.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motion_planner
)

### Generating Services

### Generating Module File
_generate_module_py(motion_planner
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motion_planner
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(motion_planner_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(motion_planner_generate_messages motion_planner_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vialab/car_ws/src/motion_planner/msg/State.msg" NAME_WE)
add_dependencies(motion_planner_generate_messages_py _motion_planner_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vialab/car_ws/src/motion_planner/msg/Local_path.msg" NAME_WE)
add_dependencies(motion_planner_generate_messages_py _motion_planner_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(motion_planner_genpy)
add_dependencies(motion_planner_genpy motion_planner_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motion_planner_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motion_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motion_planner
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(motion_planner_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motion_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/motion_planner
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(motion_planner_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motion_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motion_planner
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(motion_planner_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motion_planner)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/motion_planner
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(motion_planner_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motion_planner)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motion_planner\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motion_planner
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(motion_planner_generate_messages_py std_msgs_generate_messages_py)
endif()
