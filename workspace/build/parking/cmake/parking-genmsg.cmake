# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "parking: 2 messages, 1 services")

set(MSG_I_FLAGS "-Iparking:/home/mpc/Multi_Car_Parking/workspace/src/parking/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(parking_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg" NAME_WE)
add_custom_target(_parking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "parking" "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg" ""
)

get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg" NAME_WE)
add_custom_target(_parking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "parking" "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg" ""
)

get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/srv/maneuver.srv" NAME_WE)
add_custom_target(_parking_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "parking" "/home/mpc/Multi_Car_Parking/workspace/src/parking/srv/maneuver.srv" "parking/car_input:parking/car_state"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/parking
)
_generate_msg_cpp(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/parking
)

### Generating Services
_generate_srv_cpp(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/srv/maneuver.srv"
  "${MSG_I_FLAGS}"
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg;/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/parking
)

### Generating Module File
_generate_module_cpp(parking
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/parking
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(parking_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(parking_generate_messages parking_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg" NAME_WE)
add_dependencies(parking_generate_messages_cpp _parking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg" NAME_WE)
add_dependencies(parking_generate_messages_cpp _parking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/srv/maneuver.srv" NAME_WE)
add_dependencies(parking_generate_messages_cpp _parking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(parking_gencpp)
add_dependencies(parking_gencpp parking_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS parking_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/parking
)
_generate_msg_eus(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/parking
)

### Generating Services
_generate_srv_eus(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/srv/maneuver.srv"
  "${MSG_I_FLAGS}"
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg;/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/parking
)

### Generating Module File
_generate_module_eus(parking
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/parking
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(parking_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(parking_generate_messages parking_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg" NAME_WE)
add_dependencies(parking_generate_messages_eus _parking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg" NAME_WE)
add_dependencies(parking_generate_messages_eus _parking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/srv/maneuver.srv" NAME_WE)
add_dependencies(parking_generate_messages_eus _parking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(parking_geneus)
add_dependencies(parking_geneus parking_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS parking_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/parking
)
_generate_msg_lisp(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/parking
)

### Generating Services
_generate_srv_lisp(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/srv/maneuver.srv"
  "${MSG_I_FLAGS}"
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg;/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/parking
)

### Generating Module File
_generate_module_lisp(parking
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/parking
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(parking_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(parking_generate_messages parking_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg" NAME_WE)
add_dependencies(parking_generate_messages_lisp _parking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg" NAME_WE)
add_dependencies(parking_generate_messages_lisp _parking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/srv/maneuver.srv" NAME_WE)
add_dependencies(parking_generate_messages_lisp _parking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(parking_genlisp)
add_dependencies(parking_genlisp parking_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS parking_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/parking
)
_generate_msg_nodejs(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/parking
)

### Generating Services
_generate_srv_nodejs(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/srv/maneuver.srv"
  "${MSG_I_FLAGS}"
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg;/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/parking
)

### Generating Module File
_generate_module_nodejs(parking
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/parking
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(parking_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(parking_generate_messages parking_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg" NAME_WE)
add_dependencies(parking_generate_messages_nodejs _parking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg" NAME_WE)
add_dependencies(parking_generate_messages_nodejs _parking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/srv/maneuver.srv" NAME_WE)
add_dependencies(parking_generate_messages_nodejs _parking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(parking_gennodejs)
add_dependencies(parking_gennodejs parking_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS parking_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/parking
)
_generate_msg_py(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/parking
)

### Generating Services
_generate_srv_py(parking
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/srv/maneuver.srv"
  "${MSG_I_FLAGS}"
  "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg;/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/parking
)

### Generating Module File
_generate_module_py(parking
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/parking
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(parking_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(parking_generate_messages parking_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_state.msg" NAME_WE)
add_dependencies(parking_generate_messages_py _parking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/msg/car_input.msg" NAME_WE)
add_dependencies(parking_generate_messages_py _parking_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mpc/Multi_Car_Parking/workspace/src/parking/srv/maneuver.srv" NAME_WE)
add_dependencies(parking_generate_messages_py _parking_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(parking_genpy)
add_dependencies(parking_genpy parking_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS parking_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/parking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/parking
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(parking_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/parking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/parking
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(parking_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/parking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/parking
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(parking_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/parking)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/parking
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(parking_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/parking)
  install(CODE "execute_process(COMMAND \"/home/mpc/Envs/parking/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/parking\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/parking
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(parking_generate_messages_py std_msgs_generate_messages_py)
endif()
