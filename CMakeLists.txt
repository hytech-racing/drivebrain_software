project(drivebrain_software)


find_package(foxglove-schemas_proto_cpp CONFIG REQUIRED)
# find_package(foxglove-websocket REQUIRED) this is where this would be, IF WE HAD ONE ...

# add_library(hytech_param_server SHARED)

# target_include_directories(hytech_param_server PUBLIC 
#     $<INSTALL_INTERFACE:include/drivebrain_comms/foxglove_param_server>
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/drivebrain_comms/foxglove_param_server>    
# )

add_executable(test_param_server test/test_param_server.cpp)

# TODO(ben) get / package the foxglove websocket boi properly

target_link_libraries(test_param_server PUBLIC foxglove-schemas_proto_cpp::foxglove-schemas_proto_cpp)
