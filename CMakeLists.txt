project(drivebrain_software)
cmake_minimum_required(VERSION 3.24)

find_package(foxglove-schemas_proto_cpp CONFIG REQUIRED)
find_package(foxglove_websocket REQUIRED)

# add_library(hytech_param_server SHARED)

# target_include_directories(hytech_param_server PUBLIC 
#     $<INSTALL_INTERFACE:include/drivebrain_comms/foxglove_param_server>
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/drivebrain_comms/foxglove_param_server>    
# )

add_executable(test_param_server test/test_param_server.cpp)

target_link_libraries(test_param_server PUBLIC foxglove-schemas_proto_cpp::foxglove-schemas_proto_cpp foxglove_websocket::foxglove_websocket)
install(TARGETS test_param_server)