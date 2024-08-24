cmake_minimum_required(VERSION 3.24)
project(drivebrain_software LANGUAGES CXX C)


# just like "include what you use" and do "find what you use"

find_package(hytech_np_proto_cpp CONFIG REQUIRED)
find_package(foxglove-schemas_proto_cpp CONFIG REQUIRED)

find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(protobuf REQUIRED)
find_package(GTest REQUIRED)

find_package(dbcppp CONFIG REQUIRED)
find_package(foxglove_websocket REQUIRED)

add_library(drivebrain_core SHARED drivebrain_core_base/src/JsonFileHandler.cpp)

target_include_directories(drivebrain_core PUBLIC 
    $<INSTALL_INTERFACE:drivebrain_core_base/include/public>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/drivebrain_core_base/include/public>    
)

# right now we dont want to expose the json file handler directly to all of the 
# components of drivebrain
target_include_directories(drivebrain_core PRIVATE 
    $<INSTALL_INTERFACE:drivebrain_core_base/include/private>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/drivebrain_core_base/include/private>    
)

target_link_libraries(drivebrain_core PUBLIC nlohmann_json::nlohmann_json Boost::boost Boost::system)


# impl libraries
add_library(foxglove_param_server SHARED drivebrain_core_impl/drivebrain_comms/foxglove_param_server/src/param_server.cpp)

target_include_directories(foxglove_param_server PUBLIC 
    $<INSTALL_INTERFACE:drivebrain_core_impl/drivebrain_comms/foxglove_param_server/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/drivebrain_core_impl/drivebrain_comms/foxglove_param_server/include>    
)

target_link_libraries(foxglove_param_server PUBLIC 
    drivebrain_core 
    foxglove-schemas_proto_cpp::foxglove-schemas_proto_cpp 
    foxglove_websocket::foxglove_websocket    
)

add_library(CAN_driver SHARED drivebrain_core_impl/drivebrain_comms/car_CAN_driver/src/CANComms.cpp)

target_include_directories(CAN_driver PUBLIC
    $<INSTALL_INTERFACE:drivebrain_core_impl/drivebrain_comms/car_CAN_driver/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/drivebrain_core_impl/drivebrain_comms/car_CAN_driver/include>    
)

target_link_libraries(CAN_driver PUBLIC 
    drivebrain_core
    hytech_np_proto_cpp::hytech_np_proto_cpp
    protobuf::libprotobuf
    dbcppp::dbcppp
)

add_executable(test_CAN_driver drivebrain_core_impl/drivebrain_comms/car_CAN_driver/test/test_can_driver.cpp)
target_link_libraries(test_CAN_driver PUBLIC
    CAN_driver
    GTest::gtest_main
)

add_executable(test_boost_asio_CAN drivebrain_app/example_boost_asio_can.cpp)

target_link_libraries(test_boost_asio_CAN PUBLIC
    Boost::boost
)


add_executable(test_param_server test/test_param_server.cpp)

target_link_libraries(test_param_server PUBLIC 
    foxglove-schemas_proto_cpp::foxglove-schemas_proto_cpp 
    foxglove_websocket::foxglove_websocket)

install(TARGETS test_param_server)