cmake_minimum_required(VERSION 3.24)
project(drivebrain_software LANGUAGES CXX C)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INCLUDE_HEADERS_IN_COMPILE_COMMANDS ON)

### just like "include what you use" and do "find what you use"

## hytech's upstream libraries

### generated via nix-proto from hytech's generated CAN protobufs
find_package(hytech_np_proto_cpp CONFIG REQUIRED)
find_package(drivebrain_core_msgs_proto_cpp CONFIG REQUIRED)
### this one is generated from nix-proto but using foxglove's 
# find_package(foxglove-schemas_proto_cpp CONFIG REQUIRED) 

### off-the-shelf libraries
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(protobuf REQUIRED)
find_package(GTest REQUIRED)
find_package(mcap REQUIRED)

# custom packaged libraries, code didnt require changes just the packaging
find_package(dbcppp CONFIG REQUIRED)
find_package(foxglove_websocket REQUIRED)
find_package(cmake_macros REQUIRED)
include(create_package)

#####################
# created libraries #
#####################

# drivebrain core library
add_library(drivebrain_core SHARED 
    drivebrain_core_base/src/JsonFileHandler.cpp 
    drivebrain_core_base/src/Configurable.cpp 
    drivebrain_core_base/src/StateEstimator.cpp
    drivebrain_core_base/src/Logger.cpp)

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

target_link_libraries(drivebrain_core PUBLIC 
    nlohmann_json::nlohmann_json 
    Boost::boost 
    Boost::system 
    drivebrain_core_msgs_proto_cpp::drivebrain_core_msgs_proto_cpp
    hytech_np_proto_cpp::hytech_np_proto_cpp
)

make_cmake_package(drivebrain_core drivebrain)

# foxglove parameter server interface
add_library(foxglove_server SHARED drivebrain_core_impl/drivebrain_comms/foxglove_server/src/foxglove_server.cpp)

target_compile_features(foxglove_server PUBLIC cxx_std_17)

target_include_directories(foxglove_server PUBLIC 
    $<INSTALL_INTERFACE:drivebrain_core_impl/drivebrain_comms/foxglove_server/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/drivebrain_core_impl/drivebrain_comms/foxglove_server/include>    
)

target_link_libraries(foxglove_server PUBLIC 
    drivebrain_core
    protobuf::libprotobuf
    drivebrain_core_msgs_proto_cpp::drivebrain_core_msgs_proto_cpp
    hytech_np_proto_cpp::hytech_np_proto_cpp
    foxglove_websocket::foxglove_websocket    
)
target_compile_features(foxglove_server PUBLIC cxx_std_17) 
make_cmake_package(foxglove_server drivebrain)

# CAN driver for parsing and encoding CAN packets and interacting with a socketCAN interface
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
make_cmake_package(CAN_driver drivebrain)

add_library(ETH_driver SHARED drivebrain_core_impl/drivebrain_comms/car_ETH_driver/src/MCUETHComms.cpp)
target_include_directories(ETH_driver PUBLIC
    $<INSTALL_INTERFACE:drivebrain_core_impl/drivebrain_comms/car_ETH_driver/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/drivebrain_core_impl/drivebrain_comms/car_ETH_driver/include>    
)

target_link_libraries(ETH_driver PUBLIC 
    drivebrain_core
    hytech_np_proto_cpp::hytech_np_proto_cpp
    drivebrain_core_msgs_proto_cpp::drivebrain_core_msgs_proto_cpp
    protobuf::libprotobuf
)
make_cmake_package(ETH_driver drivebrain)

add_library(drivebrain_control SHARED drivebrain_core_impl/drivebrain_control/src/SimpleController.cpp)
# drivebrain_core_impl/drivebrain_control/include/SimpleController.hpp
target_include_directories(drivebrain_control PUBLIC
    $<INSTALL_INTERFACE:drivebrain_core_impl/drivebrain_control/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/drivebrain_core_impl/drivebrain_control/include>    
)

target_link_libraries(drivebrain_control PUBLIC 
    drivebrain_core
    hytech_np_proto_cpp::hytech_np_proto_cpp
    protobuf::libprotobuf
)
make_cmake_package(drivebrain_control drivebrain)

###############
# executables #
###############

###                     ###
### testing executables ###
###                     ###

# add_executable(test_CAN_driver drivebrain_core_impl/drivebrain_comms/car_CAN_driver/test/test_can_driver.cpp)
# target_link_libraries(test_CAN_driver PUBLIC
#     CAN_driver
#     GTest::gtest_main
# )

# add_executable(test_boost_asio_CAN drivebrain_app/example_boost_asio_can.cpp)

# target_link_libraries(test_boost_asio_CAN PUBLIC
#     Boost::boost
# )

# add_executable(test_foxglove_server test/test_foxglove_server.cpp)

# target_link_libraries(test_foxglove_server PUBLIC 
#     foxglove-schemas_proto_cpp::foxglove-schemas_proto_cpp 
#     foxglove_websocket::foxglove_websocket)

add_executable(mcu_standin test/test_mcu.cpp)

target_link_libraries(mcu_standin PUBLIC
    drivebrain_core_msgs_proto_cpp::drivebrain_core_msgs_proto_cpp
    protobuf::libprotobuf
)

add_executable(mcu_test test/test_mcu_send.cpp)


target_link_libraries(mcu_test PUBLIC
    drivebrain_core_msgs_proto_cpp::drivebrain_core_msgs_proto_cpp
    protobuf::libprotobuf
)

add_executable(test_mcap test/test_mcap/test_mcap_logging.cpp test/test_mcap/BuildFileDescriptorSet.cpp)
target_link_libraries(test_mcap PUBLIC
    drivebrain_core_msgs_proto_cpp::drivebrain_core_msgs_proto_cpp
    protobuf::libprotobuf
    mcap::mcap
)

add_executable(test_msg_logger test/test_msglogger.cpp)

target_link_libraries(test_msg_logger PUBLIC
    drivebrain_core
)




###                   ###
### system executable ###
###                   ###

add_executable(alpha_build drivebrain_app/main.cpp)

target_link_libraries(alpha_build PUBLIC
    drivebrain_core
    drivebrain_control
    CAN_driver
    foxglove_server
    ETH_driver
)
include(GNUInstallDirs)
install(TARGETS 
        # test_foxglove_server
        alpha_build
    RUNTIME 
        DESTINATION ${CMAKE_INSTALL_BINDIR})