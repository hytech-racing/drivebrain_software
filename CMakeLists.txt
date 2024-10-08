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
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/version_header/versions.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/version_header/versions.h"
    @ONLY
)

### this one is generated from nix-proto but using foxglove's 
# find_package(foxglove-schemas_proto_cpp CONFIG REQUIRED) 

find_package(libvncxx REQUIRED)

### off-the-shelf libraries
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS system program_options)
find_package(protobuf REQUIRED)
find_package(GTest REQUIRED)
find_package(mcap REQUIRED)

# custom packaged libraries, code didnt require changes just the packaging
find_package(dbcppp CONFIG REQUIRED)
find_package(foxglove_websocket REQUIRED)
find_package(cmake_macros REQUIRED)

find_package(gRPC CONFIG REQUIRED)
find_package(db_service_grpc_cpp REQUIRED)
find_package(matlab_math REQUIRED)

include(create_package)

#####################
# created libraries #
#####################

# drivebrain core library
add_library(drivebrain_core SHARED 
    drivebrain_core_base/src/JsonFileHandler.cpp 
    drivebrain_core_base/src/Configurable.cpp 
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


add_library(drivebrain_common_utils INTERFACE)

target_include_directories(drivebrain_common_utils INTERFACE
    $<INSTALL_INTERFACE:drivebrain_core_impl/drivebrain_common_utils/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/drivebrain_core_impl/drivebrain_common_utils/include>
)

target_link_libraries(drivebrain_common_utils INTERFACE
    protobuf::libprotobuf
)

make_cmake_package(drivebrain_common_utils drivebrain)

# CAN driver for parsing and encoding CAN packets and interacting with a socketCAN interface
add_library(drivebrain_comms SHARED
    drivebrain_core_impl/drivebrain_comms/src/foxglove_server.cpp
    drivebrain_core_impl/drivebrain_comms/src/CANComms.cpp
    drivebrain_core_impl/drivebrain_comms/src/VNComms.cpp
    drivebrain_core_impl/drivebrain_comms/src/MCUETHComms.cpp
    drivebrain_core_impl/drivebrain_comms/src/DBServiceImpl.cpp
)

target_compile_features(drivebrain_comms PUBLIC cxx_std_17)


target_include_directories(drivebrain_comms PUBLIC
    $<INSTALL_INTERFACE:drivebrain_core_impl/drivebrain_comms/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/drivebrain_core_impl/drivebrain_comms/include>    
)

target_include_directories(drivebrain_comms PUBLIC
    $<INSTALL_INTERFACE:version_header>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/version_header>    
)

target_link_libraries(drivebrain_comms PUBLIC 
    drivebrain_core
    drivebrain_common_utils
    gRPC::grpc++_reflection
    foxglove_websocket::foxglove_websocket
    protobuf::libprotobuf
    dbcppp::dbcppp
    hytech_np_proto_cpp::hytech_np_proto_cpp
    drivebrain_core_msgs_proto_cpp::drivebrain_core_msgs_proto_cpp
    db_service_grpc_cpp::db_service_grpc_cpp
    libvncxx::libvncxx
    drivebrain_estimation

)

make_cmake_package(drivebrain_comms drivebrain)


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

add_library(drivebrain_mcap_logger SHARED drivebrain_core_impl/drivebrain_mcap_logger/src/MCAPProtobufLogger.cpp)
target_include_directories(drivebrain_mcap_logger PUBLIC
    $<INSTALL_INTERFACE:drivebrain_core_impl/drivebrain_mcap_logger/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/drivebrain_core_impl/drivebrain_mcap_logger/include>    
)

target_link_libraries(drivebrain_mcap_logger PUBLIC
    drivebrain_core 
    drivebrain_common_utils
    protobuf::libprotobuf
    mcap::mcap
)

target_link_libraries(drivebrain_mcap_logger INTERFACE
    mcap::mcap
)
target_include_directories(drivebrain_mcap_logger PUBLIC
    $<INSTALL_INTERFACE:version_header>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/version_header>    
)
make_cmake_package(drivebrain_mcap_logger drivebrain)

add_library(drivebrain_estimation SHARED 
    drivebrain_core_impl/drivebrain_estimation/src/MatlabMath.cpp
    drivebrain_core_impl/drivebrain_estimation/src/StateEstimator.cpp
)

target_include_directories(drivebrain_estimation PUBLIC
    $<INSTALL_INTERFACE:drivebrain_core_impl/drivebrain_estimation/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/drivebrain_core_impl/drivebrain_estimation/include>    
)

target_link_libraries(drivebrain_estimation PUBLIC
    drivebrain_core 
    drivebrain_common_utils
    matlab_math::matlab_math_lib
)

make_cmake_package(drivebrain_estimation drivebrain)

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

add_executable(test_db_grpc test/test_db_interace_grpc_service.cpp)

target_link_libraries(test_db_grpc PUBLIC
    db_service_grpc_cpp::db_service_grpc_cpp
    gRPC::grpc++_reflection
)


add_executable(test_mcap test/test_mcap/test_mcap_logging.cpp test/test_mcap/BuildFileDescriptorSet.cpp)

target_link_libraries(test_mcap PUBLIC
    drivebrain_core_msgs_proto_cpp::drivebrain_core_msgs_proto_cpp
    protobuf::libprotobuf
    mcap::mcap
)

set(CMAKE_CXX_STANDARD 17) 
add_executable(test_vn test/test_vn.cpp)
target_link_libraries(test_vn PUBLIC
    Boost::boost
    Boost::system
    libvncxx::libvncxx
)

target_compile_features(test_vn PUBLIC cxx_std_11)

###                   ###
### system executable ###
###                   ###

add_executable(alpha_build drivebrain_app/main.cpp)

target_link_libraries(alpha_build PUBLIC
    drivebrain_core
    drivebrain_control
    drivebrain_comms
    drivebrain_mcap_logger
)

include(GNUInstallDirs)
install(TARGETS 
        alpha_build
    RUNTIME 
        DESTINATION ${CMAKE_INSTALL_BINDIR})