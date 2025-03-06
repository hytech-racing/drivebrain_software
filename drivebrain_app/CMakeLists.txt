cmake_minimum_required(VERSION 3.24)
project(drivebrain_app LANGUAGES CXX)

# Find required packages
find_package(drivebrain_core REQUIRED)
find_package(hytech_np_proto_cpp CONFIG REQUIRED)
find_package(drivebrain_core_msgs_proto_cpp CONFIG REQUIRED)
find_package(cmake_macros REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED COMPONENTS system program_options)

# Add executable
add_executable(drivebrain_app
    src/DriveBrainApp.cpp
    src/main.cpp
    src/debug_main.cpp
)

# Set include directories
target_include_directories(drivebrain_app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${drivebrain_core_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(drivebrain_app PRIVATE
    drivebrain_core::drivebrain_core
    hytech_np_proto_cpp::hytech_np_proto_cpp
    drivebrain_core_msgs_proto_cpp::drivebrain_core_msgs_proto_cpp
    spdlog::spdlog
    fmt::fmt
    Boost::boost
    Boost::system
    Boost::program_options
) 