{ pkgs, stdenv, cmake, boost, pkg-config, cmake_macros, protobuf, drivebrain_core_msgs_proto_cpp, hytech_np_proto_cpp, nlohmann_json, db-core-src, spdlog, fmt }:
stdenv.mkDerivation {
    name = "db-core";
    src = db-core-src;
    nativeBuildInputs = [ cmake pkg-config cmake_macros ];
    propagatedBuildInputs = [  boost protobuf drivebrain_core_msgs_proto_cpp hytech_np_proto_cpp nlohmann_json cmake_macros spdlog fmt ];
}