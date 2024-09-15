{ pkgs, stdenv, cmake, boost, pkg-config, protobuf, nlohmann_json, foxglove-ws-protocol-cpp, cmake_macros, hytech_np_proto_cpp, dbcppp, gtest, drivebrain_core_msgs_proto_cpp, ... }:

stdenv.mkDerivation {
  name = "drivebrain_software";
  src = ./.;
  nativeBuildInputs = [ cmake ];
  propagatedBuildInputs = [ protobuf boost cmake_macros nlohmann_json foxglove-ws-protocol-cpp hytech_np_proto_cpp dbcppp gtest drivebrain_core_msgs_proto_cpp];
}
