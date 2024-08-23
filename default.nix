{ pkgs, stdenv, cmake, boost, pkg-config, protobuf, nlohmann_json, foxglove-ws-protocol-cpp, easy_cmake, foxglove-schemas_proto_cpp, drivebrain_core_msgs_proto_cpp, hytech_np_proto_cpp, dbcppp, gtest, ... }:

stdenv.mkDerivation {
  name = "drivebrain_software";
  src = ./.;
  nativeBuildInputs = [ cmake ];
  propagatedBuildInputs = [ protobuf boost easy_cmake nlohmann_json foxglove-ws-protocol-cpp foxglove-schemas_proto_cpp drivebrain_core_msgs_proto_cpp hytech_np_proto_cpp dbcppp gtest ];
}
