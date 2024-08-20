{ pkgs, stdenv, cmake, boost, pkg-config,protobuf, foxglove-ws-protocol-cpp, easy_cmake, foxglove-schemas_proto_cpp, drivebrain_core_msgs_proto_cpp, ...}:

stdenv.mkDerivation {
  name = "drivebrain_software";
  src = ./.;
  nativeBuildInputs = [ cmake ];
  propagatedBuildInputs = [ protobuf boost easy_cmake foxglove-ws-protocol-cpp foxglove-schemas_proto_cpp drivebrain_core_msgs_proto_cpp ];
}