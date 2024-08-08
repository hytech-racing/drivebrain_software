{ pkgs, stdenv, cmake, foxglove-ws-protocol-cpp, easy_cmake, protobuf, foxglove-schemas_proto_cpp, ...}:

stdenv.mkDerivation {
  name = "drivebrain_software";
  src = ./.;
  nativeBuildInputs = [ cmake ];
  buildInputs = [ easy_cmake foxglove-ws-protocol-cpp protobuf foxglove-schemas_proto_cpp ];
}