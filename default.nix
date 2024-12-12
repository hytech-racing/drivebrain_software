{ pkgs, stdenv, cmake, boost, pkg-config, lz4 ,zstd, protobuf, nlohmann_json, foxglove-ws-protocol-cpp, cmake_macros, hytech_np_proto_cpp, dbcppp, gtest, drivebrain_core_msgs_proto_cpp, mcap, db_service_grpc_cpp, grpc, vn_lib, drivebrain_core, simulink_automation, spdlog, fmt, geographiclib, ... }:

stdenv.mkDerivation {
  name = "drivebrain_software";
  src = ./.;
  nativeBuildInputs = [ cmake pkg-config ];
  propagatedBuildInputs = [ protobuf lz4 zstd boost cmake_macros nlohmann_json foxglove-ws-protocol-cpp hytech_np_proto_cpp dbcppp gtest drivebrain_core_msgs_proto_cpp mcap db_service_grpc_cpp grpc vn_lib drivebrain_core simulink_automation spdlog fmt geographiclib ];
  dontStrip = true;
  cmakeFlags = [ "-DCMAKE_FIND_DEBUG_MODE=ON"];
}
