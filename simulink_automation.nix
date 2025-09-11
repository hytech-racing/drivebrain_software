{ pkgs, stdenv, cmake, drivebrain_core, db-simulink-gen-src, simulink_automation_msgs_proto_cpp }:

stdenv.mkDerivation {
    name = "matlab-math";
    src = "${db-simulink-gen-src}/matlab_math.tar.gz";
    version = "1.0.0";
    nativeBuildInputs = [ cmake ];
    propagatedBuildInputs = [ drivebrain_core simulink_automation_msgs_proto_cpp];
    # cmakeFlags = [ "-DCMAKE_FIND_DEBUG_MODE=ON" ];
}