{ pkgs, stdenv, cmake, drivebrain_core, simulink-automation-src }:

stdenv.mkDerivation {
    name = "Simulink-automation";
    src = simulink-automation-src;
    version = "1.0.0";
    nativeBuildInputs = [ cmake ];
    propagatedBuildInputs = [ cmake drivebrain_core ];
    # cmakeFlags = [ "-DCMAKE_FIND_DEBUG_MODE=ON" ];
}