{
  description = "my packages. im tired of making new repos for nix packages and im too lazy to push em up to nixpkgs";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.05";

    flake-parts.url = "github:hercules-ci/flake-parts";
    flake-parts.inputs.nixpkgs-lib.follows = "nixpkgs";

    devshell.url = "github:numtide/devshell";
    devshell.inputs.nixpkgs.follows = "nixpkgs";

    nebs-packages.url = "github:RCMast3r/nebs_packages";
    nebs-packages.inputs.nixpkgs.follows = "nixpkgs";

    easy_cmake.url = "github:RCMast3r/easy_cmake/5be1d78ff8383590a3cf5ed3680554d8e619489e";
    easy_cmake.inputs.nixpkgs.follows = "nixpkgs";

    nix-proto.url = "github:notalltim/nix-proto";
    nix-proto.inputs.nixpkgs.follows = "nixpkgs";

    foxglove-schemas-src = {
      url = "github:foxglove/schemas";
      flake = false;
    };

    ht_can.url = "github:hytech-racing/ht_can";
    ht_can.inputs.nixpkgs.follows = "nixpkgs";

    data_acq.url = "github:hytech-racing/data_acq/feature/proto_gen_packaging_fix";
    data_acq.inputs.ht_can_pkg_flake.follows = "ht_can";
    data_acq.inputs.nix-proto.follows = "nix-proto";
    data_acq.inputs.nixpkgs.follows = "nixpkgs";
  };
  outputs = { self, nixpkgs, flake-parts, devshell, nebs-packages, easy_cmake, nix-proto, foxglove-schemas-src, data_acq, ... }@inputs:

    flake-parts.lib.mkFlake { inherit inputs; }
      {
        systems = [
          "x86_64-linux"
          "aarch64-linux"
        ];
        imports = [
          inputs.flake-parts.flakeModules.easyOverlay
          inputs.devshell.flakeModule
        ];
        perSystem = { config, pkgs, system, ... }:
          let
            drivebrain_software = pkgs.callPackage ./default.nix { };
            nix-proto-foxglove-overlays = nix-proto.generateOverlays' {
              foxglove-schemas = nix-proto.mkProtoDerivation {
                name = "foxglove-schemas";
                version = "1.0.1";
                src = nix-proto.lib.srcFromNamespace {
                  root = "${foxglove-schemas-src}/schemas/proto";
                  namespace = "foxglove";
                };
              };
              drivebrain_core_msgs = nix-proto.mkProtoDerivation {
                name = "drivebrain_core_msgs";
                version = "0.0.1";
                src = nix-proto.lib.srcFromNamespace {
                  root = ./proto;
                  namespace = "drivebrain_core_msgs/v1";
                };
                # protoDeps = [base_api]; TODO add in the generated protos
              };
            };
          in
          {
            _module.args.pkgs = import inputs.nixpkgs {
              inherit system;
              overlays = [
                nebs-packages.overlays.default
                easy_cmake.overlays.default
              ] ++ data_acq.overlays.x86_64-linux ++ (nix-proto.lib.overlayToList nix-proto-foxglove-overlays);
              config = { };
            };
            packages.default = drivebrain_software;
            overlayAttrs = {
              inherit (config.packages) drivebrain_software;
            };

            devshells.default = {
              env = [ ];
              commands = [
                {
                  name = "b";
                  command = "cd $PRJ_ROOT && rm -rf build && mkdir build && cd build && cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && make -j && cd $PRJ_ROOT";
                }
                {
                  name = "bd";
                  command = "cd $PRJ_ROOT && rm -rf build && mkdir build && cd build && cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug && make -j && cd $PRJ_ROOT";
                }
              ];
              packages = [
                pkgs.openssl
                pkgs.abseil-cpp # idk wtf is going on but for some reason this is not propagating from protobuf, maybe its my nix version ? (2.20)
                pkgs.gcc # this has to be specified because otherwise it uses the system's compiler and that can screw with things :eyes:
                pkgs.glibc
                pkgs.nlohmann_json
                pkgs.boost
                pkgs.protobuf
              ];
              packagesFrom = [ drivebrain_software pkgs.foxglove-ws-protocol-cpp ];
            };

            legacyPackages =
              import nixpkgs {
                inherit system;
                overlays = [ 
                  nebs-packages.overlays.default 
                  easy_cmake.overlays.default 
                  (final: _: { drivebrain_software = final.callPackage ./default.nix { }; })
                ]
                ++ data_acq.overlays.x86_64-linux
                ++ (nix-proto.lib.overlayToList nix-proto-foxglove-overlays);
              };

          };
      };

}
