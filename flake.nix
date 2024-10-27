{
  description = "drivebrain flake";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.05";

    flake-parts.url = "github:hercules-ci/flake-parts";
    flake-parts.inputs.nixpkgs-lib.follows = "nixpkgs";

    nebs-packages.url = "github:RCMast3r/nebs_packages";
    nebs-packages.inputs.nixpkgs.follows = "nixpkgs";

    easy_cmake.url = "github:RCMast3r/easy_cmake/5be1d78ff8383590a3cf5ed3680554d8e619489e";
    easy_cmake.inputs.nixpkgs.follows = "nixpkgs";

    nix-proto.url = "github:notalltim/nix-proto";
    nix-proto.inputs.nixpkgs.follows = "nixpkgs";

    HT_proto =
      {
        type = "github";
        owner = "hytech-racing";
        repo = "HT_proto";
        ref = "2024-10-15T07_06_59";
        flake = false;
      };

    foxglove-schemas-src = {
      url = "github:foxglove/schemas";
      flake = false;
    };

    ht_can.url = "github:hytech-racing/ht_can/132";
    ht_can.inputs.nixpkgs.follows = "nixpkgs";

    data_acq.url = "github:hytech-racing/data_acq/feature/proto_gen_packaging_fix";
    data_acq.inputs.ht_can_pkg_flake.follows = "ht_can";
    data_acq.inputs.nix-proto.follows = "nix-proto";
    data_acq.inputs.nixpkgs.follows = "nixpkgs";

    vn_driver_lib.url = "github:RCMast3r/vn_driver_lib/fix/boost-compatible";

    matlab_math = {
        url = "https://github.com/hytech-racing/simulink_automation/releases/download/CodeGen_2024.10.27_05-27/matlab_math.tar.gz";
        flake = false;
    };

  };
  outputs = { self, nixpkgs, flake-parts, nebs-packages, easy_cmake, nix-proto, foxglove-schemas-src, data_acq, HT_proto, vn_driver_lib, matlab_math, ... }@inputs:
    let

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
          version = HT_proto.rev;
          src = "${HT_proto}/proto";
        };
        db_service = nix-proto.mkProtoDerivation
          {
            name = "db_service";
            version = "0.0.1";
            src = nix-proto.lib.srcFromNamespace {
              root = ./proto;
              namespace = "db_service";
            };
          };
      };
      # Create matlab math deriv and overlay
      matlab_math_derivation = { stdenv, cmake  }: stdenv.mkDerivation {
        name = "matlab-models-deriv";
        src = matlab_math;
        nativeBuildInputs = [ cmake ];
      };

      matlab_math_overlay = final: prev: {
        matlab_math = final.callPackage matlab_math_derivation { };
      };

      my_overlays = [
        (final: prev: {
          drivebrain_software = final.callPackage ./default.nix { };
        })
        matlab_math_overlay
      ] ++ (nix-proto.lib.overlayToList nix-proto-foxglove-overlays);

    in
    flake-parts.lib.mkFlake { inherit inputs; }

      {
        systems = [
          "x86_64-linux"
          "aarch64-linux"
        ];
        imports = [
          # inputs.flake-parts.flakeModules.easyOverlay
        ];


        flake.overlays = {
          default = nixpkgs.lib.composeManyExtensions my_overlays;
        };

        perSystem = { config, pkgs, system, ... }:

          {
            _module.args.pkgs = import inputs.nixpkgs {
              inherit system;
              overlays = [
                vn_driver_lib.overlays.default
                nebs-packages.overlays.default
                easy_cmake.overlays.default
                self.overlays.default
              ] ++ data_acq.overlays.x86_64-linux ++ (nix-proto.lib.overlayToList nix-proto-foxglove-overlays);
              config = { };
            };
            packages.default = pkgs.drivebrain_software;
            packages.drivebrain_software = pkgs.drivebrain_software;

            devShells.default = pkgs.mkShell rec {
              name = "nix-devshell";
              shellHook =
                let icon = "f121";
                in ''
                  dbc_path=${pkgs.ht_can_pkg}
                  export DBC_PATH=$dbc_path
                  export PS1="$(echo -e '\u${icon}') {\[$(tput sgr0)\]\[\033[38;5;228m\]\w\[$(tput sgr0)\]\[\033[38;5;15m\]} (${name}) \\$ \[$(tput sgr0)\]"
                  alias build="rm -rf build && mkdir build && cd build && cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && make -j && cd .."
                  alias br="cd build && cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && make -j && cd .."
                  alias run="./build/alpha_build config/drivebrain_config.json $DBC_PATH/hytech.dbc"
                '';
              nativeBuildInputs = [ pkgs.drivebrain_core_msgs_proto_cpp ];
              packages = [ pkgs.mcap-cli ];
              inputsFrom = [
                pkgs.drivebrain_software
              ];
            };

            devShells.tests = pkgs.mkShell rec {
              name = "test-devshell";
        
              shellHook =
                let icon = "f121";
                in ''
                  dbc_path=${pkgs.ht_can_pkg}
                  export DBC_PATH=$dbc_path
                  export PS1="$(echo -e '\u${icon}') {\[$(tput sgr0)\]\[\033[38;5;228m\]\w\[$(tput sgr0)\]\[\033[38;5;15m\]} (${name}) \\$ \[$(tput sgr0)\]"
                  alias run="./build/alpha_build config/drivebrain_config.json $DBC_PATH/hytech.dbc"
                '';

              nativeBuiltInputs = [ pkgs.drivebrain_core_msgs_proto_cpp ];

              inputsFrom = [
                pkgs.drivebrain_software
              ];
            
            };

            legacyPackages =
              import nixpkgs {
                inherit system;
                overlays = [
                  vn_driver_lib.overlays.default
                  nebs-packages.overlays.default
                  easy_cmake.overlays.default
                  self.overlays.default
                ] ++ data_acq.overlays.x86_64-linux ++ (nix-proto.lib.overlayToList nix-proto-foxglove-overlays);
              };
          };
      };

}
