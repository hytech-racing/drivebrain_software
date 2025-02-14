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
        # ref = "2024-11-08T01_13_42";
        ref = "2025-02-10T09_42_47";
        flake = false;
      };

    foxglove-schemas-src = {
      url = "github:foxglove/schemas";
      flake = false;
    };

    ht_can.url = "github:hytech-racing/ht_can/140";
    ht_can.inputs.nixpkgs.follows = "nixpkgs";
    ht_can.inputs.nix-proto.follows = "nix-proto";

    vn_driver_lib.url = "github:RCMast3r/vn_driver_lib/fix/boost-compatible";

    db-core-src = {
      url = "github:hytech-racing/drivebrain_core";
      flake = false;
    };

    simulink-automation-src = {
      url = "https://github.com/hytech-racing/simulink_automation/releases/download/CodeGen_2024.11.13_05-40/matlab_math.tar.gz";
      flake = false;
    };

    nanopb-proto-api = {
      url = "github:nanopb/nanopb";
      flake = false;
    };
  };
  outputs = { self, nixpkgs, flake-parts, nebs-packages, easy_cmake, nix-proto, foxglove-schemas-src, ht_can, HT_proto, vn_driver_lib, simulink-automation-src, db-core-src, nanopb-proto-api, ... }@inputs:
    let
      nanopb-api = nix-proto.mkProtoDerivation {
        name = "nanopb-api";
        version = "0.0.0";
        # need to remove the makefile from the proto boi because nix will attempt to build that shit
        src = builtins.filterSource (path: _: baseNameOf path != "Makefile") "${nanopb-proto-api}/generator/proto";
      };

      drivebrain_core_msgs = { nanopb-api }: nix-proto.mkProtoDerivation {
        name = "drivebrain_core_msgs";
        version = HT_proto.rev;
        src = "${HT_proto}/proto";
        protoDeps = [ nanopb-api ];
      };

      nix-proto-foxglove-overlays = nix-proto.generateOverlays' {

        inherit nanopb-api;
        inherit drivebrain_core_msgs;

        foxglove-schemas = nix-proto.mkProtoDerivation {
          name = "foxglove-schemas";
          version = "1.0.1";
          src = nix-proto.lib.srcFromNamespace {
            root = "${foxglove-schemas-src}/schemas/proto";
            namespace = "foxglove";
          };
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

      db_core_overlay = final: prev: {
        drivebrain_core = final.callPackage ./db-core.nix { inherit db-core-src; };
      };

      simulink_automation_overlay = final: prev: {
        simulink_automation = final.callPackage ./simulink_automation.nix { inherit simulink-automation-src; };
      };

      my_overlays = [
        (final: prev: {
          drivebrain_software = final.callPackage ./default.nix { };
        })
        (self: super: {
          python311 = super.python311.override {
            packageOverrides = pyself: pysuper: {
              crccheck = pysuper.crccheck.overrideAttrs (oldAttrs: {
                meta.platforms = nixpkgs.lib.platforms.all;
              });
              cantools = pysuper.cantools.overrideAttrs (oldAttrs: {
                doCheck = false;
                disabledTests = [ "test_plot_style test_plot_tz" ];
              });
            };
          };
        }
        )
        simulink_automation_overlay
        db_core_overlay
      ] ++ (nix-proto.lib.overlayToList nix-proto-foxglove-overlays);

    in
    flake-parts.lib.mkFlake { inherit inputs; }

      {
        systems = [
          "x86_64-linux"
          "aarch64-linux"
          "aarch64-darwin"
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
                ht_can.overlays.default
                self.overlays.default
              ] ++ (nix-proto.lib.overlayToList nix-proto-foxglove-overlays);
              config = { };
            };
            packages.default = pkgs.drivebrain_software;
            packages.drivebrain_software = pkgs.drivebrain_software;
            packages.drivebrain_core = pkgs.drivebrain_core;
            packages.simulink_automation = pkgs.simulink_automation;

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

            devShells.sim_automath = pkgs.mkShell rec {
              name = "nix-devshell";
              shellHook =
                let icon = "f121";
                in ''
                  
                  export PS1="$(echo -e '\u${icon}') {\[$(tput sgr0)\]\[\033[38;5;228m\]\w\[$(tput sgr0)\]\[\033[38;5;15m\]} (${name}) \\$ \[$(tput sgr0)\]"
                  
                '';
              inputsFrom = [
                pkgs.simulink_automation
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
                  ht_can.overlays.default
                  self.overlays.default
                ] ++ (nix-proto.lib.overlayToList nix-proto-foxglove-overlays);
              };
          };
      };

}
