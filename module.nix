{ config, lib, pkgs, ... }: 
let
  jsonContent = builtins.readFile ./config/drivebrain_config.json;
in
{
  options.drivebrain-service.enable = lib.mkOption {
    type = lib.types.bool;
    default = false;
    description = "Enable or disable the drivebrain service";
  };

  config = lib.mkIf config.drivebrain-service.enable {

    systemd.services.drivebrain-service = {
      description = "Drivebrain Service";
      wantedBy = [ "multi-user.target" ];
      after = [ "network.target" ];

      serviceConfig = {
        After = [ "network.target" ];
        ExecStart = "${pkgs.drivebrain_software}/bin/alpha_build /home/nixos/config/drivebrain_config.json ${pkgs.ht_can_pkg}/hytech.dbc";
        ExecStop = "/bin/kill -9 $MAINPID";
        Restart = "on-failure";
      };
    };

    # Create /home/nixos/config directory if it doesn't exist
    system.activationScripts.createConfigDir = pkgs.lib.mkForce ''
      mkdir -p /home/nixos/config
      chown nixos:users /home/nixos/config
    '';

    # Write JSON content to /home/nixos/config/drivebrain_config.json
    system.activationScripts.writeConfigFile = pkgs.lib.mkForce ''
      echo ''${jsonContent} > /home/nixos/config/drivebrain_config.json
      chown nixos:users /home/nixos/config/drivebrain_config.json
    '';
    
  };
}