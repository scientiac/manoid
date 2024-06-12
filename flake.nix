{
  description = "Yantra Swarmanoid Environment";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
  };

  outputs = {
    nixpkgs,
    ...
  } : let
    system = "x86_64-linux";

    pkgs = nixpkgs.legacyPackages.${system};

    python = let
      packageOverrides = self: super: {
        opencv4 = super.opencv4.override {
          enableGtk2 = true;
          gtk2 = pkgs.gnome2.gtk;
        };
      };
    in
      pkgs.python311.override {
        inherit packageOverrides;
        self = python;
      };

    pythonEnv = python.withPackages (ps:
      with ps; [
        opencv4
        numpy
        paho-mqtt
	flask
        # LSP
        python-lsp-server
        black
      ]);

    dependencies = with pkgs; [
      # Micropython Dependencies
      screen
      mosquitto
    ];

    shellHook = ''
            screen -S mqtt-session -dm mosquitto -c ./etc/mosquitto.conf
            echo "Type 'screen -r mqtt-session' to get mqtt logs!"
    '';

  in {
    devShells.${system}.default = pkgs.mkShell {
      buildInputs = [pythonEnv] ++ dependencies;
      shellHook = shellHook;
    };
  };
}
