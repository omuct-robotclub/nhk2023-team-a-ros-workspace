{
  inputs = {
    ros2nix.url = "github:pylgos/ros2nix";
    nixpkgs.follows = "ros2nix/nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, flake-utils, nixpkgs, ros2nix}: 
    flake-utils.lib.eachDefaultSystem (system:
      let
        # pkgs = nixpkgs.legacyPackages.${system};
        rosPkgs = ros2nix.legacyPackages.${system}.humble;
      in rec {
        packages.default = rosPkgs.buildRosPackage {
          pname = "robot_interface";
          version = "0.1.0";

          src = ./.;

          buildToolDepend = [
            "ament_cmake"
            "rosidl_default_generators"
          ];

          buildDepend = [
            "builtin_interfaces"
          ];

          execDepend = [
            "rosidl_default_runtime"
            "builtin_interfaces"
          ];
        };
      }
    );
}