{
  inputs = {
    ros2nix.url = "github:pylgos/ros2nix";
    nixpkgs.follows = "ros2nix/nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
    colcon-ros-nimble.url = "github:pylgos/colcon-ros-nimble";
    colcon-ros-nimble.inputs.ros2nix.follows = "ros2nix";
    colcon-ros-nimble.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = { self, flake-utils, nixpkgs, ros2nix, colcon-ros-nimble }: 
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        rosPkgs = ros2nix.legacyPackages.${system}.humble;
        py = pkgs.python310Packages;
        colconRosNimble = colcon-ros-nimble.packages.${system}.default;
      in rec {
        devShells.default = rosPkgs.mkRosWorkspace {
          pkgs = [
            colconRosNimble
            rosPkgs.systemPackages.python3-colcon-common-extensions
            rosPkgs.ament_cmake
            rosPkgs.rosidl_default_generators
            rosPkgs.ament_lint_common
            rosPkgs.ros2cli
            rosPkgs.ros2run
            rosPkgs.ros2topic
            rosPkgs.ros2service
            rosPkgs.ros2param
            rosPkgs.rcl
            rosPkgs.geometry_msgs
            rosPkgs.diagnostic_msgs
            rosPkgs.builtin_interfaces
            rosPkgs.std_srvs
            rosPkgs.sensor_msgs
            rosPkgs.rcl_interfaces
            rosPkgs.teleop_twist_keyboard
            rosPkgs.desktop
            pkgs.gdb
          ];

          buildInputs = [
            
          ];
        };
      }
    );
}