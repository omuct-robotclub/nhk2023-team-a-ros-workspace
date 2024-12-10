{
  inputs = {
    ros2nix.url = "github:pylgos/ros2nix";
    nixpkgs.follows = "ros2nix/nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
    colcon-ros-nimble.url = "github:pylgos/colcon-ros-nimble";
    colcon-ros-nimble.inputs.ros2nix.follows = "ros2nix";
    colcon-ros-nimble.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs =
    {
      self,
      flake-utils,
      nixpkgs,
      ros2nix,
      colcon-ros-nimble,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        nim-overlay = final: prev: {
          nim = final.callPackage ./nim { inherit nixpkgs; };
        };
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nim-overlay ];
        };
        rosPkgs = ros2nix.legacyPackages.${system}.humble;
        py = pkgs.python310Packages;
        colconRosNimble = colcon-ros-nimble.packages.${system}.default;
        cligen = pkgs.fetchFromGitHub {
          owner = "c-blake";
          repo = "cligen";
          rev = "1.7.8";
          hash = "sha256-iImmHie0hMpKGdA8uRMLOXbbUlID68w/8ABhi4OyqKU=";
        };
        nimble-ament-build = pkgs.stdenv.mkDerivation {
          pname = "nimble-ament-build";
          version = "0.1.0";
          src = pkgs.fetchFromGitHub {
            owner = "Pylgos";
            repo = "nimble_ament_build";
            rev = "d05319487b234d87a071b4c7d6af7fe9f59291af";
            hash = "sha256-Nvx0Imbv0duApAMZ9WuDQcjkBNShP+pu3d29id0LNGo=";
          };
          nativeBuildInputs = [ pkgs.nim ];
          buildPhase = ''
            mkdir -p $out/bin
            nim c -d:release -o:$out/bin/nimble-ament-build -p:${cligen} --nimcache:nimcache src/nimble_ament_build.nim
          '';
          installPhase = "true";
        };
        nimble-unwrapped = pkgs.stdenv.mkDerivation rec {
          pname = "nimble-unwrapped";
          version = "0.13.1";
          strictDeps = true;

          src = pkgs.fetchFromGitHub {
            owner = "nim-lang";
            repo = "nimble";
            rev = "v${version}";
            sha256 = "1idb4r0kjbqv16r6bgmxlr13w2vgq5332hmnc8pjbxiyfwm075x8";
          };

          depsBuildBuild = [ pkgs.nim ];
          buildInputs = [ pkgs.openssl ];

          nimFlags = [
            "-d:release"
          ];

          buildPhase = ''
            runHook preBuild
            HOME=$NIX_BUILD_TOP nim c $nimFlags src/nimble
            runHook postBuild
          '';

          installPhase = ''
            runHook preBuild
            install -Dt $out/bin src/nimble
            runHook postBuild
          '';

          meta = with pkgs.lib; {
            description = "Package manager for the Nim programming language";
            homepage = "https://github.com/nim-lang/nimble";
            license = licenses.bsd3;
            maintainers = with maintainers; [ ehmry ];
            mainProgram = "nimble";
          };
        };
      in
      rec {
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
            rosPkgs.robot_localization
            rosPkgs.desktop
            rosPkgs.rosbridge_server
            rosPkgs.foxglove_bridge
            rosPkgs.pcl_ros
            rosPkgs.tinyxml2_vendor
            rosPkgs.xacro
            rosPkgs.nav2_map_server
            rosPkgs.nav2_costmap_2d
            rosPkgs.nav2_lifecycle_manager
            rosPkgs.backward_ros
            rosPkgs.urg_node
            rosPkgs.plotjuggler_ros
            rosPkgs.plotjuggler
            rosPkgs.navigation2
            rosPkgs.tf_transformations
            rosPkgs.rmw_cyclonedds_cpp
            pkgs.python310Packages.transforms3d
            pkgs.xtensor
            pkgs.xsimd
            pkgs.flann
            pkgs.gdb
            pkgs.cairo
            pkgs.clang-tools
            pkgs.nim
            nimble-unwrapped
            nimble-ament-build
          ];

          buildInputs = [

          ];

          shellHook = ''
            set +u
            unset COLCON_CURRENT_PREFIX
          '';

          # RMW_IMPLEMENTATION="rmw_cyclonedds_cpp";
          FASTRTPS_DEFAULT_PROFILES_FILE = ./fastdds.xml;
        };
      }
    );
}
