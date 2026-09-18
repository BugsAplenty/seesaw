{
  description = "ROS 2 (Jazzy) dev environment for a z-lethic-style rocking lidar: Waveshare SC15 bus servo + Slamtec RPLidar A1M8";

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nix-ros-overlay, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };

        # Switch to "rolling" or "humble" here if you prefer another distro.
        ros = pkgs.rosPackages.jazzy;

        packages = [
          pkgs.colcon
          pkgs.git
          pkgs.usbutils # lsusb, for identifying your serial adapters
          pkgs.fish
        ]
        ++ (with pkgs.python3Packages; [
          pyserial   # talks to the SC15 servo bus
          numpy      # scan -> point cloud math
          setuptools
          wheel
        ])
        ++ (with ros; [
          ros-base           # rclcpp/rclpy, ros2cli, common msgs
          rclpy
          sensor-msgs
          std-msgs
          geometry-msgs
          tf2-ros
          launch
          launch-ros
          rviz2              # visualize the assembled 3D cloud
          slam-toolbox       # optional: 2D mapping from the same scans
          rplidar-ros
        ]);

        welcome = ''
          echo "ROS 2 (jazzy) rocking-lidar dev shell ready."
          echo "First run:  git clone --recurse-submodules https://github.com/Slamtec/sllidar_ros2 src/sllidar_ros2"
          echo "Then:       colcon build --symlink-install"
          echo "Run:        ros2 launch seesaw seesaw.launch.py"
        '';
      in
      {
        # Default shell: drops you into fish, with the full ROS 2 environment
        # (PATH, PYTHONPATH, AMENT_PREFIX_PATH, ...) inherited from the shellHook.
        devShells.default = pkgs.mkShell {
          name = "rocking-lidar";
          inherit packages;
        };
      });

  # Binary cache for ROS packages so you don't compile all of ROS from source.
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
