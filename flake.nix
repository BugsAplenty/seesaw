{
  inputs.nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay";

  outputs = { self, nix-ros-overlay }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nix-ros-overlay {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
        ros = pkgs.rosPackages.humble;
        rosEnv = ros.buildEnv {
          paths = with ros; [
            ros-core
            rmw-fastrtps-cpp
            rviz2
            rclcpp  # C++ core
            sensor-msgs  # LaserScan msg
            geometry-msgs  # Optional: poses/frames
          ];
        };
        python = pkgs.python3.withPackages (ps: [ ps.numpy ]);
      in {
        devShells.default = pkgs.mkShell {
          name = "seesaw-ros2";
          packages = [
            pkgs.colcon
            python
            rosEnv
            pkgs.pkg-config
            pkgs.cmake
            pkgs.gcc
          ];
          shellHook = ''
            export QT_QPA_PLATFORM=xcb
            export QT_QPA_PLATFORM_PLUGIN_PATH=${pkgs.qt5.qtbase}/lib/qt-*/plugins/platforms
            export LD_LIBRARY_PATH=${pkgs.qt5.qtbase}/lib:${rosEnv}/lib:$LD_LIBRARY_PATH
            unset QTDIR QT_PLUGIN_PATH QT5DIR  # Clear conflicts
            eval "$(${rosEnv}/share/${rosEnv.name}/local_setup.sh)"
            export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
            export QT_QPA_PLATFORM=xcb
            alias cb="colcon build --packages-select seesaw_ros2"
            alias ci="colcon build --packages-up-to seesaw_ros2 --symlink-install"
            echo "=== Seesaw ROS2 Ready ==="
            echo "- cd seesaw_ros2 && cb"
            echo "- source install/setup.bash"
            echo "- ros2 run seesaw_ros2 udp_reader"
            echo "- rviz2"
          '';
        };
      });

  nixConfig.extra-substituters = [ "https://ros.cachix.org" ];
  nixConfig.extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
}
