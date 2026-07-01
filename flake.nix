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
            ament-cmake-core
            ament-cmake
            ros-core
            rmw-cyclonedds-cpp
            rviz2
            rclcpp
            sensor-msgs
            nav-msgs      # Odometry
            std-msgs      # Header
            geometry-msgs # Quaternion/Pose/Twist/Vector3
            tf2-ros       # tf2 transforms
            tf2           # tf2 core
            tf2-geometry-msgs  # tf2 msg conversions
            slam-toolbox              # lidar SLAM, fixes yaw drift
            robot-localization        # EKF fusion node (optional but useful)
            # --- Added Missing Dependencies ---
            robot-state-publisher
            imu-tools
            joint-state-publisher   # Headless version avoids PyQt5 conflict
            ament-lint-auto
            ament-lint-common
            rko-lio
            # --- CLI Debugging Tools ---
            # (rqt suite removed temporarily due to pyqt5/Python 3.13 patch conflicts)
            ros2bag               # Command line tool for recording/playing data
            rosbag2-storage-default-plugins # SQLite3 storage backend for rosbag
            tf2-tools             # CLI tools like tf2_echo and tf2_monitor
          ];
        };
        python = pkgs.python3.withPackages (ps: [ ps.numpy ]);
      in {
        devShells.default = pkgs.mkShell {
          name = "seesaw-ros2";
          packages = [
            pkgs.colcon
            pkgs.pkg-config
            pkgs.cmake
            pkgs.gcc
            pkgs.gnumake
            pkgs.eigen    # Eigen3 headers
            pkgs.qt5.qtbase # RViz Qt deps
            python
            rosEnv
          ];
          shellHook = ''
            # Source properly first
            source ${rosEnv}/setup.sh 2>/dev/null || \
            source ${rosEnv}/share/${rosEnv.name}/local_setup.sh 2>/dev/null || true

            # Explicitly propagate AMENT paths into CMAKE so find_package works
            export CMAKE_PREFIX_PATH="${rosEnv}:$CMAKE_PREFIX_PATH"
            export AMENT_PREFIX_PATH="${rosEnv}:$AMENT_PREFIX_PATH"

            # Needed for ament_cmake_core specifically
            export CMAKE_MODULE_PATH="${rosEnv}/share/ament_cmake_core/cmake:$CMAKE_MODULE_PATH"
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default"/></Interfaces></General></Domain></CycloneDDS>'
            export QT_QPA_PLATFORM=xcb
            export QT_QPA_PLATFORM_PLUGIN_PATH=${pkgs.qt5.qtbase}/lib/qt-*/plugins/platforms
            export LD_LIBRARY_PATH=${pkgs.qt5.qtbase}/lib:${pkgs.eigen}/lib:${rosEnv}/lib:$LD_LIBRARY_PATH
            unset QTDIR QT_PLUGIN_PATH QT5DIR
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            alias cb="colcon build --packages-select seesaw_ros2"
            alias ci="colcon build --packages-up-to seesaw_ros2 --symlink-install"
            alias test="ros2 run seesaw_ros2 udp_reader & sleep 2 && rviz2"
            echo "=== Seesaw ROS2 C++ Ready ==="
            echo "- Debugging tools active: ros2 bag, tf2_tools, rviz2"
            echo "- mkdir -p seesaw_ros2/src"
            echo "- cd seesaw_ros2"
            echo "- cb  # or ci for symlink-install"
            echo "- source install/setup.bash"
            echo "- ros2 run seesaw_ros2 udp_reader --ros-args -p lidar_port:=12345 -p imu_port:=12346"
            echo "- In new term: rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"
            echo "- Send UDP to ports 12345(lidar)/12346(imu) from ESP32"
          '';
        };
      });
}
