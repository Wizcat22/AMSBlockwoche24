# CMake generated Testfile for 
# Source directory: /home/edurob/ams-bw-ws2425/catkin_ws/src/explore_lite
# Build directory: /home/edurob/ams-bw-ws2425/catkin_ws/build/explore_lite
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_explore_lite_roslaunch-check_launch "/home/edurob/ams-bw-ws2425/catkin_ws/build/explore_lite/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/edurob/ams-bw-ws2425/catkin_ws/build/explore_lite/test_results/explore_lite/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/edurob/ams-bw-ws2425/catkin_ws/build/explore_lite/test_results/explore_lite" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/edurob/ams-bw-ws2425/catkin_ws/build/explore_lite/test_results/explore_lite/roslaunch-check_launch.xml\" \"/home/edurob/ams-bw-ws2425/catkin_ws/src/explore_lite/launch\" ")
set_tests_properties(_ctest_explore_lite_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/edurob/ams-bw-ws2425/catkin_ws/src/explore_lite/CMakeLists.txt;81;roslaunch_add_file_check;/home/edurob/ams-bw-ws2425/catkin_ws/src/explore_lite/CMakeLists.txt;0;")
subdirs("gtest")
