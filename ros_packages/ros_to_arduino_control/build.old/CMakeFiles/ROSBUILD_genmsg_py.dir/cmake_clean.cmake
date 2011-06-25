FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ros_to_arduino_control/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/ros_to_arduino_control/msg/__init__.py"
  "../src/ros_to_arduino_control/msg/_drive_cmd.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
