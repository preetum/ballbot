FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ros_ard_to_fro/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/ros_ard_to_fro/msg/__init__.py"
  "../src/ros_ard_to_fro/msg/_odom_data.py"
  "../src/ros_ard_to_fro/msg/_drive_cmd.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
