FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/cmd_vel_simulator/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/cmd_vel_simulator/msg/__init__.py"
  "../src/cmd_vel_simulator/msg/_goal_msg.py"
  "../src/cmd_vel_simulator/msg/_drive_cmd.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
