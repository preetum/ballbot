FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/cmd_vel_simulator/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/goal_msg.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_goal_msg.lisp"
  "../msg_gen/lisp/drive_cmd.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_drive_cmd.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
