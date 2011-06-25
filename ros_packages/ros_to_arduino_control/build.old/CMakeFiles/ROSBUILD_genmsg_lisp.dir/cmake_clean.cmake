FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ros_to_arduino_control/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/drive_cmd.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_drive_cmd.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
