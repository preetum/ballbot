FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/arduino_broadcaster/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/arduino_data.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_arduino_data.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
