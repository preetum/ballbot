FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ros_to_arduino_control/msg"
  "../msg_gen"
  "CMakeFiles/rospack_genmsg_all"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rospack_genmsg_all.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
