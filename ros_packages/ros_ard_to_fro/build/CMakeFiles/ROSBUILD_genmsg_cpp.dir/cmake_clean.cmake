FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ros_ard_to_fro/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/ros_ard_to_fro/odom_data.h"
  "../msg_gen/cpp/include/ros_ard_to_fro/drive_cmd.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
