FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/navigation/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/navigation/msg/__init__.py"
  "../src/navigation/msg/_goal_msg.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
