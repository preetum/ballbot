FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/arduino_broadcaster/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/arduino_broadcaster/msg/__init__.py"
  "../src/arduino_broadcaster/msg/_arduino_data.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
