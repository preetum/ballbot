FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/arduino_broadcaster/msg"
  "../msg_gen"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/arduino_broadcast.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
