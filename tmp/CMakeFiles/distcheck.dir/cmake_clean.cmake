FILE(REMOVE_RECURSE
  "doc/viaopt.doxytag"
  "doc/doxygen.log"
  "doc/doxygen-html"
  "CMakeFiles/distcheck"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/distcheck.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
