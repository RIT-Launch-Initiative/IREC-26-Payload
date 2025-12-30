file(REMOVE_RECURSE
  "json_grammar.h"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/json_grammar.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
