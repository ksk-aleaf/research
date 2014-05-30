FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ui_alt/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/ui_alt/src_tf_uialt.h"
  "../msg_gen/cpp/include/ui_alt/tf_uialt.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
