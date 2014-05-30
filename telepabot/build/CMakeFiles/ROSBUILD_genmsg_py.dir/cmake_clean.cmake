FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ui_alt/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/ui_alt/msg/__init__.py"
  "../src/ui_alt/msg/_src_tf_uialt.py"
  "../src/ui_alt/msg/_tf_uialt.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
