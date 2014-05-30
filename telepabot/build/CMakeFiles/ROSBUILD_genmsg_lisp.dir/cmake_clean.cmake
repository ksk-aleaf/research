FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ui_alt/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/src_tf_uialt.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_src_tf_uialt.lisp"
  "../msg_gen/lisp/tf_uialt.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_tf_uialt.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
