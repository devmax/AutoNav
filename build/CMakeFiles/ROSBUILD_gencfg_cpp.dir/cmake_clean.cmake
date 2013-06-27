FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/AutoNav/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/AutoNav/CircleParamsConfig.h"
  "../docs/CircleParamsConfig.dox"
  "../docs/CircleParamsConfig-usage.dox"
  "../src/AutoNav/cfg/CircleParamsConfig.py"
  "../docs/CircleParamsConfig.wikidoc"
  "../cfg/cpp/AutoNav/StateestimationParamsConfig.h"
  "../docs/StateestimationParamsConfig.dox"
  "../docs/StateestimationParamsConfig-usage.dox"
  "../src/AutoNav/cfg/StateestimationParamsConfig.py"
  "../docs/StateestimationParamsConfig.wikidoc"
  "../cfg/cpp/AutoNav/AutopilotParamsConfig.h"
  "../docs/AutopilotParamsConfig.dox"
  "../docs/AutopilotParamsConfig-usage.dox"
  "../src/AutoNav/cfg/AutopilotParamsConfig.py"
  "../docs/AutopilotParamsConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
