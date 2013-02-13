FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/AutoNav/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/AutoNav/msg/__init__.py"
  "../src/AutoNav/msg/_predictInternal.py"
  "../src/AutoNav/msg/_filter_state.py"
  "../src/AutoNav/msg/_offsets.py"
  "../src/AutoNav/msg/_control_commands.py"
  "../src/AutoNav/msg/_obs_IMU_XYZ.py"
  "../src/AutoNav/msg/_obs_IMU_RPY.py"
  "../src/AutoNav/msg/_obs_tag.py"
  "../src/AutoNav/msg/_predictUpTo.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
