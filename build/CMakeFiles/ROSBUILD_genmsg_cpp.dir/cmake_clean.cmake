FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/AutoNav/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/AutoNav/predictInternal.h"
  "../msg_gen/cpp/include/AutoNav/filter_state.h"
  "../msg_gen/cpp/include/AutoNav/offsets.h"
  "../msg_gen/cpp/include/AutoNav/control_commands.h"
  "../msg_gen/cpp/include/AutoNav/obs_IMU_XYZ.h"
  "../msg_gen/cpp/include/AutoNav/obs_IMU_RPY.h"
  "../msg_gen/cpp/include/AutoNav/eulerpose.h"
  "../msg_gen/cpp/include/AutoNav/obs_tag.h"
  "../msg_gen/cpp/include/AutoNav/predictUpTo.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
