FILE(REMOVE_RECURSE
  "src/amee/msg"
  "msg_gen"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/amee/Pose.h"
  "msg_gen/cpp/include/amee/Motor.h"
  "msg_gen/cpp/include/amee/Odometry.h"
  "msg_gen/cpp/include/amee/KeyboardCommand.h"
  "msg_gen/cpp/include/amee/MovementCommand.h"
  "msg_gen/cpp/include/amee/IRDistances.h"
  "msg_gen/cpp/include/amee/Velocity.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
