FILE(REMOVE_RECURSE
  "src/amee/msg"
  "msg_gen"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/Pose.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_Pose.lisp"
  "msg_gen/lisp/Motor.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_Motor.lisp"
  "msg_gen/lisp/Odometry.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_Odometry.lisp"
  "msg_gen/lisp/KeyboardCommand.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_KeyboardCommand.lisp"
  "msg_gen/lisp/MovementCommand.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_MovementCommand.lisp"
  "msg_gen/lisp/IRDistances.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_IRDistances.lisp"
  "msg_gen/lisp/Velocity.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_Velocity.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
