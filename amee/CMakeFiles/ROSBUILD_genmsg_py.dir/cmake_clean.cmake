FILE(REMOVE_RECURSE
  "src/amee/msg"
  "msg_gen"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/amee/msg/__init__.py"
  "src/amee/msg/_Pose.py"
  "src/amee/msg/_Motor.py"
  "src/amee/msg/_Odometry.py"
  "src/amee/msg/_KeyboardCommand.py"
  "src/amee/msg/_MovementCommand.py"
  "src/amee/msg/_IRDistances.py"
  "src/amee/msg/_Velocity.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
