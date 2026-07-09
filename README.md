# diff drive bot

simple 2 wheel differential drive robot with a front caster ball.

notes:
- base_footprint is the root link at z=0 (ground level). no inertia here so kdl parser doesnt complain!
- base_link sits 0.05m up
- 2 continuous joints for left and right wheels
- 1 caster ball for passive balance

launch in rviz:
ros2 launch launch/display.launch.py
