Machine Learning Example - Andrew Barrette

This is a playground for my introduction to reinforcement learning. The provided version (v1.20) uses advantage learning. The robot has 3 distance-limited visual rays which can have a value of 0 for nothing, 1 for red, or 2 for green. The robot receives +1 reward for touching the green "charging pad" from the front, -1 reward for touching a wall (in which case the robot turns yellow as an indicator), or -0.01 reward otherwise (thus resulting in impatience hopefully).

Executing:
Executing MLtestC1.20.exe requires glut32.dll to exist in the same folder. Upon executing, a terminal should appear showing simulation progress information, and a simulation should appear which depicts the simulated robot and environment.

Controls:
p	Pause simulation
space	Hide terminal progress information and display new frame every 200 time-steps (simulation runs much faster)
+/-	Zoom in/out
mouse	The green charging pad can be dragged around by clicking and dragging.