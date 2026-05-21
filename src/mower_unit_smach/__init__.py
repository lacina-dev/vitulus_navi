"""
mower_unit_smach — modular SMACH state machine for autonomous mowing.

Submodules:
  helpers    — shared utilities (TF, MBF, path segmentisation)
  publishers — centralised ROS publishers
  states     — all SMACH state classes
  monitors   — MonitorState callbacks (weather, battery, temp, stop)
  recovery   — blocked motor & navigation recovery sub-SMs
  top_level  — CriticalError, TerminalError, ReturnToDock
  mission    — mission child SM, mission concurrence, wait-for-program
"""
