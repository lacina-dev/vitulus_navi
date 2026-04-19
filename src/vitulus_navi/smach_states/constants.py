#!/usr/bin/env python
"""
Constants for the mower SMACH state machine.
"""

# Distance thresholds
DISTANCE_TO_TARGET_THRESHOLD = 0.2  # Distance threshold for considering robot at target position

# Path parameters
SEGMENTIZE_DISTANCE = 0.03  # Distance between poses when segmenting paths
PATH_START_OFFSET = 0.18  # Offset distance for path start

# Publisher queue sizes
QUEUE_SIZE_DEFAULT = 1
QUEUE_SIZE_LOGS = 10

# Sound melodies
MELODY_SHORT_BEEP = 5
MELODY_BEEP = 1
MELODY_DOUBLE_BEEP = 2

# Mower states
MOWER_STATE_READY = 'READY'
MOWER_STATE_OFF = 'OFF'

# MBF outcomes
MBF_SUCCESS = 0
MBF_FAILURE = 1
MBF_CANCELED = 2

# Timeout durations (seconds)
TIMEOUT_WAIT_FOR_MBF = 30.0
TIMEOUT_WAIT_FOR_SERVICE = 5.0

# RPM tolerance for mower
RPM_TOLERANCE = 100  # Mower RPM tolerance range (+/-)