"""Make the ROS python packages importable for the offline tests in tools/.

The tests load nodes/navi_man as a module, which imports rospy, roslaunch and
the workspace's generated messages. From a sourced robot shell that just works;
from a plain shell (a test runner that does not source setup.bash) it does not.
This adds the same directories setup.bash would, only when they are missing.
No ROS master is contacted - the tests never init a node.
"""
import glob
import os
import sys

_SUB = os.path.join('lib', 'python3', 'dist-packages')


def ensure():
    try:
        import roslaunch  # noqa: F401
        import rospy  # noqa: F401
        return
    except ImportError:
        pass
    cands = []
    # devel spaces of the workspace(s) this checkout lives in, innermost first
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        cands.append(os.path.join(d, 'devel', _SUB))
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    cands.append(os.path.join('/home/vitulus/catkin_ws', 'devel', _SUB))
    cands.extend(sorted(glob.glob(os.path.join('/opt/ros', '*', _SUB))))
    for c in cands:
        if os.path.isdir(c) and c not in sys.path:
            sys.path.append(c)
