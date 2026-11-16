#!/usr/bin/env python3
"""T36 L1: a node that starts launch files keeps its own logging.

roslaunch.configure_logging() reloads the python logging configuration: the
handlers of the loggers `root` (the node's log file) and `rosout` (the
/rosout publisher of rospy, the console) are closed and replaced. After the
first roslaunch from inside navi_transform / navi_man the node's log file
ended and none of its warnings reached /rosout_agg (2026-11-16: the alarms of
the heading watchdogs were nowhere to be found). Both nodes now wrap the
call and put their handlers back.

  - the real roslaunch.configure_logging (logs into a temporary ROS_LOG_DIR):
    unwrapped it replaces the handlers (the fault), wrapped the handlers,
    levels and the file of the node are the same as before and a warning
    after it reaches the node's file and the `rosout` handler,
  - both nodes install the wrapper once, at import, and every call site in
    them goes through roslaunch.configure_logging.

Run:  python3 test_t36_l1_roslaunch_logging.py   (needs rospy / roslaunch - robot env)
"""
import importlib.machinery
import importlib.util
import logging
import os
import re
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

NAVI_MAN = os.path.join(HERE, '..', 'nodes', 'navi_man')


class Catch(logging.Handler):
    """Stands for rospy's RosOutHandler: keeps what would go to /rosout."""
    def __init__(self):
        logging.Handler.__init__(self)
        self.msgs = []
        self.closed_n = 0

    def emit(self, record):
        self.msgs.append((record.levelname, record.getMessage()))

    def close(self):
        self.closed_n += 1
        logging.Handler.close(self)


def node_logging(path):
    """Loggers as a rospy node has them: root -> the node's log file, rosout
    -> the /rosout handler; returns (root, rosout, file handler, catch)."""
    root, rosout = logging.getLogger(), logging.getLogger('rosout')
    for lg in (root, rosout):
        for h in list(lg.handlers):
            lg.removeHandler(h)
    fh = logging.handlers.RotatingFileHandler(path, 'a', 50000000, 4)
    fh.setFormatter(logging.Formatter('[%(name)s][%(levelname)s] %(message)s'))
    root.addHandler(fh)
    root.setLevel(logging.INFO)
    catch = Catch()
    rosout.addHandler(catch)
    rosout.setLevel(logging.DEBUG)
    rosout.propagate = True
    return root, rosout, fh, catch


def main():
    import logging.handlers  # noqa: F401
    fails = []

    def check(name, cond, detail=''):
        print('  %-72s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    tmp = tempfile.mkdtemp(prefix='t36_l1_')
    os.environ['ROS_LOG_DIR'] = tmp
    nt = R.load('navi_transform_t36_l1')
    import roslaunch
    wrapped = roslaunch.configure_logging
    original = wrapped._original

    print('the fault, with the call as roslaunch has it:')
    f0 = os.path.join(tmp, 'node_unwrapped.log')
    root, rosout, fh, catch = node_logging(f0)
    original('uuid-unwrapped')
    logging.getLogger('rosout').warning('after the unwrapped call')
    logging.getLogger().handlers and [h.flush() for h in logging.getLogger().handlers]
    check('the node\'s file handler and the /rosout handler are gone, the warning is lost',
          fh not in root.handlers and catch not in rosout.handlers and catch.msgs == []
          and 'after the unwrapped call' not in open(f0).read())
    other = [f for f in os.listdir(os.path.join(tmp, 'uuid-unwrapped')) if f.startswith('roslaunch-')]
    check('... it went to the roslaunch-<host>-<pid>.log instead',
          len(other) == 1 and 'after the unwrapped call'
          in open(os.path.join(tmp, 'uuid-unwrapped', other[0])).read())

    print('with the wrapper of the nodes:')
    f1 = os.path.join(tmp, 'node_wrapped.log')
    root, rosout, fh, catch = node_logging(f1)
    logging.getLogger('rosout').warning('before')
    wrapped('uuid-wrapped')
    check('handlers of root and rosout are the same objects, levels and propagate kept',
          root.handlers == [fh] and rosout.handlers == [catch] and root.level == logging.INFO
          and rosout.level == logging.DEBUG and rosout.propagate is True)
    logging.getLogger('rosout').warning('after the wrapped call')
    logging.getLogger('rosout').error('an error too')
    fh.flush()
    text = open(f1).read()
    check('a warning and an error after it reach the /rosout handler',
          catch.msgs == [('WARNING', 'before'), ('WARNING', 'after the wrapped call'),
                         ('ERROR', 'an error too')])
    check('... and the node\'s log file (reopened after roslaunch closed it)',
          'before' in text and 'after the wrapped call' in text and 'an error too' in text)
    wrapped('uuid-wrapped-2')
    wrapped('uuid-wrapped-3')
    logging.getLogger('rosout').warning('after three launches')
    fh.flush()
    check('three launches in a row (EKF, navsat, navsat again): still one handler each',
          root.handlers == [fh] and rosout.handlers == [catch]
          and catch.msgs[-1] == ('WARNING', 'after three launches')
          and 'after three launches' in open(f1).read())
    logging.getLogger('roslaunch').info('roslaunch says something')
    fh.flush()
    check('what roslaunch says in this process lands in the node\'s file, not on /rosout',
          'roslaunch says something' in open(f1).read()
          and not [m for m in catch.msgs if 'roslaunch says' in m[1]])
    for lg in (logging.getLogger(), logging.getLogger('rosout')):
        for h in list(lg.handlers):
            lg.removeHandler(h)
    wrapped('uuid-bare')
    check('logging not set up before the call: roslaunch\'s own configuration stays',
          len(logging.getLogger().handlers) == 1)

    print('the two nodes:')
    check('navi_transform installs the wrapper at import', wrapped._keeps_node_logging is True
          and nt.roslaunch.configure_logging is wrapped)
    src = open(R.NAVI_TF).read()
    calls = re.findall(r'^\s+roslaunch\.configure_logging\(uuid\)', src, re.M)
    check('navi_transform: its %d call sites go through roslaunch.configure_logging' % len(calls),
          len(calls) == 3)
    sys.path.insert(0, os.path.join(HERE, '..', 'src'))
    spec = importlib.util.spec_from_loader(
        'navi_man_t36_l1', importlib.machinery.SourceFileLoader('navi_man_t36_l1', NAVI_MAN))
    nm = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(nm)
        loaded = True
    except Exception as e:                       # a dev box without docker etc.
        loaded = False
        print('    navi_man not importable here (%s) - source check only' % e)
    if loaded:
        check('navi_man imported after navi_transform: the wrapper is not stacked',
              nm.roslaunch.configure_logging is wrapped and wrapped._original is original)
    msrc = open(NAVI_MAN).read()
    check('navi_man: the wrapper is installed at import, its call site goes through it',
          msrc.count('roslaunch.configure_logging = _keep_node_logging(roslaunch.configure_logging)') == 1
          and len(re.findall(r'^\s+roslaunch\.configure_logging\(uuid\)', msrc, re.M)) == 1)

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
