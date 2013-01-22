#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_utilities/LICENSE
#

##############################################################################
# Imports
##############################################################################

import os
import argparse
import subprocess
import signal
import sys
from time import sleep
import roslib
roslib.load_manifest('rocon_utilities')
import roslaunch
# Can call roslaunch.main(argv) directly
from .system import which, wait_pid
import rocon_utilities.console as console
import xml.etree.ElementTree as ElementTree

##############################################################################
# Global variables
##############################################################################

processes = []
roslaunch_pids = []

##############################################################################
# Methods
##############################################################################


def preexec():
    '''
      Don't forward signals.

      http://stackoverflow.com/questions/3791398/how-to-stop-python-from-propagating-signals-to-subprocesses
    '''
    os.setpgrp()  # setpgid(0,0)


def get_roslaunch_pid(parent_pid):
    '''
      Get the pid of the roslaunch process running in the terminal
      specified by the parent pid.
    '''
    ps_command = subprocess.Popen("ps -o pid -o comm --ppid %d --noheaders" % parent_pid, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    pids = []
    for pair in ps_output.split("\n")[:-1]:
        [pid, command] = pair.lstrip(' ').split(" ")
        if command == 'roslaunch':
            pids.append(int(pid))
        #os.kill(int(pid_str), sig)
    return pids


def signal_handler(sig, frame):
    global processes
    global roslaunch_pids
    # kill roslaunch's
    for pid in roslaunch_pids:
        try:
            os.kill(pid, signal.SIGHUP)
        except OSError:
            continue
    for pid in roslaunch_pids:
        print("Waiting for roslaanch to terminate [pid: %d]" % pid)
        wait_pid(pid)
        print("Terminated roslaunch [pid: %d]" % pid)
    sleep(1)
    # now kill konsoles
    for p in processes:
        p.terminate()


def parse_arguments():
    parser = argparse.ArgumentParser(description="Rocon's multiple master launcher.")
    terminal_group = parser.add_mutually_exclusive_group()
    terminal_group.add_argument('-k', '--konsole', action='store_false', help='spawn individual ros systems via multiple konsole terminals')
    terminal_group.add_argument('-g', '--gnome', action='store_true', help='spawn individual ros systems via multiple gnome terminals')
    # Force package, launcher pairs, I like this better than roslaunch style which is a bit vague
    parser.add_argument('package', nargs='?', default='', help='name of the package in which to find the concert launcher')
    parser.add_argument('launcher', nargs=1, help='name of the concert launch configuration (xml) file')
    #parser.add_argument('launchers', nargs='+', help='package and concert launch configuration (xml) file configurations, roslaunch style')
    args = parser.parse_args()
    return args


def main():
    global processes
    global roslaunch_pids
    signal.signal(signal.SIGINT, signal_handler)
    args = parse_arguments()
    if not which('konsole') and not which('gnome-terminal'):
        console.error("Cannot find a suitable terminal [konsole, gnome-termional]")
        sys.exit(1)
    terminal = None
    if args.konsole:
        if not which('konsole'):
            console.error("Cannot find 'konsole' [hint: try --gnome for gnome-terminal instead]")
            sys.exit(1)
        terminal = 'konsole'
    elif args.gnome:
        if not which('gnome-terminal'):
            console.error("Cannot find 'gnome-terminal' [hint: try --konsole instead]")
            sys.exit(1)
        terminal = 'gnome-terminal'
    if args.package == '':
        rocon_launcher = roslaunch.rlutil.resolve_launch_arguments(args.launcher)[0]
    else:
        rocon_launcher = roslaunch.rlutil.resolve_launch_arguments([args.package] + args.launcher)[0]
    #arguments = roslaunch.rlutil.resolve_launch_arguments(args.launchers)
    tree = ElementTree.parse(rocon_launcher)
    root = tree.getroot()
    # should check for root concert tag
    launchers = []
    for launch in root.findall('launch'):
        launchers.append((launch.get('package'), launch.get('name')))
    port = 11311
    for launcher in launchers:
        print("Launching [%s, %s] on port %s" % (launcher[0], launcher[1], port))
        p = subprocess.Popen([terminal, '--nofork', '--hold', '-e', "/bin/bash", "-c", "roslaunch --port %s %s %s" % (port, launcher[0], launcher[1])], preexec_fn=preexec)  # use --hold with konsole for debugging
        processes.append(p)
        port = port + 1
    sleep(1)
    for p in processes:
        roslaunch_pids.extend(get_roslaunch_pid(p.pid))
    signal.pause()
