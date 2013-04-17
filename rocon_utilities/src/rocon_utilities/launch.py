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
import roslaunch

# Local imports
from .system import which, wait_pid
import rocon_utilities.console as console
import xml.etree.ElementTree as ElementTree
import ros_utilities

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
    pids = []
    if retcode == 0:
        for pair in ps_output.split("\n")[:-1]:
            [pid, command] = pair.lstrip(' ').split(" ")
            if command == 'roslaunch':
                pids.append(int(pid))
    else:
        # Presume this roslaunch was killed by ctrl-c or terminated already.
        # Am not worrying about classifying between the above presumption and real errors for now
        pass
    return pids


def signal_handler(sig, frame):
    global processes
    global roslaunch_pids
    for p in processes:
        roslaunch_pids.extend(get_roslaunch_pid(p.pid))
    # kill roslaunch's
    for pid in roslaunch_pids:
        try:
            os.kill(pid, signal.SIGHUP)
        except OSError:
            continue
    for pid in roslaunch_pids:
        console.pretty_println("Terminating roslaunch [pid: %d]" % pid, console.bold)
        wait_pid(pid)
        #console.pretty_println("Terminated roslaunch [pid: %d]" % pid, console.bold)
    sleep(1)
    # now kill konsoles
    for p in processes:
        p.terminate()


def parse_rocon_launcher(rocon_launcher, default_roslaunch_options):
    '''
      Parses an rocon multi-launcher (xml file).

      @param rocon_launcher : xml file in rocon_launch format
      @param default_roslaunch_options : options to pass to roslaunch (usually "--screen")
      @return launchers : list with launcher parameters as dictionary elements of the list.

      @raise IOError : if it can't find any of the individual launchers on the filesystem.
    '''
    tree = ElementTree.parse(rocon_launcher)
    root = tree.getroot()
    # should check for root concert tag
    launchers = []
    ports = []
    default_port = 11311
    for launch in root.findall('launch'):
        parameters = {}
        parameters['options'] = default_roslaunch_options
        parameters['package'] = launch.get('package')
        parameters['name'] = launch.get('name')
        parameters['path'] = ros_utilities.find_resource(parameters['package'], parameters['name'])  # raises an IO error if there is a problem.
        parameters['port'] = launch.get('port', str(default_port))
        if parameters['port'] == str(default_port):
            default_port += 1
        if parameters['port'] in ports:
            parameters['options'] = parameters['options'] + " " + "--wait"
        else:
            ports.append(parameters['port'])
        launchers.append(parameters)
    return launchers


def parse_arguments():
    parser = argparse.ArgumentParser(description="Rocon's multiple master launcher.")
    terminal_group = parser.add_mutually_exclusive_group()
    terminal_group.add_argument('-k', '--konsole', default=False, action='store_true', help='spawn individual ros systems via multiple konsole terminals')
    terminal_group.add_argument('-g', '--gnome', default=False, action='store_true', help='spawn individual ros systems via multiple gnome terminals')
    parser.add_argument('--screen', action='store_true', help='run each roslaunch with the --screen option')
    # Force package, launcher pairs, I like this better than roslaunch style which is a bit vague
    parser.add_argument('package', nargs='?', default='', help='name of the package in which to find the concert launcher')
    parser.add_argument('launcher', nargs=1, help='name of the concert launch configuration (xml) file')
    #parser.add_argument('launchers', nargs='+', help='package and concert launch configuration (xml) file configurations, roslaunch style')
    args = parser.parse_args()
    return args


def choose_terminal(gnome_flag, konsole_flag):
    '''
      Use ubuntu's x-terminal-emulator to choose the shell, or over-ride if it there is a flag.
    '''
    if konsole_flag:
        if not which('konsole'):
            console.error("Cannot find 'konsole' [hint: try --gnome for gnome-terminal instead]")
            sys.exit(1)
        return 'konsole'
    elif gnome_flag:
        if not which('gnome-terminal'):
            console.error("Cannot find 'gnome' [hint: try --konsole for konsole instead]")
            sys.exit(1)
        return 'gnome-terminal'
    else:
        if not which('x-terminal-emulator'):
            console.error("Cannot find 'x-terminal-emulator' [hint: try --gnome or --konsole instead]")
            sys.exit(1)
        p = subprocess.Popen([which('update-alternatives'), '--query', 'x-terminal-emulator'], stdout=subprocess.PIPE)
        terminal = None
        for line in p.stdout:
            if line.startswith("Value:"):
                terminal = os.path.basename(line.split()[1])
                break
        if terminal not in ["gnome-terminal", "gnome-terminal.wrapper", "konsole"]:
            console.warning("You are using an esoteric unsupported terminal [%s]" % terminal)
            if which('konsole'):
                terminal = 'konsole'
                console.warning(" --> falling back to 'konsole'")
            elif which('gnome-terminal'):
                console.warning(" --> falling back to 'gnome-terminal'")
                terminal = 'gnome-terminal'
            else:
                console.error("Unsupported terminal set for 'x-terminal-emulator' [%s][hint: try --gnome or --konsole instead]" % terminal)
                sys.exit(1)
        return terminal


def main():
    global processes
    global roslaunch_pids
    signal.signal(signal.SIGINT, signal_handler)
    args = parse_arguments()
    if not which('konsole') and not which('gnome-terminal')and not which('x-terminal-emulator'):
        console.error("Cannot find a suitable terminal [x-terminal-emulator, konsole, gnome-termional]")
        sys.exit(1)
    terminal = choose_terminal(args.gnome, args.konsole)

    if args.package == '':
        rocon_launcher = roslaunch.rlutil.resolve_launch_arguments(args.launcher)[0]
    else:
        rocon_launcher = roslaunch.rlutil.resolve_launch_arguments([args.package] + args.launcher)[0]
    if args.screen:
        roslaunch_options = "--screen"
    else:
        roslaunch_options = ""
    launchers = parse_rocon_launcher(rocon_launcher, roslaunch_options)
    for launcher in launchers:
        console.pretty_println("Launching [%s, %s] on port %s" % (launcher['package'], launcher['name'], launcher['port']), console.bold)
        if terminal == 'konsole':
            p = subprocess.Popen([terminal, '--nofork', '--hold', '-e', "/bin/bash", "-c", "roslaunch %s --port %s %s %s" %
                              (launcher['options'], launcher['port'], launcher['package'], launcher['name'])], preexec_fn=preexec)
        elif terminal == 'gnome-terminal.wrapper' or terminal == 'gnome-terminal':
            # --disable-factory inherits the current environment, bit wierd.
            p = subprocess.Popen(['gnome-terminal', '--disable-factory', '-e', "/bin/bash", "-e", "roslaunch %s --port %s %s %s" %
                              (launcher['options'], launcher['port'], launcher['package'], launcher['name'])], preexec_fn=preexec)
        processes.append(p)
    signal.pause()
