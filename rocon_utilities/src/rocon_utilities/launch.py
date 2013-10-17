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
import tempfile

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


def _process_arg_tag(tag):
    '''
      Process the arg tag. Kind of hate replicating what roslaunch does with
      arg tags, but there's no easy way to pull roslaunch code.
    '''
    name = tag.get('name')  # returns None if not found.
    if name is None:
        console.error("<arg> tag must have a name attribute.")
        sys.exit(1)
    value = tag.get('value')
    default = tag.get('default')
    #print("Arg tag processing: (%s, %s, %s)" % (name, value, default))
    if value is not None and default is not None:
        console.error("<arg> tag must have one and only one of value/default attributes specified.")
        sys.exit(1)
    if value is None and default is None:
        console.error("<arg> tag must have one of value/default attributes specified.")
        sys.exit(1)
    if value is None:
        value = default
    return (name, value)


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
    # These are intended for re-use in launcher args via $(arg ...) like regular roslaunch
    for tag in root.findall('arg'):
        unused_name, unused_value = _process_arg_tag(tag)
    for launch in root.findall('launch'):
        parameters = {}
        parameters['args'] = []
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
        for tag in launch.findall('arg'):
            name, value = _process_arg_tag(tag)
            parameters['args'].append((name, value))
    return launchers


def parse_arguments():
    parser = argparse.ArgumentParser(description="Rocon's multiple master launcher.")
    terminal_group = parser.add_mutually_exclusive_group()
    terminal_group.add_argument('-k', '--konsole', default=False, action='store_true', help='spawn individual ros systems via multiple konsole terminals')
    terminal_group.add_argument('-g', '--gnome', default=False, action='store_true', help='spawn individual ros systems via multiple gnome terminals')
    parser.add_argument('--screen', action='store_true', help='run each roslaunch with the --screen option')
    parser.add_argument('--no-terminals', action='store_true', help='do not spawn terminals for each roslaunch')
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
    terminal = None
    if not args.no_terminals:
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
    temporary_launchers = []
    for launcher in launchers:
        console.pretty_println("Launching [%s, %s] on port %s" % (launcher['package'], launcher['name'], launcher['port']), console.bold)
        ##########################
        # Customise the launcher
        ##########################
        temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        print("Launching %s" % temp.name)
        launcher_filename = ros_utilities.find_resource(launcher['package'], launcher['name'])
        launch_text = '<launch>\n'
        if args.screen:
            launch_text += '  <param name="rocon/screen" value="true"/>\n'
        else:
            launch_text += '  <param name="rocon/screen" value="false"/>\n'
        launch_text += '  <include file="%s">\n' % launcher_filename
        for (arg_name, arg_value) in launcher['args']:
            launch_text += '    <arg name="%s" value="%s"/>\n' % (arg_name, arg_value)
        launch_text += '  </include>\n'
        launch_text += '</launch>\n'
        #print launch_text
        temp.write(launch_text)
        temp.close()  # unlink it later
        temporary_launchers.append(temp)
        ##########################
        # Start the terminal
        ##########################
        if terminal == 'konsole':
            p = subprocess.Popen([terminal, '--nofork', '--hold', '-e', "/bin/bash", "-c", "roslaunch %s --port %s %s" %
                              (launcher['options'], launcher['port'], temp.name)], preexec_fn=preexec)
        elif terminal == 'gnome-terminal.wrapper' or terminal == 'gnome-terminal':
            # --disable-factory inherits the current environment, bit wierd.
            p = subprocess.Popen(['gnome-terminal', '--disable-factory', '-e', "/bin/bash", "-e", "roslaunch %s --port %s %s" %
                              (launcher['options'], launcher['port'], temp.name)], preexec_fn=preexec)
        else:
            cmd = ["roslaunch"]
            if launcher['options']:
                cmd.append(launcher['options'])
            cmd.extend(["--port", launcher['port'], temp.name])
            p = subprocess.Popen(cmd, preexec_fn=preexec)
        processes.append(p)
    signal.pause()
    # Have to unlink them here rather than in the for loop above, because the whole gnome-terminal
    # subprocess takes a while to kick in (in the background) and the unlinking may occur before
    # it actually runs the roslaunch that needs the file.
    for temporary_launcher in temporary_launchers:
        os.unlink(temporary_launcher.name)
