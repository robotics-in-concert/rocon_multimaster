#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_utilities/LICENSE
#

##############################################################################
# Imports
##############################################################################

import argparse
import subprocess
import sys
import roslib
roslib.load_manifest('rocon_utilities')
#import roslaunch
# Can call roslaunch.main(argv) directly
from .system import which
import rocon_utilities.console as console

##############################################################################
# Methods
##############################################################################


def parse_arguments():
    parser = argparse.ArgumentParser(description="Rocon's multiple master launcher.")
    terminal_group = parser.add_mutually_exclusive_group()
    terminal_group.add_argument('-k', '--konsole', action='store_false', help='spawn individual ros systems via multiple konsole terminals')
    terminal_group.add_argument('-g', '--gnome', action='store_true', help='spawn individual ros systems via multiple gnome terminals')
    args = parser.parse_args()
    return args


def main():
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
    subprocess.Popen([terminal, '-e', "/bin/bash", "-c", "roscore"], shell=True)
    #subprocess.Popen([terminal, '-e', '/bin/bash', '-c', 'roscore'], shell=True)
