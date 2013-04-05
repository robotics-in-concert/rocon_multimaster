#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_utilities/LICENSE
#

# Too much effort to maintain here and in the package.xml
#__author__ = "Daniel Stonier, Jihoon Lee, Piyush Khandelwal"
#__copyright__ = "Copyright (c) 2012 Daniel Stonier, Yujin Robot"
#__license__ = "BSD"
#__version__ = "0.1.0"
#__maintainer__ = "Daniel Stonier"
#__email__ = "d.stonier@gmail.com"
#__date__ = "2012-11-30"

import console
from launch import main as launch
from gateways import create_gateway_rule, create_gateway_remote_rule
from ros_utilities import SubscriberProxy, find_resource
from exceptions import TimeoutExpiredError