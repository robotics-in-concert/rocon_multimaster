#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

###############################################################################
# Imports
###############################################################################

import netifaces
import rocon_python_wifi.iwlibs as pythonwifi
import rospy

from gateway_msgs.msg import ConnectionStatistics

###############################################################################
# Thread
###############################################################################


class NetworkInterfaceManager(object):

    '''
      Uses netifaces and pythonwifi to pull information about the network
      being used to connect to the hub
    '''

    def __init__(self, interface_name=None):
        '''
          Initializes the interface manager. If the interface_name is None or an
          empty string, then the interface manager attempts to auto detect the
          interface being used.
        '''
        self.interface_name, self.interface_type = \
            self.detect_network_interface(interface_name)

    def detect_network_interface(self, interface_name=''):
        '''
          Auto detects the network interface is none is supplied. If one is
          supplied, this function verifies that the interface is connected.

          :returns: the interface name and type if detected successfully
          :rtype: (str, int8) or None
        '''

        interfaces = []

        # Detect wireless interfaces first
        for interface in pythonwifi.getWNICnames():
            interfaces.append((interface, ConnectionStatistics.WIRELESS))

        # Detect wired network interfaces. This command also detects wireless
        # ones. Don't add them again
        for inf in netifaces.interfaces():
            if inf in pythonwifi.getWNICnames():
                continue
            addrs = netifaces.ifaddresses(inf)
            if not netifaces.AF_INET in addrs:
                continue
            else:
                address = addrs[netifaces.AF_INET][0]['addr']
                if address.split('.')[0] == '127':
                    continue
                interfaces.append((inf, ConnectionStatistics.WIRED))

        for interface in interfaces:
            if interface[0] == interface_name:
                return interface[0], interface[1]

        if interface_name:
            rospy.logwarn("Interface " + interface_name + " requested, but I " +
                          " was unabled to find it on the system. I will try " +
                          " and auto-detect the interface.")

        if len(interfaces) == 0:
            rospy.logerr("Unable to auto detect a single interface. Cannot " +
                         "send network information to hub.")
            return None, None
        elif len(interfaces) > 1:
            rospy.logerr("This machine is connected via multiple active " +
                         "interfaces. Detected: " + str(interfaces) + ". Please " +
                         "select a single interface using the network_interface " +
                         "param (or environment variable GATEWAY_NETWORK_INTERFACE " +
                         "if using the launcher). Cannot send network information to hub.")
            return None, None

        return interfaces[0][0], interfaces[0][1]

    def get_statistics(self):
        '''
          If the network interface manager is aware of which network interface
          was used to connect to the hub, then it prepares network statistics
          for that interface

          @return network_statistics
          @rtype gateway_msgs.ConnectionStatistics
        '''

        gateway_statistics = ConnectionStatistics()
        if not self.interface_name:
            gateway_statistics.network_info_available = False
            return gateway_statistics
        gateway_statistics.network_info_available = True
        gateway_statistics.network_type = self.interface_type
        if self.interface_type == ConnectionStatistics.WIRED:
            return gateway_statistics

        try:
            wifi = pythonwifi.Wireless(self.interface_name)
            gateway_statistics.wireless_bitrate = \
                float(wifi.wireless_info.getBitrate().value)  # Raw bitrate
            _, qual, _, _ = wifi.getStatistics()
        except IOError as e:
            rospy.logwarn("Gateway : not updating wireless statistics [wireless dropped out][%s]" % str(e))
            return gateway_statistics

        gateway_statistics.wireless_link_quality = int(qual.quality)
        # The -256 is a hack. The value returned by pythonwifi seems to be off.
        gateway_statistics.wireless_signal_level = \
            float(qual.signallevel) - 256.0
        gateway_statistics.wireless_noise_level = float(qual.noiselevel)

        return gateway_statistics
