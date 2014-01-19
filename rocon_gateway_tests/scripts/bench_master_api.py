#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/hydro-devel/rocon_gateway_tests/LICENSE 
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_gateway
import rocon_console.console as console
from gateway_msgs.msg import ConnectionType
import sys
import itertools
import rosgraph

##############################################################################
# Main
##############################################################################
#
# get_system_state retrieves lists of the form
# [ [topic1, [node1, ...]], [topic2, [node3, ...]], ... ]
# subscriber example
#   ['/d/chatter', ['/d/talker']]
#   {'subscriber': [{subscriber, name: /d/chatter, node: /d/listener, uri: http://snorriheim:58916/, topic_type: std_msgs/String}
# service example:
#   ['/b/add_two_ints', ['/b/add_two_ints_server']]
#   {service, name: /b/add_two_ints, node: /b/add_two_ints_server, uri: http://snorriheim:51256/, service_api: rosrpc://snorriheim:42463}
# action client example:
#   ['/a/fibonacci', ['/a/fibonacci_client']]
#   {action_client, name: /a/fibonacci, node: /a/fibonacci_client, uri: http://snorriheim:58280/, topic_type: actionlib_tutorials/Fibonacci}
# action server example
#   ['/d/fibonacci_server', ['/d/fibonacci_server']]
#   {action_server, name: /a/fibonacci_server, node: /a/fibonacci_server, uri: http://snorriheim:60001/, topic_type: actionlib_tutorials/Fibonacci}  

if __name__ == '__main__':
    rospy.init_node('bench_master_api')
    master = rosgraph.Master(rospy.get_name())
    connection_cache = rocon_gateway.ConnectionCache(master.getSystemState)
    times = {}
    names = ['start_time', 'get_system_state', 'extract_action_servers', 'extract_action_clients', 'get_pubs', 'get_subs', 'get_services', 'get_action_servers', 'get_action_clients']
    for name in names:
        times[name] = 0.0
    while not rospy.is_shutdown():
        try:
            connections = {}
            times['start_time'] = rospy.get_time()  # float version
            publishers, subscribers, services = connection_cache._get_system_state()
            times['get_system_state'] = rospy.get_time()
            action_servers, publishers, subscribers = connection_cache._get_action_servers(publishers, subscribers)
            times['extract_action_servers'] = rospy.get_time()
            action_clients, publishers, subscribers = connection_cache._get_action_clients(publishers, subscribers)
            times['extract_action_clients'] = rospy.get_time()
            connections[ConnectionType.PUBLISHER] = connection_cache._get_connections_from_pub_sub_list(publishers, ConnectionType.PUBLISHER)
            times['get_pubs'] = rospy.get_time()
            connections[ConnectionType.SUBSCRIBER] = connection_cache._get_connections_from_pub_sub_list(subscribers, ConnectionType.SUBSCRIBER)
            times['get_subs'] = rospy.get_time()
            connections[ConnectionType.SERVICE] = connection_cache._get_connections_from_service_list(services, ConnectionType.SERVICE)
            times['get_services'] = rospy.get_time()
            connections[ConnectionType.ACTION_SERVER] = connection_cache._get_connections_from_action_list(action_servers, ConnectionType.ACTION_SERVER)
            times['get_action_servers'] = rospy.get_time()
            connections[ConnectionType.ACTION_CLIENT] = connection_cache._get_connections_from_action_list(action_clients, ConnectionType.ACTION_CLIENT)
            times['get_action_clients'] = rospy.get_time()
            print(console.bold + "Benchmarks" + console.reset)
            for i in range(1, len(names)):
                print(console.cyan + "  %s: " % names[i] + console.yellow + "%s" % (times[names[i]] - times[names[i-1]]) + console.reset)
            print(console.green + "  total_time: " + console.magenta + "%s" % (times[names[-1]] - times[names[0]]) + console.reset)
        except Exception as e:
            print(console.red + "Lost contact with the master [%s][%s]" % (str(e), type(e)) + console.reset)
            break
