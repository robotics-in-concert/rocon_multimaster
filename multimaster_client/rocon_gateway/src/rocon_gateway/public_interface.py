#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

from gateway_comms.msg import Connection

##############################################################################
# Public Interface
##############################################################################

class PublicInterface(object):
    '''
      The public interface is the set of connections (pubs/subs/services/actions)
      that are exposed and made available for freely sharing with a multimaster system.
      
      It consists of two parts:
      
       * list of currently available connections to be shared
       * list of connections and filters that will be watched 
         and shared if they become available 
      
      Q) Filters should be in the form of whitelists/blacklists or just whitelists?
    '''
    def __init__(self):
        self.interface = dict()
        self.interface[Connection.PUBLISHER] = set()
        self.interface[Connection.SUBSCRIBER] = set()
        self.interface[Connection.SERVICE] = set()
        self.interface[Connection.ACTION_SERVER] = set()
        self.interface[Connection.ACTION_CLIENT] = set()

    ##########################################################################
    # Public Interfaces
    ##########################################################################

    def add(self,connection_type,connection):
        '''
        Attempt to add a connection to the public interface. 
        
        @param connection : a stringified connection representation (usually a triple)
        @type str
        @return failure if already present, success otherwise
        @rtype bool
        '''

        if connection in self.interface[connection_type]:
            return False
        else:
            self.interface[connection_type].add(connection)
        return True

    def remove(self,connection_type,connection):
        '''
        Attempt to remove a connection from the public interface.
        
        @param connection : a stringified connection representation (usually a triple)
        @type str
        @return failure if already present, success otherwise
        @rtype bool
        '''

        if not (connection in self.interface[connection_type]):
            return False
        else:
            self.interface[connection_type].remove(connection)
        return True

