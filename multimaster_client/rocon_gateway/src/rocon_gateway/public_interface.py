#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/multimaster_client/rocon_gateway/LICENSE 
#

##############################################################################
# Imports
##############################################################################

from .utils import Connection, connectionTypeString

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
        self.interface = {}
        self.interface["topic"] = []       # later split into pub, sub
        self.interface["service"] = []

    ##########################################################################
    # Public Interfaces
    ##########################################################################

    def add(self,connection):
        '''
        Attempt to add a connection to the public interface. 
        
        @param connection : a stringified connection representation (usually a triple)
        @type str
        @return failure if already present, success otherwise
        @rtype bool
        @raise ConenctionTypeError : if the stringified representation is invalid
        '''
        identifier = connectionTypeString(connection)
        if identifier not in ['topic', 'service']:  # action not yet implemented
            raise ConnectionTypeError("trying to add an invalid connection type to the public interface [%s]"%connection)

        if connection in self.interface[identifier]:
            return False
        else:
            self.interface[identifier].append(connection)
        return True

    def remove(self,connection):
        '''
        Attempt to remove a connection from the public interface.
        
        @param connection : a stringified connection representation (usually a triple)
        @type str
        @return failure if already present, success otherwise
        @rtype bool
        @raise ConenctionTypeError : if the stringified representation is invalid
        '''

        identifier = connectionTypeString(connection)
        if identifier not in ['topic', 'service']:  # action not yet implemented
            raise ConnectionTypeError("trying to remove an invalid connection type from the public interface [%s]"%connection)

        if not (connection in self.interface[identifier]):
            return False
        else:
            self.interface[identifier].remove(connection)
        return True

    ##########################################################################
    # Filters
    ##########################################################################

    # ToDo
