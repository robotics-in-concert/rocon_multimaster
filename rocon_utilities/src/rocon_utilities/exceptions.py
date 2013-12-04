#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

##############################################################################
# Exceptions
##############################################################################


class TimeoutExpiredError(Exception):
    pass


class ResourceNotFoundException(IOError):
    """
      Resource Not Found Exception
    """
    pass


class InvalidPlatformInfoString(Exception):
    pass
