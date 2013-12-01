#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

import os
import time
import errno
import threading
import subprocess
# Local imports
from .exceptions import TimeoutExpiredError

##############################################################################
# Methods
##############################################################################


def which(program):

    def is_exe(fpath):
        return os.path.isfile(fpath) and os.access(fpath, os.X_OK)

    fpath, unused_fname = os.path.split(program)
    if fpath:
        if is_exe(program):
            return program
    else:
        for path in os.environ["PATH"].split(os.pathsep):
            path = path.strip('"')
            exe_file = os.path.join(path, program)
            if is_exe(exe_file):
                return exe_file

    return None

##############################################################################
# Subprocess
##############################################################################


class Popen(object):
    '''
      Use this if you want to attach a postexec function to popen (which
      is not supported by popen at all).
    '''
    __slots__ = [
            '_proc',
            '_thread',
            'terminate'
        ]

    def __init__(self, popen_args, preexec_fn=None, postexec_fn=None):
        '''
          @param popen_args : list/tuple of usual popen args
          @type list/tuple

          @param preexec_fn : usual popen pre-exec function
          @type method with no args

          @param postexec_fn : the callback which we support for postexec.
          @type method with no args
        '''
        self._proc = None
        self._thread = threading.Thread(target=self._run_in_thread, args=(popen_args, preexec_fn, postexec_fn))
        self._thread.start()

    def send_signal(self, sig):
        self._proc.send_signal(sig)

    def terminate(self):
        '''
          @raise OSError if the process has already shut down.
        '''
        return self._proc.terminate() if self._proc is not None else None

    def _run_in_thread(self, popen_args, preexec_fn, postexec_fn):
        '''
          Worker function for the thread, creates the subprocess itself.
        '''
        if preexec_fn is not None:
            self._proc = subprocess.Popen(popen_args, preexec_fn=preexec_fn)
            print("PID: %s" % self._proc.pid)
        else:
            self._proc = subprocess.Popen(popen_args)
        self._proc.wait()
        if postexec_fn is not None:
            postexec_fn()
        return

##############################################################################
# PID
##############################################################################


def pid_exists(pid):
    """Check whether pid exists in the current process table."""
    if pid < 0:
        return False
    try:
        os.kill(pid, 0)
    except OSError, e:
        return e.errno == errno.EPERM
    else:
        return True


def wait_pid(pid, timeout=None):
    """Wait for process with pid 'pid' to terminate and return its
    exit status code as an integer.

    If pid is not a children of os.getpid() (current process) just
    waits until the process disappears and return None.

    If pid does not exist at all return None immediately.

    Raise TimeoutExpiredError on timeout expired (if specified).
    """
    def check_timeout(delay):
        if timeout is not None:
            if time.time() >= stop_at:
                raise TimeoutExpiredError
        time.sleep(delay)
        return min(delay * 2, 0.04)

    if timeout is not None:
        waitcall = lambda: os.waitpid(pid, os.WNOHANG)
        stop_at = time.time() + timeout
    else:
        waitcall = lambda: os.waitpid(pid, 0)

    delay = 0.0001
    while 1:
        try:
            retpid, status = waitcall()
        except OSError, err:
            if err.errno == errno.EINTR:
                delay = check_timeout(delay)
                continue
            elif err.errno == errno.ECHILD:
                # This has two meanings:
                # - pid is not a child of os.getpid() in which case
                #   we keep polling until it's gone
                # - pid never existed in the first place
                # In both cases we'll eventually return None as we
                # can't determine its exit status code.
                while 1:
                    if pid_exists(pid):
                        delay = check_timeout(delay)
                    else:
                        return
            else:
                raise
        else:
            if retpid == 0:
                # WNOHANG was used, pid is still running
                delay = check_timeout(delay)
                continue
            # process exited due to a signal; return the integer of
            # that signal
            if os.WIFSIGNALED(status):
                return os.WTERMSIG(status)
            # process exited using exit(2) system call; return the
            # integer exit(2) system call has been called with
            elif os.WIFEXITED(status):
                return os.WEXITSTATUS(status)
            else:
                # should never happen
                raise RuntimeError("unknown process exit status")