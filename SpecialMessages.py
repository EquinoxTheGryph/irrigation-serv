### IMPORTS
from datetime import datetime

### CLASS
class SpecialMessages:
     
    ### COLORS
    _reset =    '\033[0m'
    _error =    '\033[0;31m'    # Level 1
    _special =  '\033[0;35m'    # Level 1
    _normal =   '\033[0m'       # Level 2
    _warning =  '\033[0;33m'    # Level 3
    _info =     '\033[0;34m'    # Level 4
    _debug =    '\033[0;32m'    # Level 5
    
    _log_level = 3 # Can be a range between 0 and 5, 0 meaning it will run silently (Not Recommended!), defaults to 3
    
    def __init__(self, value = 3):
        self._log_level = value
    
    # Set the log level
    def set_log_level(self, value):
        self._log_level = value
        print("Setting log level to %s (%s)" % (self._log_level, value))

    # Print special message
    def s(self, _msg):
        self._print(_msg, "SPC", self._special, 1)
    
    # Print error message
    def e(self, _msg):
        self._print(_msg, "ERR", self._error, 1)
    
    # Print normal message
    def n(self, _msg):
        self._print(_msg, "NRM", self._normal, 2)
    
    # Print warning message
    def w(self, _msg):
        self._print(_msg, "WRN", self._warning, 3)
    
    # Print info message
    def i(self, _msg):
        self._print(_msg, "INF", self._info, 4)
        
    # Print debug message
    def d(self, _msg):
        self._print(_msg, "DBG", self._debug, 5)
    
    # 
    def _print(self, _msg, _tag, _color, _min_log_level):
        if self._log_level >= _min_log_level:
            _time = datetime.now()
            print('%s - %s%s: %s%s'%(_time, _color, _tag, _msg, self._reset))
