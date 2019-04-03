### IMPORTS
import signal, SpecialMessages

### CLASS
# Class that makes sure the program exits cleanly
class GracefulKiller:
    
    kill_now = False
    _msg = SpecialMessages.SpecialMessages(5)
    
    def __init__(self):
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

    def exit_gracefully(self,signum, frame):
        self.kill_now = True
        self._msg.s("Exit Signal Recieved!")
