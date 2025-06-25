class SeerError(Exception):
    pass

class MessagePackError(SeerError):
    pass

class ConnectionError(SeerError):
    pass

class TimeoutError(SeerError):
    pass