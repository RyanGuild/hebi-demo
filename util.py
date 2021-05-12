from time import sleep
from hebi import Lookup

_lookup = None

def get_lookup() -> Lookup:
    global _lookup
    if _lookup is None:
        _lookup = Lookup()
        sleep(2.0)
    return _lookup