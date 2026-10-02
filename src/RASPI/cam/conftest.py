import os
import sys

# pytest desde src/RASPI/cam: `pure_pursuit`, `vision` y `wro_runtime` se
# importan igual que en la Pi.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
