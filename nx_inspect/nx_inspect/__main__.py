"""Enable ``python -m nx_inspect`` as an alias for the ``nx-inspect`` CLI."""

import sys

from .cli import main

if __name__ == "__main__":
    sys.exit(main())
