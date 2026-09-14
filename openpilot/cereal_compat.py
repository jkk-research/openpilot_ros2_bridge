import os

import capnp


NO_TRAVERSAL_LIMIT = 2**64 - 1

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_HERE)
_CEREAL_DIR = os.path.join(_REPO_ROOT, "cereal")

capnp.remove_import_hook()
LOG = capnp.load(os.path.join(_CEREAL_DIR, "log.capnp"))


def log_from_bytes(dat: bytes, struct=LOG.Event):
    with struct.from_bytes(dat, traversal_limit_in_words=NO_TRAVERSAL_LIMIT) as msg:
        return msg
