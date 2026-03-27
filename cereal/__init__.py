import os
import capnp
import rospkg

NO_TRAVERSAL_LIMIT = 2**64-1

package_path = rospkg.RosPack().get_path('lexus_platform')
cereal_path = os.path.join(package_path, 'src/cereal')
capnp.remove_import_hook()

log = capnp.load(os.path.join(cereal_path, "log.capnp"))
car = capnp.load(os.path.join(cereal_path, "car.capnp"))
custom = capnp.load(os.path.join(cereal_path, "custom.capnp"))

def log_from_bytes(dat: bytes, struct: capnp.lib.capnp._StructModule = log.Event) -> capnp.lib.capnp._DynamicStructReader:
    # Decodes the ZMQ message 
    with struct.from_bytes(dat, traversal_limit_in_words=NO_TRAVERSAL_LIMIT) as msg:
        return msg
