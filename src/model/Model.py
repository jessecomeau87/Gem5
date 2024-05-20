from m5.proxy import *
from m5.params import *
from m5.util.pybind import PyBindMethod
from m5.objects.ClockedObject import ClockedObject


class SpatterKernelType(Enum):
    vals = ["scatter", "gather"]


class Model(ClockedObject):
    type = "Model"
    cxx_header = "model/model.hh"
    cxx_class = "gem5::Model"

    system = Param.System(Parent.any, "System this model is a part of.")

    port = RequestPort("Port to send memory requests.")

    base_indexer_addr = Param.Addr(
        "Base physical address for indexer array. "
        "We assume indexer is contiguous in the PAS."
    )
    base_values_addr = Param.Addr(
        "Base physical address for values array. "
        "We assume values is contiguous in the PAS."
    )

    int_regfile_size = Param.Int("Size of the integer register file.")
    fp_regfile_size = Param.Int("Size of the floating point register file.")
    request_gen_latency = Param.Int(
        "Number of cycles to spend for creating a request."
    )
    request_gen_bandwidth = Param.Int("Number of requests generate per cycle.")
    request_buffer_size = Param.Int("Size of the request buffer.")
    send_bandwidth = Param.Int("Number of requests to send in parallel.")

    cxx_exports = [PyBindMethod("addKernel")]
