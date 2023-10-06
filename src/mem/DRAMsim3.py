# Copyright (c) 2013 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
from m5.citations import add_citation
from m5.objects.AbstractMemory import *
from m5.params import *


# A wrapper for DRAMSim3 multi-channel memory controller
class DRAMsim3(AbstractMemory):
    type = "DRAMsim3"
    cxx_header = "mem/dramsim3.hh"
    cxx_class = "gem5::memory::DRAMsim3"

    # A single port for now
    port = ResponsePort(
        "port for receiving requests fromthe CPU or other requestor"
    )

    configFile = Param.String(
        "ext/dramsim3/DRAMsim3/configs/DDR4_8Gb_x8_2400.ini",
        "The configuration file to use with DRAMSim3",
    )
    filePath = Param.String(
        "ext/dramsim3/DRAMsim3/", "Directory to prepend to file names"
    )


add_citation(
    DRAMsim3,
    """@article{Li:2020:dramsim3,
  author       = {Shang Li and
                  Zhiyuan Yang and
                  Dhiraj Reddy and
                  Ankur Srivastava and
                  Bruce L. Jacob},
  title        = {DRAMsim3: {A} Cycle-Accurate, Thermal-Capable {DRAM} Simulator},
  journal      = {{IEEE} Compututer Architecture Letters},
  volume       = {19},
  number       = {2},
  pages        = {110--113},
  year         = {2020},
  url          = {https://doi.org/10.1109/LCA.2020.2973991},
  doi          = {10.1109/LCA.2020.2973991}
}
""",
)
