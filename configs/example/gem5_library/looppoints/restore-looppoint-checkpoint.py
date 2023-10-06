# Copyright (c) 2023 The Regents of the University of California
# All rights reserved.
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
"""
This configuration script shows an example of how to restore a checkpoint that
was taken for a LoopPoint simulation region in the example-restore.py.
All the LoopPoint information should be passed in through the JSON file
generated by the gem5 simulator when all the checkpoints were taken.

This script builds a more complex board than the board used for taking
checkpoints.

Usage
-----
```
./build/X86/gem5.opt \
    configs/example/gem5_library/looppoints/restore-looppoint-checkpoint.py
```
"""
import argparse

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires
from m5.stats import dump
from m5.stats import reset

requires(isa_required=ISA.X86)

parser = argparse.ArgumentParser(description="An restore checkpoint script.")

parser.add_argument(
    "--checkpoint-region",
    type=str,
    required=False,
    choices=(
        "1",
        "2",
        "3",
        "5",
        "6",
        "7",
        "8",
        "9",
        "10",
        "11",
        "12",
        "13",
        "14",
    ),
    default="1",
    help="The checkpoint region to restore from.",
)
args = parser.parse_args()

# The cache hierarchy can be different from the cache hierarchy used in taking
# the checkpoints
cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="32kB",
    l1i_size="32kB",
    l2_size="256kB",
)

# The memory structure can be different from the memory structure used in
# taking the checkpoints, but the size of the memory must be equal or larger.
memory = DualChannelDDR4_2400(size="2GB")

processor = SimpleProcessor(
    cpu_type=CPUTypes.TIMING,
    isa=ISA.X86,
    # The number of cores must be equal or greater than that used when taking
    # the checkpoint.
    num_cores=9,
)

board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

board.set_workload(
    obtain_resource(
        f"x86-matrix-multiply-omp-100-8-looppoint-region-{args.checkpoint_region}"
    )
)


# This generator will dump the stats and exit the simulation loop when the
# simulation region reaches its end. In the case there is a warmup interval,
# the simulation stats are reset after the warmup is complete.
def reset_and_dump():
    if len(board.get_looppoint().get_targets()) > 1:
        print("Warmup region ended. Resetting stats.")
        reset()
        yield False
    print("Region ended. Dumping stats.")
    dump()
    yield True


simulator = Simulator(
    board=board,
    on_exit_event={ExitEvent.SIMPOINT_BEGIN: reset_and_dump()},
)

simulator.run()
