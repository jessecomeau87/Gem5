import os

os.system(
    "gem5-resources-manager gem5-resources-creator binary  -f \"{ 'id': 'riscv-hello', 'description': 'A 'Hello World!' binary, compiled to RISCV.', 'architecture': 'RISCV', 'size': 4674040, 'tags': [ 'hello' ], 'is_zipped': false, 'md5sum': '6d9494d22b90d817e826b0d762fda973', 'source': 'src/simple', 'url': 'https://storage.googleapis.com/dist.gem5.org/dist/develop/test-progs/hello/bin/riscv/linux/hello-20220728', 'code_examples': [ { 'example': 'https://github.com/gem5/gem5/tree/develop/configs/example/gem5_library/checkpoints/riscv-hello-restore-checkpoint.py', 'tested': true }, { 'example': 'https://github.com/gem5/gem5/tree/develop/configs/example/gem5_library/checkpoints/riscv-hello-save-checkpoint.py', 'tested': true }, { 'example': 'https://github.com/gem5/gem5/tree/develop/configs/example/gem5_library/riscvmatched-hello.py', 'tested': true } ], 'license': '', 'author': [], 'source_url': 'https://github.com/gem5/gem5-resources/tree/develop/src/simple', 'resource_version': '1.0.0', 'gem5_versions': [ '23.0' ], 'example_usage': '' }\""
)
