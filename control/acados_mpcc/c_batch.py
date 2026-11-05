"""Compile and call one batched C entry point for the generated Slash solver."""
import ctypes
import os
from pathlib import Path
import subprocess

import numpy as np


_DOUBLE_PTR = ctypes.POINTER(ctypes.c_double)


class CBatch:
    def __init__(self, solver):
        if solver.ocp.name != 'slash_mpcc':
            raise ValueError('C batch backend supports the generated Slash solver only')
        generated = Path(solver.ocp.code_gen_options.code_export_directory).resolve()
        acados = Path(os.environ['ACADOS_SOURCE_DIR']).resolve()
        output = generated / 'libslash_mpcc_c_batch.so'
        command = [
            'cc', '-O2', '-std=c11', '-fPIC', '-shared',
            '-I' + str(generated),
            '-I' + str(acados),
            '-I' + str(acados / 'interfaces'),
            '-I' + str(acados / 'external'),
            '-I' + str(acados / 'external' / 'blasfeo' / 'include'),
            '-I' + str(acados / 'external' / 'hpipm' / 'include'),
            str(Path(__file__).with_name('c_batch_bridge.c')),
            '-L' + str(generated), '-lacados_ocp_solver_slash_mpcc',
            '-L' + str(acados / 'lib'), '-lacados',
            '-Wl,-rpath,$ORIGIN', '-Wl,-rpath,' + str(acados / 'lib'),
            '-o', str(output),
        ]
        compiled = subprocess.run(command, capture_output=True, text=True)
        if compiled.returncode:
            raise RuntimeError(f'C batch compilation failed: {compiled.stderr}')
        self.library = ctypes.CDLL(str(output))
        self.function = self.library.slash_mpcc_c_batch_step
        self.function.argtypes = [
            ctypes.c_void_p, ctypes.c_int, _DOUBLE_PTR, _DOUBLE_PTR,
            _DOUBLE_PTR, _DOUBLE_PTR, _DOUBLE_PTR, _DOUBLE_PTR,
            _DOUBLE_PTR, _DOUBLE_PTR, ctypes.POINTER(ctypes.c_int),
        ]
        self.function.restype = ctypes.c_int
        self.solver = solver

    def step(self, x_guess, u_guess, parameters, lower_h, upper_h):
        n = self.solver.N
        arrays = [
            np.ascontiguousarray(a, dtype=np.float64)
            for a in (x_guess, u_guess, parameters, lower_h, upper_h)
        ]
        nx = int(self.solver.ocp.model.x.numel())
        expected = [(n + 1, nx), (n, 3), (n + 1, 4), (n + 1,), (n + 1,)]
        if [a.shape for a in arrays] != expected:
            raise ValueError('C batch input dimensions do not match the generated solver')
        x_out = np.empty_like(arrays[0])
        u_out = np.empty_like(arrays[1])
        timings = np.zeros(3, dtype=np.float64)
        status = ctypes.c_int()
        pointers = [a.ctypes.data_as(_DOUBLE_PTR) for a in arrays]
        result = self.function(
            self.solver.capsule, n, *pointers,
            x_out.ctypes.data_as(_DOUBLE_PTR),
            u_out.ctypes.data_as(_DOUBLE_PTR),
            timings.ctypes.data_as(_DOUBLE_PTR), ctypes.byref(status),
        )
        if result:
            raise RuntimeError(f'C batch acados API failed with code {result}')
        return status.value, x_out, u_out, timings
