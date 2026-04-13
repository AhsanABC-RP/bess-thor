#!/usr/bin/env python3
"""
Minimal ctypes wrapper around NVIDIA nvJPEG for GPU JPEG encoding.

Loads libnvjpeg.so.12 (ships with CUDA toolkit) and libcudart.so.12.
Designed to replace torchvision.io.encode_jpeg when PyTorch is unavailable
on the platform (e.g. Thor / JetPack 7).

HARD-FAILS at import time if either library is unavailable or probe fails.
No CPU fallback — silent fallback was the bug we're fixing.
"""

import ctypes
import sys
from ctypes import (
    POINTER, Structure, byref, c_char_p, c_float, c_int, c_size_t,
    c_uint, c_void_p, c_ubyte, cast,
)

NVJPEG_STATUS_SUCCESS = 0

# nvjpegBackend_t
NVJPEG_BACKEND_DEFAULT = 0
NVJPEG_BACKEND_HYBRID = 1
NVJPEG_BACKEND_GPU_HYBRID = 2
NVJPEG_BACKEND_HARDWARE = 3

# nvjpegInputFormat_t (from CUDA nvjpeg.h):
#   NVJPEG_INPUT_RGB=3, NVJPEG_INPUT_BGR=4, NVJPEG_INPUT_RGBI=5, NVJPEG_INPUT_BGRI=6
NVJPEG_INPUT_RGB = 3       # planar RGB (3 separate channels)
NVJPEG_INPUT_BGR = 4       # planar BGR
NVJPEG_INPUT_RGBI = 5      # interleaved RGB (H, W, 3)
NVJPEG_INPUT_BGRI = 6      # interleaved BGR

# nvjpegChromaSubsampling_t
NVJPEG_CSS_444 = 0
NVJPEG_CSS_422 = 1
NVJPEG_CSS_420 = 2

# nvjpegJpegEncoding_t
NVJPEG_ENCODING_BASELINE_DCT = 0xC0

NVJPEG_MAX_COMPONENT = 4


class _nvjpegImage_t(Structure):
    _fields_ = [
        ("channel", POINTER(c_ubyte) * NVJPEG_MAX_COMPONENT),
        ("pitch", c_size_t * NVJPEG_MAX_COMPONENT),
    ]


def _load_lib(names):
    last_err = None
    for n in names:
        try:
            return ctypes.CDLL(n)
        except OSError as e:
            last_err = e
    raise OSError(f"Could not load any of {names}: {last_err}")


class NvJpegEncoder:
    """GPU JPEG encoder using NVIDIA nvJPEG. One instance per process."""

    def __init__(self, quality: int = 85):
        self.quality = int(quality)

        # Load libraries
        self.cudart = _load_lib([
            "libcudart.so.12", "libcudart.so",
            "/usr/local/cuda/lib64/libcudart.so.12",
        ])
        self.nvjpeg = _load_lib([
            "libnvjpeg.so.12", "libnvjpeg.so",
            "/usr/local/cuda/lib64/libnvjpeg.so.12",
        ])

        # cudaMalloc(void **devPtr, size_t size)
        self.cudart.cudaMalloc.argtypes = [POINTER(c_void_p), c_size_t]
        self.cudart.cudaMalloc.restype = c_int
        # cudaFree(void *devPtr)
        self.cudart.cudaFree.argtypes = [c_void_p]
        self.cudart.cudaFree.restype = c_int
        # cudaMemcpy(dst, src, size, kind)  — kind=1 H2D
        self.cudart.cudaMemcpy.argtypes = [c_void_p, c_void_p, c_size_t, c_int]
        self.cudart.cudaMemcpy.restype = c_int
        # cudaDeviceSynchronize()
        self.cudart.cudaDeviceSynchronize.argtypes = []
        self.cudart.cudaDeviceSynchronize.restype = c_int

        # nvjpegCreateSimple(nvjpegHandle_t *handle)
        self.nvjpeg.nvjpegCreateSimple.argtypes = [POINTER(c_void_p)]
        self.nvjpeg.nvjpegCreateSimple.restype = c_int
        # nvjpegEncoderStateCreate(handle, state, stream)
        self.nvjpeg.nvjpegEncoderStateCreate.argtypes = [
            c_void_p, POINTER(c_void_p), c_void_p
        ]
        self.nvjpeg.nvjpegEncoderStateCreate.restype = c_int
        # nvjpegEncoderParamsCreate(handle, params, stream)
        self.nvjpeg.nvjpegEncoderParamsCreate.argtypes = [
            c_void_p, POINTER(c_void_p), c_void_p
        ]
        self.nvjpeg.nvjpegEncoderParamsCreate.restype = c_int
        # nvjpegEncoderParamsSetQuality(params, quality, stream)
        self.nvjpeg.nvjpegEncoderParamsSetQuality.argtypes = [
            c_void_p, c_int, c_void_p
        ]
        self.nvjpeg.nvjpegEncoderParamsSetQuality.restype = c_int
        # nvjpegEncoderParamsSetSamplingFactors(params, subsampling, stream)
        self.nvjpeg.nvjpegEncoderParamsSetSamplingFactors.argtypes = [
            c_void_p, c_int, c_void_p
        ]
        self.nvjpeg.nvjpegEncoderParamsSetSamplingFactors.restype = c_int
        # nvjpegEncoderParamsSetEncoding(params, encoding, stream)
        self.nvjpeg.nvjpegEncoderParamsSetEncoding.argtypes = [
            c_void_p, c_int, c_void_p
        ]
        self.nvjpeg.nvjpegEncoderParamsSetEncoding.restype = c_int
        # nvjpegEncoderParamsSetOptimizedHuffman(params, optimized, stream)
        self.nvjpeg.nvjpegEncoderParamsSetOptimizedHuffman.argtypes = [
            c_void_p, c_int, c_void_p
        ]
        self.nvjpeg.nvjpegEncoderParamsSetOptimizedHuffman.restype = c_int
        # nvjpegEncodeImage(handle, state, params, source, input_format, width, height, stream)
        self.nvjpeg.nvjpegEncodeImage.argtypes = [
            c_void_p, c_void_p, c_void_p, POINTER(_nvjpegImage_t),
            c_int, c_int, c_int, c_void_p,
        ]
        self.nvjpeg.nvjpegEncodeImage.restype = c_int
        # nvjpegEncodeRetrieveBitstream(handle, state, data, length, stream)
        self.nvjpeg.nvjpegEncodeRetrieveBitstream.argtypes = [
            c_void_p, c_void_p, POINTER(c_ubyte), POINTER(c_size_t), c_void_p
        ]
        self.nvjpeg.nvjpegEncodeRetrieveBitstream.restype = c_int

        # Initialize nvJPEG handle, encoder state, params
        self.handle = c_void_p()
        self._check(
            self.nvjpeg.nvjpegCreateSimple(byref(self.handle)),
            "nvjpegCreateSimple",
        )

        self.state = c_void_p()
        self._check(
            self.nvjpeg.nvjpegEncoderStateCreate(
                self.handle, byref(self.state), None
            ),
            "nvjpegEncoderStateCreate",
        )

        self.params = c_void_p()
        self._check(
            self.nvjpeg.nvjpegEncoderParamsCreate(
                self.handle, byref(self.params), None
            ),
            "nvjpegEncoderParamsCreate",
        )
        self._check(
            self.nvjpeg.nvjpegEncoderParamsSetQuality(
                self.params, self.quality, None
            ),
            "nvjpegEncoderParamsSetQuality",
        )
        self._check(
            self.nvjpeg.nvjpegEncoderParamsSetSamplingFactors(
                self.params, NVJPEG_CSS_420, None
            ),
            "nvjpegEncoderParamsSetSamplingFactors",
        )
        self._check(
            self.nvjpeg.nvjpegEncoderParamsSetEncoding(
                self.params, NVJPEG_ENCODING_BASELINE_DCT, None
            ),
            "nvjpegEncoderParamsSetEncoding",
        )
        self._check(
            self.nvjpeg.nvjpegEncoderParamsSetOptimizedHuffman(
                self.params, 0, None
            ),
            "nvjpegEncoderParamsSetOptimizedHuffman",
        )

        # Reusable device buffer grown on demand
        self._dev_ptr = c_void_p(0)
        self._dev_size = 0

        # Probe encode with a 16x16 black RGB frame to fail fast at init
        probe = bytes(16 * 16 * 3)
        try:
            self.encode(probe, 16, 16, input_format=NVJPEG_INPUT_RGBI)
        except Exception as exc:
            raise RuntimeError(f"nvJPEG probe encode failed: {exc}")

    def _check(self, status: int, what: str):
        if status != NVJPEG_STATUS_SUCCESS:
            raise RuntimeError(f"{what} failed with nvJPEG status {status}")

    def _ensure_device_buffer(self, nbytes: int):
        if nbytes <= self._dev_size:
            return
        if self._dev_ptr.value:
            self.cudart.cudaFree(self._dev_ptr)
        self._dev_ptr = c_void_p(0)
        rc = self.cudart.cudaMalloc(byref(self._dev_ptr), nbytes)
        if rc != 0 or not self._dev_ptr.value:
            raise RuntimeError(f"cudaMalloc({nbytes}) failed rc={rc}")
        self._dev_size = nbytes

    def encode(
        self,
        host_buffer: bytes,
        width: int,
        height: int,
        input_format: int = NVJPEG_INPUT_RGBI,
    ) -> bytes:
        """Encode a host-side interleaved 3-channel image to a JPEG byte string.

        host_buffer: contiguous HxWx3 uint8 bytes (RGB for RGBI, BGR for BGRI).
        """
        nbytes = width * height * 3
        if len(host_buffer) != nbytes:
            raise ValueError(
                f"host_buffer size {len(host_buffer)} != expected {nbytes}"
            )

        self._ensure_device_buffer(nbytes)

        # H2D copy — cudaMemcpyHostToDevice = 1
        src = (c_ubyte * nbytes).from_buffer_copy(host_buffer)
        rc = self.cudart.cudaMemcpy(
            self._dev_ptr, cast(src, c_void_p), nbytes, 1
        )
        if rc != 0:
            raise RuntimeError(f"cudaMemcpy H2D failed rc={rc}")

        # Build nvjpegImage_t: for interleaved RGB, channel[0] = device ptr,
        # pitch[0] = row stride (width*3), remaining channels zero.
        img = _nvjpegImage_t()
        img.channel[0] = cast(self._dev_ptr, POINTER(c_ubyte))
        img.pitch[0] = width * 3
        for i in range(1, NVJPEG_MAX_COMPONENT):
            img.channel[i] = cast(c_void_p(0), POINTER(c_ubyte))
            img.pitch[i] = 0

        self._check(
            self.nvjpeg.nvjpegEncodeImage(
                self.handle, self.state, self.params,
                byref(img), input_format, width, height, None,
            ),
            "nvjpegEncodeImage",
        )

        # Retrieve bitstream: first call with NULL data to get length
        length = c_size_t(0)
        self._check(
            self.nvjpeg.nvjpegEncodeRetrieveBitstream(
                self.handle, self.state, None, byref(length), None
            ),
            "nvjpegEncodeRetrieveBitstream(size)",
        )
        self.cudart.cudaDeviceSynchronize()

        buf = (c_ubyte * length.value)()
        self._check(
            self.nvjpeg.nvjpegEncodeRetrieveBitstream(
                self.handle, self.state, buf, byref(length), None
            ),
            "nvjpegEncodeRetrieveBitstream(data)",
        )
        return bytes(buf[:length.value])


def _self_test():
    enc = NvJpegEncoder(quality=85)
    # 64x64 random-ish gradient
    data = bytes((x * 3) % 256 for x in range(64 * 64 * 3))
    jpeg = enc.encode(data, 64, 64, input_format=NVJPEG_INPUT_RGBI)
    assert jpeg[:2] == b"\xff\xd8", f"Not a JPEG: {jpeg[:4].hex()}"
    print(f"OK: encoded 64x64 -> {len(jpeg)} bytes JPEG")


if __name__ == "__main__":
    try:
        _self_test()
    except Exception as exc:
        print(f"FATAL: nvJPEG self-test failed: {exc}", file=sys.stderr)
        sys.exit(2)
