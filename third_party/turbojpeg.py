# -*- coding: UTF-8 -*-
#
# PyTurboJPEG - A Python wrapper of libjpeg-turbo for decoding and encoding JPEG image.
#
# Copyright (c) 2018-2021, Lilo Huang. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

__author__ = 'Lilo Huang <kuso.cc@gmail.com>'
__version__ = '1.5.1'

from ctypes import *
from ctypes.util import find_library
import platform
import numpy as np
import math
import warnings
import os

# default libTurboJPEG library path
DEFAULT_LIB_PATHS = {
    'Darwin': ['/usr/local/opt/jpeg-turbo/lib/libturbojpeg.dylib'],
    'Linux': [
        '/usr/lib/x86_64-linux-gnu/libturbojpeg.so.0',
        '/usr/lib/aarch64-linux-gnu/libturbojpeg.so.0',
        '/opt/libjpeg-turbo/lib64/libturbojpeg.so'
    ],
    'Windows': ['C:/libjpeg-turbo64/bin/turbojpeg.dll']
}

# error codes
# see details in https://github.com/libjpeg-turbo/libjpeg-turbo/blob/master/turbojpeg.h
TJERR_WARNING = 0
TJERR_FATAL = 1

# color spaces
# see details in https://github.com/libjpeg-turbo/libjpeg-turbo/blob/master/turbojpeg.h
TJCS_RGB = 0
TJCS_YCbCr = 1
TJCS_GRAY = 2
TJCS_CMYK = 3
TJCS_YCCK = 4

# pixel formats
# see details in https://github.com/libjpeg-turbo/libjpeg-turbo/blob/master/turbojpeg.h
TJPF_RGB = 0
TJPF_BGR = 1
TJPF_RGBX = 2
TJPF_BGRX = 3
TJPF_XBGR = 4
TJPF_XRGB = 5
TJPF_GRAY = 6
TJPF_RGBA = 7
TJPF_BGRA = 8
TJPF_ABGR = 9
TJPF_ARGB = 10
TJPF_CMYK = 11

# chrominance subsampling options
# see details in https://github.com/libjpeg-turbo/libjpeg-turbo/blob/master/turbojpeg.h
TJSAMP_444 = 0
TJSAMP_422 = 1
TJSAMP_420 = 2
TJSAMP_GRAY = 3
TJSAMP_440 = 4
TJSAMP_411 = 5

# transform operations
# see details in https://github.com/libjpeg-turbo/libjpeg-turbo/blob/master/turbojpeg.h
TJXOP_NONE = 0
TJXOP_HFLIP = 1
TJXOP_VFLIP = 2
TJXOP_TRANSPOSE = 3
TJXOP_TRANSVERSE = 4
TJXOP_ROT90 = 5
TJXOP_ROT180 = 6
TJXOP_ROT270 = 7

# transform options
# see details in https://github.com/libjpeg-turbo/libjpeg-turbo/blob/master/turbojpeg.h
TJXOPT_PERFECT = 1
TJXOPT_TRIM = 2
TJXOPT_CROP = 4
TJXOPT_GRAY = 8
TJXOPT_NOOUTPUT = 16
TJXOPT_PROGRESSIVE = 32
TJXOPT_COPYNONE = 64

# MCU block width (in pixels) for a given level of chrominance subsampling.
# MCU block sizes:
#  - 8x8 for no subsampling or grayscale
#  - 16x8 for 4:2:2
#  - 8x16 for 4:4:0
#  - 16x16 for 4:2:0
#  - 32x8 for 4:1:1
tjMCUWidth = [8, 16, 16, 8, 8, 32]

# MCU block height (in pixels) for a given level of chrominance subsampling.
# MCU block sizes:
#  - 8x8 for no subsampling or grayscale
#  - 16x8 for 4:2:2
#  - 8x16 for 4:4:0
#  - 16x16 for 4:2:0
#  - 32x8 for 4:1:1
tjMCUHeight = [8, 8, 16, 8, 16, 8]

# miscellaneous flags
# see details in https://github.com/libjpeg-turbo/libjpeg-turbo/blob/master/turbojpeg.h
# note: TJFLAG_NOREALLOC cannot be supported due to reallocation is needed by PyTurboJPEG.
TJFLAG_BOTTOMUP = 2
TJFLAG_FASTUPSAMPLE = 256
TJFLAG_FASTDCT = 2048
TJFLAG_ACCURATEDCT = 4096
TJFLAG_STOPONWARNING = 8192
TJFLAG_PROGRESSIVE = 16384
TJFLAG_LIMITSCANS = 32768

class CroppingRegion(Structure):
    _fields_ = [("x", c_int), ("y", c_int), ("w", c_int), ("h", c_int)]

class TransformStruct(Structure):
    _fields_ = [("r", CroppingRegion), ("op", c_int), ("options", c_int), ("data", c_void_p),
        ("customFilter", c_void_p)]

CUSTOMFILTER = CFUNCTYPE(
    c_int, POINTER(c_short), CroppingRegion, CroppingRegion, c_int, c_int,
    POINTER(TransformStruct))

class TurboJPEG(object):
    """A Python wrapper of libjpeg-turbo for decoding and encoding JPEG image."""
    def __init__(self, lib_path=None):
        turbo_jpeg = cdll.LoadLibrary(
            self.__find_turbojpeg() if lib_path is None else lib_path)
        self.__init_decompress = turbo_jpeg.tjInitDecompress
        self.__init_decompress.restype = c_void_p
        self.__buffer_size = turbo_jpeg.tjBufSize
        self.__buffer_size.argtypes = [c_int, c_int, c_int]
        self.__buffer_size.restype = c_ulong
        self.__init_compress = turbo_jpeg.tjInitCompress
        self.__init_compress.restype = c_void_p
        self.__buffer_size_YUV2 = turbo_jpeg.tjBufSizeYUV2
        self.__buffer_size_YUV2.argtypes = [c_int, c_int, c_int, c_int]
        self.__buffer_size_YUV2.restype = c_ulong
        self.__plane_width = turbo_jpeg.tjPlaneWidth
        self.__plane_width.argtypes = [c_int, c_int, c_int]
        self.__plane_width.restype = c_int
        self.__plane_height = turbo_jpeg.tjPlaneHeight
        self.__plane_height.argtypes = [c_int, c_int, c_int]
        self.__plane_height.restype = c_int
        self.__destroy = turbo_jpeg.tjDestroy
        self.__destroy.argtypes = [c_void_p]
        self.__destroy.restype = c_int
        self.__decompress_header = turbo_jpeg.tjDecompressHeader3
        self.__decompress_header.argtypes = [
            c_void_p, POINTER(c_ubyte), c_ulong, POINTER(c_int),
            POINTER(c_int), POINTER(c_int), POINTER(c_int)]
        self.__decompress_header.restype = c_int
        self.__decompress = turbo_jpeg.tjDecompress2
        self.__decompress.argtypes = [
            c_void_p, POINTER(c_ubyte), c_ulong, POINTER(c_ubyte),
            c_int, c_int, c_int, c_int, c_int]
        self.__decompress.restype = c_int
        self.__decompressToYUV2 = turbo_jpeg.tjDecompressToYUV2
        self.__decompressToYUV2.argtypes = [
            c_void_p, POINTER(c_ubyte), c_ulong, POINTER(c_ubyte),
            c_int, c_int, c_int, c_int]
        self.__decompressToYUV2.restype = c_int
        self.__decompressToYUVPlanes = turbo_jpeg.tjDecompressToYUVPlanes
        self.__decompressToYUVPlanes.argtypes = [
            c_void_p, POINTER(c_ubyte), c_ulong, POINTER(POINTER(c_ubyte)),
            c_int, POINTER(c_int), c_int, c_int]
        self.__decompressToYUVPlanes.restype = c_int
        self.__compress = turbo_jpeg.tjCompress2
        self.__compress.argtypes = [
            c_void_p, POINTER(c_ubyte), c_int, c_int, c_int, c_int,
            POINTER(c_void_p), POINTER(c_ulong), c_int, c_int, c_int]
        self.__compress.restype = c_int
        self.__compressFromYUV = turbo_jpeg.tjCompressFromYUV
        self.__compressFromYUV.argtypes = [
            c_void_p, POINTER(c_ubyte), c_int, c_int, c_int, c_int,
            POINTER(c_void_p), POINTER(c_ulong), c_int, c_int]
        self.__compressFromYUV.restype = c_int
        self.__init_transform = turbo_jpeg.tjInitTransform
        self.__init_transform.restype = c_void_p
        self.__transform = turbo_jpeg.tjTransform
        self.__transform.argtypes = [
            c_void_p, POINTER(c_ubyte), c_ulong, c_int, POINTER(c_void_p),
            POINTER(c_ulong), POINTER(TransformStruct), c_int]
        self.__transform.restype = c_int
        self.__free = turbo_jpeg.tjFree
        self.__free.argtypes = [c_void_p]
        self.__free.restype = None
        self.__get_error_str = turbo_jpeg.tjGetErrorStr
        self.__get_error_str.restype = c_char_p
        # tjGetErrorStr2 is only available in newer libjpeg-turbo
        self.__get_error_str2 = getattr(turbo_jpeg, 'tjGetErrorStr2', None)
        if self.__get_error_str2 is not None:
            self.__get_error_str2.argtypes = [c_void_p]
            self.__get_error_str2.restype = c_char_p
        # tjGetErrorCode is only available in newer libjpeg-turbo
        self.__get_error_code = getattr(turbo_jpeg, 'tjGetErrorCode', None)
        if self.__get_error_code is not None:
            self.__get_error_code.argtypes = [c_void_p]
            self.__get_error_code.restype = c_int
        class ScalingFactor(Structure):
            _fields_ = ('num', c_int), ('denom', c_int)
        get_scaling_factors = turbo_jpeg.tjGetScalingFactors
        get_scaling_factors.argtypes = [POINTER(c_int)]
        get_scaling_factors.restype = POINTER(ScalingFactor)
        num_scaling_factors = c_int()
        scaling_factors = get_scaling_factors(byref(num_scaling_factors))
        self.__scaling_factors = frozenset(
            (scaling_factors[i].num, scaling_factors[i].denom)
            for i in range(num_scaling_factors.value)
        )
        self.compress_handle = None
        self.decompress_handle = None

    def decode_header(self, jpeg_buf):
        """decodes JPEG header and returns image properties as a tuple.
           e.g. (width, height, jpeg_subsample, jpeg_colorspace)
        """
        handle = self.__get_decompress_handle()
        try:
            width = c_int()
            height = c_int()
            jpeg_subsample = c_int()
            jpeg_colorspace = c_int()
            jpeg_array = np.frombuffer(jpeg_buf, dtype=np.uint8)
            src_addr = self.__getaddr(jpeg_array)
            status = self.__decompress_header(
                handle, src_addr, jpeg_array.size, byref(width), byref(height),
                byref(jpeg_subsample), byref(jpeg_colorspace))
            if status != 0:
                self.__report_error(handle)
            return (width.value, height.value, jpeg_subsample.value, jpeg_colorspace.value)
        except Exception as e:
            print(e)
            return None

    def decode(self, jpeg_buf, pixel_format=TJPF_BGR, scaling_factor=None, flags=0):
        """decodes JPEG memory buffer to numpy array."""
        handle = self.__get_decompress_handle()
        try:
            pixel_size = [3, 3, 4, 4, 4, 4, 1, 4, 4, 4, 4, 4]
            jpeg_array = np.frombuffer(jpeg_buf, dtype=np.uint8)
            src_addr = self.__getaddr(jpeg_array)
            scaled_width, scaled_height, _, _ = \
                self.__get_header_and_dimensions(handle, jpeg_array.size, src_addr, scaling_factor)
            img_array = np.empty(
                [scaled_height, scaled_width, pixel_size[pixel_format]],
                dtype=np.uint8)
            dest_addr = self.__getaddr(img_array)
            status = self.__decompress(
                handle, src_addr, jpeg_array.size, dest_addr, scaled_width,
                0, scaled_height, pixel_format, flags)
            if status != 0:
                self.__report_error(handle)
            return img_array
        except Exception as e:
            print(e)
            return None

    def decode_to_yuv(self, jpeg_buf, scaling_factor=None, pad=4, flags=0):
        """decodes JPEG memory buffer to yuv array."""
        handle = self.__get_decompress_handle()
        try:
            jpeg_array = np.frombuffer(jpeg_buf, dtype=np.uint8)
            src_addr = self.__getaddr(jpeg_array)
            scaled_width, scaled_height, jpeg_subsample, _ = \
                self.__get_header_and_dimensions(handle, jpeg_array.size, src_addr, scaling_factor)
            buffer_size = self.__buffer_size_YUV2(scaled_width, pad, scaled_height, jpeg_subsample)
            buffer_array = np.empty(buffer_size, dtype=np.uint8)
            dest_addr = self.__getaddr(buffer_array)
            status = self.__decompressToYUV2(
                handle, src_addr, jpeg_array.size, dest_addr, scaled_width,
                pad, scaled_height, flags)
            if status != 0:
                self.__report_error(handle)
            plane_sizes = list()
            plane_sizes.append((scaled_height, scaled_width))
            if jpeg_subsample != TJSAMP_GRAY:
                for i in range(1, 3):
                    plane_sizes.append((
                        self.__plane_height(i, scaled_height, jpeg_subsample),
                        self.__plane_width(i, scaled_width, jpeg_subsample)))
            return buffer_array, plane_sizes
        except Exception as e:
            print(e)
            return None

    def decode_to_yuv_planes(self, jpeg_buf, scaling_factor=None, strides=(0, 0, 0), flags=0):
        """decodes JPEG memory buffer to yuv planes."""
        handle = self.__get_decompress_handle()
        try:
            jpeg_array = np.frombuffer(jpeg_buf, dtype=np.uint8)
            src_addr = self.__getaddr(jpeg_array)
            scaled_width, scaled_height, jpeg_subsample, _ = \
                self.__get_header_and_dimensions(handle, jpeg_array.size, src_addr, scaling_factor)
            num_planes = 3
            if jpeg_subsample == TJSAMP_GRAY:
                num_planes = 1
            strides_addr = (c_int * num_planes)()
            dest_addr = (POINTER(c_ubyte) * num_planes)()
            planes = list()
            for i in range(num_planes):
                if strides[i] == 0:
                    strides_addr[i] = self.__plane_width(i, scaled_width, jpeg_subsample)
                else:
                    strides_addr[i] = strides[i]
                planes.append(np.empty(
                    (self.__plane_height(i, scaled_height, jpeg_subsample), strides_addr[i]), dtype=np.uint8))
                dest_addr[i] = self.__getaddr(planes[i])
            status = self.__decompressToYUVPlanes(
                handle, src_addr, jpeg_array.size, dest_addr, scaled_width, strides_addr, scaled_height, flags)
            if status != 0:
                self.__report_error(handle)
            return planes
        except Exception as e:
            print(e)
            return None

    def encode(self, img_array, quality=80, pixel_format=TJPF_BGR, jpeg_subsample=TJSAMP_420, flags=0):
        """encodes numpy array to JPEG memory buffer."""
        handle = self.__get_compress_handle()
        jpeg_buf = c_void_p()
        jpeg_size = c_ulong()
        height, width, _ = img_array.shape
        src_addr = self.__getaddr(img_array)
        status = self.__compress(
            handle, src_addr, width, img_array.strides[0], height, pixel_format,
            byref(jpeg_buf), byref(jpeg_size), jpeg_subsample, quality, flags)
        if status != 0:
            self.__report_error(handle)
        dest_buf = create_string_buffer(jpeg_size.value)
        memmove(dest_buf, jpeg_buf.value, jpeg_size.value)
        self.__free(jpeg_buf)
        return dest_buf.raw

    def encode_YUV(self, img_array, quality=80, jpeg_subsample=TJSAMP_420, flags=0):
        """encodes numpy array to JPEG memory buffer."""
        handle = self.__get_compress_handle()

        jpeg_buf = c_void_p()
        jpeg_size = c_ulong()
        height, width = img_array.shape
        height = int(height * 2 / 3)
        src_addr = self.__getaddr(img_array)
        status = self.__compressFromYUV(
            handle, src_addr, width, 4, height, jpeg_subsample, byref(jpeg_buf),
            byref(jpeg_size), quality, flags)
        if status != 0:
            self.__report_error(handle)
        dest_buf = create_string_buffer(jpeg_size.value)
        memmove(dest_buf, jpeg_buf.value, jpeg_size.value)
        self.__free(jpeg_buf)
        return dest_buf.raw

    def scale_with_quality(self, jpeg_buf, scaling_factor=None, quality=85, flags=0):
        """decompresstoYUV with scale factor, recompresstoYUV with quality factor"""
        handle = self.__get_decompress_handle()
        try:
            jpeg_array = np.frombuffer(jpeg_buf, dtype=np.uint8)
            src_addr = self.__getaddr(jpeg_array)
            scaled_width, scaled_height, jpeg_subsample, _ = self.__get_header_and_dimensions(
                handle, jpeg_array.size, src_addr, scaling_factor)
            buffer_YUV_size = self.__buffer_size_YUV2(
                scaled_height, 4, scaled_width, jpeg_subsample)
            img_array = np.empty([buffer_YUV_size])
            dest_addr = self.__getaddr(img_array)
            status = self.__decompressToYUV2(
                handle, src_addr, jpeg_array.size, dest_addr, scaled_width, 4, scaled_height, flags)
            if status != 0:
                self.__report_error(handle)
                return
            self.__destroy(handle)
            handle = self.__init_compress()
            jpeg_buf = c_void_p()
            jpeg_size = c_ulong()
            status = self.__compressFromYUV(
                handle, dest_addr, scaled_width, 4, scaled_height, jpeg_subsample, byref(jpeg_buf),
                byref(jpeg_size), quality, flags)
            if status != 0:
                self.__report_error(handle)
            dest_buf = create_string_buffer(jpeg_size.value)
            memmove(dest_buf, jpeg_buf.value, jpeg_size.value)
            self.__free(jpeg_buf)
            return dest_buf.raw
        except Exception as e:
            print(e)
            return None

    def crop(self, jpeg_buf, x, y, w, h, preserve=False, gray=False):
        """losslessly crop a jpeg image with optional grayscale"""
        handle = self.__init_transform()
        try:
            jpeg_array = np.frombuffer(jpeg_buf, dtype=np.uint8)
            src_addr = self.__getaddr(jpeg_array)
            width = c_int()
            height = c_int()
            jpeg_colorspace = c_int()
            jpeg_subsample = c_int()
            status = self.__decompress_header(
                handle, src_addr, jpeg_array.size, byref(width), byref(height),
                byref(jpeg_subsample), byref(jpeg_colorspace))
            if status != 0:
                self.__report_error(handle)
            x, w = self.__axis_to_image_boundaries(
                x, w, width.value, preserve, tjMCUWidth[jpeg_subsample.value])
            y, h = self.__axis_to_image_boundaries(
                y, h, height.value, preserve, tjMCUHeight[jpeg_subsample.value])
            dest_array = c_void_p()
            dest_size = c_ulong()
            region = CroppingRegion(x, y, w, h)
            crop_transform = TransformStruct(region, TJXOP_NONE,
                TJXOPT_CROP | (gray and TJXOPT_GRAY))
            status = self.__transform(
                handle, src_addr, jpeg_array.size, 1, byref(dest_array), byref(dest_size),
                byref(crop_transform), 0)
            dest_buf = create_string_buffer(dest_size.value)
            memmove(dest_buf, dest_array.value, dest_size.value)
            self.__free(dest_array)
            if status != 0:
                self.__report_error(handle)
            return dest_buf.raw
        finally:
            self.__destroy(handle)

    def __get_header_and_dimensions(self, handle, jpeg_array_size, src_addr, scaling_factor):
        """returns scaled image dimensions and header data"""
        if scaling_factor is not None and \
            scaling_factor not in self.__scaling_factors:
            raise ValueError('supported scaling factors are ' +
                str(self.__scaling_factors))
        width = c_int()
        height = c_int()
        jpeg_colorspace = c_int()
        jpeg_subsample = c_int()
        status = self.__decompress_header(
            handle, src_addr, jpeg_array_size, byref(width), byref(height),
            byref(jpeg_subsample), byref(jpeg_colorspace))
        if status != 0:
            self.__report_error(handle)
        scaled_width = width.value
        scaled_height = height.value
        if scaling_factor is not None:
            def get_scaled_value(dim, num, denom):
                return (dim * num + denom - 1) // denom
            scaled_width = get_scaled_value(
                scaled_width, scaling_factor[0], scaling_factor[1])
            scaled_height = get_scaled_value(
                scaled_height, scaling_factor[0], scaling_factor[1])
        return scaled_width, scaled_height, jpeg_subsample, jpeg_colorspace

    def __axis_to_image_boundaries(self, a, b, img_boundary, preserve, mcuBlock):
        img_b = img_boundary - (img_boundary % mcuBlock)
        delta_a = a % mcuBlock
        if a > img_b:
            a = img_b
        else:
            a = a - delta_a
        if not preserve:
            b = b + delta_a
        if (a + b) > img_b:
            b = img_b - a
        return a, b

    def __report_error(self, handle):
        """reports error while error occurred"""
        if self.__get_error_code is not None:
            # using new error handling logic if possible
            if self.__get_error_code(handle) == TJERR_WARNING:
                warnings.warn(self.__get_error_string(handle))
                return
        # fatal error occurred
        raise IOError(self.__get_error_string(handle))

    def __get_error_string(self, handle):
        """returns error string"""
        if self.__get_error_str2 is not None:
            # using new interface if possible
            return self.__get_error_str2(handle).decode()
        # fallback to old interface
        return self.__get_error_str().decode()

    def __find_turbojpeg(self):
        """returns default turbojpeg library path if possible"""
        lib_path = find_library('turbojpeg')
        if lib_path is not None:
            return lib_path
        for lib_path in DEFAULT_LIB_PATHS[platform.system()]:
            if os.path.exists(lib_path):
                return lib_path
        if platform.system() == 'Linux' and 'LD_LIBRARY_PATH' in os.environ:
            ld_library_path = os.environ['LD_LIBRARY_PATH']
            for path in ld_library_path.split(':'):
                lib_path = os.path.join(path, 'libturbojpeg.so.0')
                if os.path.exists(lib_path):
                    return lib_path
        raise RuntimeError(
            'Unable to locate turbojpeg library automatically. '
            'You may specify the turbojpeg library path manually.\n'
            'e.g. jpeg = TurboJPEG(lib_path)')

    def __getaddr(self, nda):
        """returns the memory address for a given ndarray"""
        return cast(nda.__array_interface__['data'][0], POINTER(c_ubyte))

    def __get_compress_handle(self):
        if self.compress_handle is None:
            self.compress_handle = self.__init_compress()
        return self.compress_handle

    def __get_decompress_handle(self):
        if self.decompress_handle is None:
            self.decompress_handle = self.__init_decompress()
        return self.decompress_handle

    @property
    def scaling_factors(self):
        return self.__scaling_factors