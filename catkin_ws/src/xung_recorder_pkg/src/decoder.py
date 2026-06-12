"""
***************************************
*                                     *
* Copyright (c) 2023                  *
*                                     *
* Technische Hochschule Aschaffenburg *
* Manuel Hetzel                       *
* KAV Labor                           *
* 63743 Aschaffenburg                 *
* Wuerzburger Str. 45                 *
*                                     *
***************************************

/**
 * @file decoder.py for Xung data recording
 * @author Manuel Hetzel
 * @date 4.10.2023
 * @version 1.0
 *
 * @brief Provides video decoding helpers used by the camera recorder.
 *
*/
"""


# #
# # Copyright 2019 NVIDIA Corporation
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #    http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.
# #

# # Starting from Python 3.8 DLL search policy has changed.
# # We need to add path to CUDA DLLs explicitly.
# import sys
# import os

# if os.name == 'nt':
#     # Add CUDA_PATH env variable
#     cuda_path = os.environ["CUDA_PATH"]
#     if cuda_path:
#         os.add_dll_directory(cuda_path)
#     else:
#         print("CUDA_PATH environment variable is not set.", file = sys.stderr)
#         print("Can't set CUDA DLLs search path.", file = sys.stderr)
#         exit(1)

#     # Add PATH as well for minor CUDA releases
#     sys_path = os.environ["PATH"]
#     if sys_path:
#         paths = sys_path.split(';')
#         for path in paths:
#             if os.path.isdir(path):
#                 os.add_dll_directory(path)
#     else:
#         print("PATH environment variable is not set.", file = sys.stderr)
#         exit(1)

# import pycuda.driver as cuda
# import PyNvCodec as nvc
# import numpy as np
# import cv2

# def decode(gpuID, encFilePath, decFilePath):
#     cuda.init()
#     cuda_ctx = cuda.Device(gpuID).retain_primary_context()
#     cuda_ctx.push()
#     cuda_str = cuda.Stream()
#     cuda_ctx.pop()

#     decFile = open(decFilePath, "wb")

#     nvDmx = nvc.PyFFmpegDemuxer(encFilePath)
#     nvDec = nvc.PyNvDecoder(nvDmx.Width(), nvDmx.Height(), nvDmx.Format(), nvDmx.Codec(), cuda_ctx.handle, cuda_str.handle)
#     nvCvt = nvc.PySurfaceConverter(nvDmx.Width(), nvDmx.Height(), nvDmx.Format(), nvc.PixelFormat.YUV420, cuda_ctx.handle, cuda_str.handle)
#     nvDwn = nvc.PySurfaceDownloader(nvDmx.Width(), nvDmx.Height(), nvCvt.Format(), cuda_ctx.handle, cuda_str.handle)

#     packet = np.ndarray(shape=(0), dtype=np.uint8)
#     frameSize = int(nvDmx.Width() * nvDmx.Height() * 3 / 2)
#     rawFrame = np.ndarray(shape=(frameSize), dtype=np.uint8)
#     pdata_in, pdata_out = nvc.PacketData(), nvc.PacketData()

#     # Determine colorspace conversion parameters.
#     # Some video streams don't specify these parameters so default values
#     # are most widespread bt601 and mpeg.
#     cspace, crange = nvDmx.ColorSpace(), nvDmx.ColorRange()
#     if nvc.ColorSpace.UNSPEC == cspace:
#         cspace = nvc.ColorSpace.BT_709
#     if nvc.ColorRange.UDEF == crange:
#         crange = nvc.ColorRange.MPEG
#     cc_ctx = nvc.ColorspaceConversionContext(cspace, crange)
#     print('Color space: ', str(cspace))
#     print('Color range: ', str(crange))

#     while True:
#         # Demuxer has sync design, it returns packet every time it's called.
#         # If demuxer can't return packet it usually means EOF.
#         if not nvDmx.DemuxSinglePacket(packet):
#             break

#         # Get last packet data to obtain frame timestamp
#         nvDmx.LastPacketData(pdata_in)

#         # Decoder is async by design.
#         # As it consumes packets from demuxer one at a time it may not return
#         # decoded surface every time the decoding function is called.
#         surface_nv12 = nvDec.DecodeSurfaceFromPacket(pdata_in, packet, pdata_out)
#         if not surface_nv12.Empty():
#             surface_yuv420 = nvCvt.Execute(surface_nv12, cc_ctx)
#             if surface_yuv420.Empty():
#                 break
#             if not nvDwn.DownloadSingleSurface(surface_yuv420, rawFrame):
#                 break
            
#             a = rawFrame.reshape(int(2160*1.5), 4096)
#             bgr = cv2.cvtColor(a, cv2.COLOR_YUV2BGR_I420)
#             cv2.imwrite('/workspace/data/test.png', bgr)

#             bits = bytearray(rawFrame)
#             decFile.write(bits)

#     # Now we flush decoder to emtpy decoded frames queue.
#     while True:
#         surface_nv12 = nvDec.FlushSingleSurface()
#         if surface_nv12.Empty():
#             break
#         surface_yuv420 = nvCvt.Execute(surface_nv12, cc_ctx)
#         if surface_yuv420.Empty():
#             break
#         if not nvDwn.DownloadSingleSurface(surface_yuv420, rawFrame):
#             break
#         bits = bytearray(rawFrame)
#         decFile.write(bits)

# if __name__ == "__main__":

#     print("This sample decodes input video to raw YUV420 file on given GPU.")
#     print("Usage: SampleDecode.py $gpu_id $input_file $output_file.")

#     gpuID = 0
#     encFilePath = '/workspace/data/20221025_102656/raw/camera_uhk1.mkv'
#     decFilePath = '/workspace/data/20221025_102656/raw/result.bin'
    
#     nvDec = nvc.PyNvDecoder(encFilePath, gpuID)
    
#     to_rgb = nvc.PySurfaceConverter(nvDec.Width(), nvDec.Height(), nvc.PixelFormat.NV12, nvc.PixelFormat.RGB, gpuID)
    
#     while True:
#         # Obtain NV12 decoded surface from decoder;
#         rawSurface = nvDec.DecodeSingleSurface()
#         if (rawSurface.Empty()):
#             break
        


#         # Convert to RGB interleaved;
#         rgb_byte = to_rgb.Execute(nv12_smaller)




    
    

# a = 5
#     #decode(gpuID, encFilePath, decFilePath)



#
# Copyright 2019 NVIDIA Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# Starting from Python 3.8 DLL search policy has changed.
# We need to add path to CUDA DLLs explicitly.
import sys
import os

if os.name == 'nt':
    # Add CUDA_PATH env variable
    cuda_path = os.environ["CUDA_PATH"]
    if cuda_path:
        os.add_dll_directory(cuda_path)
    else:
        print("CUDA_PATH environment variable is not set.", file = sys.stderr)
        print("Can't set CUDA DLLs search path.", file = sys.stderr)
        exit(1)

    # Add PATH as well for minor CUDA releases
    sys_path = os.environ["PATH"]
    if sys_path:
        paths = sys_path.split(';')
        for path in paths:
            if os.path.isdir(path):
                os.add_dll_directory(path)
    else:
        print("PATH environment variable is not set.", file = sys.stderr)
        exit(1)

import pycuda.driver as cuda
import PyNvCodec as nvc
import numpy as np
import cv2

def decode(gpuID, encFilePath, decFilePath):
    """Decode the configured operation.
    
    Args:
        gpuID (object): Input gpuID value.
        encFilePath (object): Input encFilePath value.
        decFilePath (object): Input decFilePath value.
    
    Returns:
        None
    """
    cuda.init()
    cuda_ctx = cuda.Device(gpuID).retain_primary_context()
    cuda_ctx.push()
    cuda_str = cuda.Stream()
    cuda_ctx.pop()

    decFile = open(decFilePath, "wb")

    nvDmx = nvc.PyFFmpegDemuxer(encFilePath)
    nvDec = nvc.PyNvDecoder(nvDmx.Width(), nvDmx.Height(), nvDmx.Format(), nvDmx.Codec(), cuda_ctx.handle, cuda_str.handle)
    nvCvt = nvc.PySurfaceConverter(nvDmx.Width(), nvDmx.Height(), nvDmx.Format(), nvc.PixelFormat.Y, cuda_ctx.handle, cuda_str.handle)
    nvDwn = nvc.PySurfaceDownloader(nvDmx.Width(), nvDmx.Height(), nvCvt.Format(), cuda_ctx.handle, cuda_str.handle)

    packet = np.ndarray(shape=(0), dtype=np.uint8)
    frameSize = int(nvDmx.Width() * nvDmx.Height() * 3 / 2)
    rawFrame = np.ndarray(shape=(frameSize), dtype=np.uint8)
    pdata_in, pdata_out = nvc.PacketData(), nvc.PacketData()

    # Determine colorspace conversion parameters.
    # Some video streams don't specify these parameters so default values
    # are most widespread bt601 and mpeg.
    cspace, crange = nvDmx.ColorSpace(), nvDmx.ColorRange()
    if nvc.ColorSpace.UNSPEC == cspace:
        cspace = nvc.ColorSpace.BT_709
    if nvc.ColorRange.UDEF == crange:
        crange = nvc.ColorRange.JPEG
    cc_ctx = nvc.ColorspaceConversionContext(cspace, crange)
    print('Color space: ', str(cspace))
    print('Color range: ', str(crange))
    
    cnt = 0

    while True:
        # Demuxer has sync design, it returns packet every time it's called.
        # If demuxer can't return packet it usually means EOF.
        if not nvDmx.DemuxSinglePacket(packet):
            break

        # Get last packet data to obtain frame timestamp
        nvDmx.LastPacketData(pdata_in)

        # Decoder is async by design.
        # As it consumes packets from demuxer one at a time it may not return
        # decoded surface every time the decoding function is called.
        surface_nv12 = nvDec.DecodeSurfaceFromPacket(pdata_in, packet, pdata_out)
        if not surface_nv12.Empty():
            surface_yuv420 = nvCvt.Execute(surface_nv12, cc_ctx)
            if surface_yuv420.Empty():
                break
            if not nvDwn.DownloadSingleSurface(surface_yuv420, rawFrame):
                break

            a = rawFrame.reshape(2160, 4096)
            a = cv2.demosaicing(a, cv2.COLOR_BAYER_RG2RGB)
            cv2.imwrite('/workspace/data/test_' + str(cnt).zfill(7) + '.bmp', a)
            cnt += 1

            bits = bytearray(rawFrame)
            decFile.write(bits)

    # Now we flush decoder to emtpy decoded frames queue.
    while True:
        surface_nv12 = nvDec.FlushSingleSurface()
        if surface_nv12.Empty():
            break
        surface_yuv420 = nvCvt.Execute(surface_nv12, cc_ctx)
        if surface_yuv420.Empty():
            break
        if not nvDwn.DownloadSingleSurface(surface_yuv420, rawFrame):
            break
        
        a = rawFrame.reshape(2160, 4096)
        a = cv2.demosaicing(a, cv2.COLOR_BAYER_RG2RGB)
        cv2.imwrite('/workspace/data/test_' + str(cnt).zfill(7) + '.bmp', a)
        cnt += 1
        
        bits = bytearray(rawFrame)
        decFile.write(bits)

if __name__ == "__main__":

    print("This sample decodes input video to raw YUV420 file on given GPU.")
    print("Usage: SampleDecode.py $gpu_id $input_file $output_file.")

    gpuID = 0
    encFilePath = '/workspace/data/20230224_132223/raw/camera_uhk6.bin'
    decFilePath = '/workspace/data/20230224_132223/raw/result.bin'

    decode(gpuID, encFilePath, decFilePath)
