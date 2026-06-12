#!/usr/bin/env python3

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
 * @file camera.py for Xung data recording
 * @author Manuel Hetzel
 * @date 4.10.2023
 * @version 1.0
 *
 * @brief Records synchronized ROS camera streams to encoded video and metadata.
 *
*/
"""

import PyNvCodec as nvc
import rospy
import json
import os
import numpy as np
import time
import csv
import av

from termcolor import colored
from sensor_msgs.msg import Image


ID_LEADING_ZEROS = 7
NAME_LENGTH = 24


# class declaration
class XungCAMRecorder(object):
    """ros xung camera recorder node

    Args:
        object ([class]): []
    """
    
    # init class instance
    def __init__(self, name, gpuid, dest_path, recording_signal=False, shutdown_signal=False, feedback_dict={}):
        
        # Encoder params
        """Initialize the XungCAMRecorder instance.
        
        Args:
            name (object): Input name value.
            gpuid (object): Input gpuid value.
            dest_path (object): Input dest path value.
            recording_signal (object): Input recording signal value.
            shutdown_signal (object): Input shutdown signal value.
            feedback_dict (object): Input feedback dict value.
        
        Returns:
            None
        """
        self.image_w = 4096
        self.image_h = 2160
        self.codec = 'hevc'
        self.profile = 'high'
        self.tune = 'high_quality'
        self.preset = 'P5'
        self.bitrate = '20M'
        
        # camera recorder params
        self.callback_cnt = 0
        self.enc_cnt = 0
        self.time = 0
        self.previous_ts = 0
        self.name = name
        self.dest_path = dest_path
        self.recording_signal = recording_signal
        self.shutdown_signal = shutdown_signal
        self.feedback_dict = feedback_dict
        self.ref_timestamp = 0
        self.camera_sync = {}
        self.first_camera = True
        self.first_sucess = False
        
        # setup ros node
        rospy.init_node("xung_cam_" + self.name + "_recorder_node", anonymous=False)
        
        # Set subscriber to camera ros node image topic
        self.camera_subscriber = rospy.Subscriber(self.name + '_raw', Image, self.camera_callback)
        
        # Storage path   
        self.storage_path = os.path.join(self.dest_path, 'camera', 'raw')
        os.makedirs(self.storage_path, exist_ok=True)
        
        # Create gpu nvenc encoder       
        self.nv_enc = nvc.PyNvEncoder({ 'preset': self.preset,
                                'tuning_info': self.tune, 
                                'codec': self.codec, 
                                'profile': self.profile, 
                                's': f'{self.image_w}x{self.image_h}',
                                'bitrate': self.bitrate ,
                                }, gpuid)
        
        self.enc_frame = np.ndarray(shape=(0), dtype=np.uint8)
        
        # Create encoded data file
        self.enc_file = open(os.path.join(self.storage_path, 'camera_' + self.name + '.bin'), "wb")

        # Image ts and metadata storage
        self.timestamp_file = open(os.path.join(self.storage_path, 'camera_' + self.name + '.csv'), 'w', encoding='UTF8')
        self.csv_ts_writer = csv.writer(self.timestamp_file)
        
        # Write image metadata header
        self.csv_ts_writer.writerow(['topic_name', 'width', 'height', 'type'])
        self.csv_ts_writer.writerow(['/camera_' + self.name , self.image_w, self.image_h, 'bayer8rg'])
        self.csv_ts_writer.writerow(['image_id | timestamp | delta | callbacks'])
        
        
    def camera_callback(self, msg):

        # Get augmented cloud data
        """Build callback.
        
        Args:
            msg (object): Input msg value.
        
        Returns:
            None
        """
        if self.recording_signal.value:

            # Monitor processing time
            start_time = time.time()
            
            # Get data from subscriber
            ts = (msg.header.stamp.secs << 32 | msg.header.stamp.nsecs)
            cam_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width),
            
            # Encode image
            success = self.nv_enc.EncodeSingleFrame(cam_image, self.enc_frame, sync=False)
            
            # Write encoded image data to bytestream
            if(success):
                
                # Get encoded image as raw binary stream
                enc_byte_rray = bytearray(self.enc_frame)
                
                # Write encoded data to file
                self.enc_file.write(enc_byte_rray)
                
                # Write ts and metadata to file
                if self.first_camera:
                
                    gps_delta = 0
                    self.first_camera = False
                
                else:
                
                    gps_delta = (ts - self.previous_ts) 
                
                # Recording and meta data
                self.camera_sync.update({ts: {"id": str(self.enc_cnt).zfill(ID_LEADING_ZEROS), 
                                            "gps_delta": gps_delta,
                                            "callbacks": self.callback_cnt,
                                            }}) 
                
                self.csv_ts_writer.writerow([str(self.enc_cnt).zfill(ID_LEADING_ZEROS) + ' | ' + str(ts) + ' | ' + str(gps_delta)+ ' | ' + str(self.callback_cnt)])
                self.enc_cnt += 1
                self.previous_ts = ts
                
            # Encoder needs some frame before first success
            if not self.first_sucess and success:
                
                self.first_sucess = True
                
            # Start counting when encoder gives first sucess
            if self.first_sucess:
                
                # Increment id counter and monitor processing time
                self.callback_cnt += 1
                self.time = round((time.time() - start_time) * 1000)
        
        
    def start(self):     
        
        # Callbacks are doing the data saving jobs, this function waits for stop signal
        """Start the configured operation.
        
        Returns:
            None
        """
        while not rospy.is_shutdown():
            
            # Stop/shutdown signal
            if self.shutdown_signal.value:
                
                break
            
            # Send feedback
            self.feedback_dict[self.name] = [self.enc_cnt, self.time]
            
            # Else sleep for 5 ms and retest
            rospy.sleep(0.005)
            
        self.stop()  
    
            
    def stop(self):
        
        # Unregister from all subscriped topics
        """Stop the configured operation.
        
        Returns:
            None
        """
        self.camera_subscriber.unregister()
        
        # Sleep for 250 ms to finish all callbacks
        rospy.sleep(0.25)
        
        # Save sync data     
        json_cam = os.path.join(self.storage_path, 'camera_' + self.name + '.json')
        
        with open(json_cam, 'w') as fp:
            json.dump(self.camera_sync, fp)
        
        # Close the open files
        self.timestamp_file.close()        
        self.enc_file.close()
        
        # Feedback
        print(colored(f" {self.name} closed - Cb: {self.callback_cnt}, Enc: {self.enc_cnt}", 'green'))
        
        return