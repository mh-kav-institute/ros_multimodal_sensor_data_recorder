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
 * @file alb_edge.py for Xung data recording
 * @author Manuel Hetzel
 * @date 4.10.2023
 * @version 1.0
 *
 * @brief Records edge-lidar point clouds, reflectivity images, and synchronization metadata.
 *
*/
"""

import rospy
import os
import numpy as np
import cv2
import time
import json
import copy


from termcolor import colored
from utils import destagger, os1_pixel_shift_by_row
from std_msgs.msg import UInt64
from xung_recorder_pkg.msg import AugmentedCloud


ID_LEADING_ZEROS = 7
TS_LOWER_DIGITS = 9
TS_DIGITS = 13
TS_DIGITS_OUT = 16
NAME_LENGTH = 24
MAX_PROCESSING_TIME = 40


# class declaration
class XungAlbEdgeRecorder(object):
    """ros xung alb recorder node
    
    Args:
        object ([class]): []
    """
    
    # init class instance
    def __init__(self, name, dest_path, recording_signal=False, shutdown_signal=False, feedback_dict={}):
        
        # Recording params
        """Initialize the XungAlbEdgeRecorder instance.
        
        Args:
            name (object): Input name value.
            dest_path (object): Input dest path value.
            recording_signal (object): Input recording signal value.
            shutdown_signal (object): Input shutdown signal value.
            feedback_dict (object): Input feedback dict value.
        
        Returns:
            None
        """
        self.pc_cnt = 0
        self.pc_time = 0
        self.name = name
        self.topics = ["augmented_cloud"]
        self.dest_path = dest_path
        self.recording_signal = recording_signal
        self.shutdown_signal = shutdown_signal
        self.feedback_dict = feedback_dict
        self.subscriber_list = []
        self.track_data = {}
        self.augmented_cloud_data = {}
        self.ref_timestamp = 0
        self.previous_alb_cloud_ts = 0
        self.previous_gps_ts = 0
        self.first_cloud = True
        
        # setup ros node
        rospy.init_node("xung_alb_" + name + "_recorder_node", anonymous=False)
        
        self.subscriber_list.append(rospy.Subscriber('xung_timestamp_node', UInt64, self.ts_callback))
        
        # setup ros subscriber
        for topic in self.topics:
            
            if topic == 'augmented_cloud':
                
                self.subscriber_list.append(rospy.Subscriber('alb_' + name + '_augmented_cloud', AugmentedCloud, self.augmented_cloud_callback))
                
        # Data storage structure
        self.storage_path = os.path.join(self.dest_path, 'alb', self.name)
        self.xyz_path = os.path.join(self.storage_path, 'augmented_cloud')
        self.reflectivity_path = os.path.join(self.storage_path, 'reflectivity')
        self.lidar_image_path = os.path.join(self.storage_path, 'lidar_image')
        os.makedirs(self.storage_path, exist_ok=True)
        os.makedirs(self.xyz_path, exist_ok=True)
        os.makedirs(self.reflectivity_path, exist_ok=True)
        os.makedirs(self.lidar_image_path, exist_ok=True)
        
        
    def augmented_cloud_callback(self, msg):
        
        # Get augmented cloud data
        """Record cloud callback.
        
        Args:
            msg (object): Input msg value.
        
        Returns:
            None
        """
        if self.recording_signal.value:
            
            # Get current gps timestamp
            gps_ts = copy.deepcopy(self.ref_timestamp)
            
            # Get current alb timestamp
            alb_ts = [str(msg.header.stamp.secs), str(msg.header.stamp.nsecs)]
            alb_ts = self.convert_alb_timestamp(ts=alb_ts)
            
            # Monitor processing time
            start_time = time.time()
            
            # Get timestamp and meta data
            data_rows = msg.number_of_layers
            data_cols = int(msg.number_of_points / data_rows)
            number_of_points = msg.number_of_points
            
            # Get x,y,z coordinates of point cloud data and stack into 3 channel array
            xyz_pc_data = np.asarray(msg.cartesian_coordinates, dtype=np.float32).reshape(-1,3)
            
            # Get reflectivities of point cloud data
            reflectivity_data = np.frombuffer(np.asarray(msg.reflectivities), dtype=np.uint8).reshape(data_cols, data_rows).T
            
            # Destagger xyz point cloud data
            xyz_pc_data = destagger(
            fields=xyz_pc_data,
            pixels_per_column=data_rows,
            columns_per_frame=data_cols,
            pixel_shift_by_row=os1_pixel_shift_by_row)
            
            # Destagger reflectivity data
            reflectivity_data = destagger(
            fields=reflectivity_data,
            pixels_per_column=data_rows,
            columns_per_frame=data_cols,
            pixel_shift_by_row=os1_pixel_shift_by_row)
            
            # Create lidar as image representation
            depth_data = np.linalg.norm(xyz_pc_data, axis=-1)
            depth_data = depth_data.reshape(data_cols, data_rows).T
            lidar_img_data = np.stack([255 * depth_data / (2 ** 16), 255 * np.ones_like(reflectivity_data),reflectivity_data], axis=-1).astype(np.uint8)
            
            # hsv to color rgb
            lidar_img_data = cv2.cvtColor(lidar_img_data, cv2.COLOR_HSV2BGR)
            
            # brighten image for human interpretability
            lidar_img_data = cv2.cvtColor(lidar_img_data, cv2.COLOR_BGR2YUV)
            
            # equalize the histogram of the Y channel
            lidar_img_data[:, :, 0] = cv2.equalizeHist(lidar_img_data[:, :, 0])
            
            # convert the YUV image back to RGB format
            lidar_img_data = cv2.cvtColor(lidar_img_data, cv2.COLOR_YUV2RGB)
            
            # Save data to files, check name lenth
            #data_name = (str(self.pc_cnt).zfill(ID_LEADING_ZEROS) + '_' + str(gps_ts)).ljust(NAME_LENGTH, '0')
            data_name = (str(gps_ts)).ljust(NAME_LENGTH, '0')
            
            # Point cloud coordinates
            with open(os.path.join(self.xyz_path, data_name), 'wb') as file:
                
                xyzrid_data = np.stack([xyz_pc_data[...,0],xyz_pc_data[...,1],xyz_pc_data[...,2], reflectivity_data.reshape(number_of_points)], axis=-1)
                np.save(file, xyzrid_data)
                
            # Lidar as image and Reflectivity values
            cv2.imwrite(os.path.join(self.lidar_image_path, data_name + '.bmp'), lidar_img_data)
            cv2.imwrite(os.path.join(self.reflectivity_path, data_name + '.bmp'), reflectivity_data)
            
            # Increment id counter and Monitor processing time
            if self.first_cloud:
                
                alb_step = 0
                gps_step = 0
                self.first_cloud = False
                
            # Determine steps
            else:
                
                alb_step = ((alb_ts - self.previous_alb_cloud_ts) / 1000)
                gps_step = ((gps_ts - self.previous_gps_ts) / 1000)
                
            # Recording, meta and synchro data
            self.augmented_cloud_data.update({gps_ts: {"id": str(self.pc_cnt).zfill(ID_LEADING_ZEROS),
                                                        "gps_ts": gps_ts,
                                                        "alb_ts": alb_ts, 
                                                        "gps_step": gps_step, 
                                                        "alb_step": alb_step,
                                                        "num_rows": data_rows,
                                                        "num_cols": data_cols,
                                                        "num_points": number_of_points
                                                        }})
            
            # Increment counter and update times
            self.previous_alb_cloud_ts = alb_ts
            self.previous_gps_ts = gps_ts
            self.pc_cnt += 1
            self.pc_time = round((time.time() - start_time) * 1000)
            
            
    def ts_callback(self, msg):
        
        # Get time reference data
        """Store callback.
        
        Args:
            msg (object): Input msg value.
        
        Returns:
            None
        """
        self.ref_timestamp = msg.data
    
    
    # ALB timestamp is splitted into two parts, lower part can variate in length, this needs to be fixed to reconstruct timestamp as single uint64    
    def convert_alb_timestamp(self, ts):
        
        """Convert alb timestamp.
        
        Args:
            ts (object): Input ts value.
        
        Returns:
            object: Result produced by the operation.
        """
        upper = ts[0]
        lower = ts[1]
        
        diff_lower = len(lower) - TS_LOWER_DIGITS
        
        # check length of lower part, fix it if its not correct
        if diff_lower < 0:
            
            lower = lower.zfill(TS_LOWER_DIGITS)
            
        # combine both parts and check length
        full = upper + lower
        diff = len(full) - TS_DIGITS
        
        # fix overall length if its to long (16 digits include µs, 19 digits include ns)
        if diff > 0:
            
            res = full[:-diff]
            
        elif diff < 0:
            
            res = full[::-1].zfill(TS_DIGITS)[::-1]
        
        else:
            
            res = full
            
        # We round it to 13 digits, but output should be 16 digits, so add 0 zeros    
        return int(str(res)[::-1].zfill(TS_DIGITS_OUT)[::-1])
        
        
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
            self.feedback_dict[self.name] = [self.pc_cnt, self.pc_time]
            
            # Else sleep for 5 ms and retest
            rospy.sleep(0.005)
            
        self.stop()  
        
        
    def stop(self):
        
        # Unregister from all subscriped topics
        """Stop the configured operation.
        
        Returns:
            None
        """
        for subs in self.subscriber_list:
            
            subs.unregister()
            
        # Sleep for 250 ms to finish all callbacks
        rospy.sleep(0.25)
            
        # Save sync and meta data
        # Augmented cloud
        if self.augmented_cloud_data:
            
            json_file = os.path.join(self.storage_path, self.name + '_augmented_cloud.json')
            
            with open(json_file, 'w') as fp:
                json.dump(self.augmented_cloud_data, fp)
                
        # Feedback
        print(colored(f" {self.name} closed - PC: {self.pc_cnt}", 'green'))
        
        return