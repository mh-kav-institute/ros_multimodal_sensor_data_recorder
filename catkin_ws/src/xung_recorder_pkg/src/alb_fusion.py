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
 * @file alb_fusion.py for Xung data recording
 * @author Manuel Hetzel
 * @date 4.10.2023
 * @version 1.0
 *
 * @brief Records fused lidar point clouds and tracked-object trajectories.
 *
*/
"""

import rospy
import os
import numpy as np
import time
import json
import copy


from termcolor import colored
from std_msgs.msg import UInt64
from scipy.spatial.transform import Rotation as R
from itertools import combinations, product
from xung_recorder_pkg.msg import AugmentedCloud, TrackedObjects


ID_LEADING_ZEROS = 7
TS_LOWER_DIGITS = 9
TS_DIGITS = 13
TS_DIGITS_OUT = 16
NAME_LENGTH = 24
MAX_PROCESSING_TIME = 40


# Only these five traffic classes are available in outsight software stack
labels = {  "CAR":            {"id": 5, "color": (49,92,0), "name": "car"},
            "PERSON":         {"id": 0, "color": (5,80,255), "name": "person"},
            "TWO_WHEELER":    {"id": 1, "color": (220,117,0), "name": "bicycle"},
            "TRUCK":          {"id": 8, "color": (102,255,224), "name": "truck"},
            "BUS":            {"id": 8, "color": (102,255,224), "name": "truck"},
            "UNKNOWN":        {"id": -1, "color": (224,224,224), "name": "unknown"},
            }

# Inverse rotation from lidar coords system to xung coords system
to_xung_rotation_inv = np.array(    [[ 7.40907699e-01,  6.71447079e-01, -1.46495981e-02],
                                    [-6.71206989e-01,  7.41042962e-01,  1.83445935e-02],
                                    [ 2.31734073e-02, -3.75873800e-03,  9.99724401e-01]]
                                )

# Inverse translation from lidar coords system to xung coords system
to_xung_translation_inv = np.array([-5.19694068e-02, 3.55795709e-02, -1.46746019e-04])

# class declaration
class XungAlbFusionRecorder(object):
    """ros xung alb recorder node
    
    Args:
        object ([class]): []
    """
    
    # init class instance
    def __init__(self, name, dest_path, recording_signal=False, shutdown_signal=False, feedback_dict={}):
        
        # Recording params
        """Initialize the XungAlbFusionRecorder instance.
        
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
        self.traj_cnt = 0
        self.name = name
        self.topics = ["augmented_cloud", "tracked_objects"]
        self.dest_path = dest_path
        self.recording_signal = recording_signal
        self.shutdown_signal = shutdown_signal
        self.feedback_dict = feedback_dict
        self.subscriber_list = []
        self.track_data = {}
        self.augmented_cloud_data = {}
        self.ref_timestamp = 0
        self.previous_cloud_ts = 0
        self.previous_track_ts = 0
        self.previous_gps_cloud_ts = 0
        self.previous_gps_track_ts = 0
        self.first_cloud = True
        self.first_track = True
        
        # setup ros node
        rospy.init_node("xung_alb_" + name + "_recorder_node", anonymous=False)
        
        self.subscriber_list.append(rospy.Subscriber('xung_timestamp_node', UInt64, self.ts_callback))
        
        # setup ros subscriber
        for topic in self.topics:
            
            if topic == 'augmented_cloud':
                
                self.subscriber_list.append(rospy.Subscriber('alb_' + name + '_augmented_cloud', AugmentedCloud, self.augmented_cloud_callback))
                
            elif topic == 'tracked_objects':
                
                self.subscriber_list.append(rospy.Subscriber('alb_' + name + '_tracked_objects', TrackedObjects, self.tracked_objects_callback))
                
        # Data storage structure
        self.storage_path = os.path.join(self.dest_path, 'alb',  self.name)
        self.xyz_path = os.path.join(self.storage_path, 'augmented_cloud')
        os.makedirs(self.storage_path, exist_ok=True)
        os.makedirs(self.xyz_path, exist_ok=True)
        
        
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
            number_of_points = msg.number_of_points
            
            # Get x,y,z coordinates of point cloud data and stack into 3 channel array
            xyz_pc_data = np.asarray(msg.cartesian_coordinates, dtype=np.float32).reshape(-1,3)
            
            # Get reflectivities of point cloud data
            reflectivity_data = np.frombuffer(msg.reflectivities, dtype=np.uint8)
            
            # Get track to point mapping
            track_id_data = np.asarray(msg.object_ids, dtype=np.float32)
            
            # Save data to files, check name lenth
            #data_name = (str(self.pc_cnt).zfill(ID_LEADING_ZEROS) + '_' + str(gps_ts)).ljust(NAME_LENGTH, '0')
            data_name = (str(gps_ts)).ljust(NAME_LENGTH, '0')
            
            # Point cloud coordinates
            with open(os.path.join(self.xyz_path, data_name), 'wb') as file:
                
                xyzrid_data = np.stack([xyz_pc_data[...,0],xyz_pc_data[...,1],xyz_pc_data[...,2], reflectivity_data, track_id_data], axis=-1)
                np.save(file, xyzrid_data)
                
            # Increment id counter and Monitor processing time
            if self.first_cloud:
                
                alb_step = 0
                gps_step = 0
                self.first_cloud = False
                
            # Determine steps
            else:
                
                alb_step = ((alb_ts - self.previous_cloud_ts) / 1000)
                gps_step = ((gps_ts - self.previous_gps_cloud_ts) / 1000) 
                
            # Recording, meta and synchro data
            self.augmented_cloud_data.update({gps_ts: {"id": str(self.pc_cnt).zfill(ID_LEADING_ZEROS),
                                                        "gps_ts": gps_ts,
                                                        "alb_ts": alb_ts, 
                                                        "gps_step": gps_step, 
                                                        "alb_step": alb_step, 
                                                        "num_points": number_of_points
                                                        }})
            
            # Increment counter and update times
            self.previous_cloud_ts = alb_ts
            self.previous_gps_cloud_ts = gps_ts
            self.pc_cnt += 1
            self.pc_time = round((time.time() - start_time) * 1000)
            
            
    def tracked_objects_callback(self, msg):
        
        # Get trajectory data
        """Record objects callback.
        
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
            
            # Get timestamp and data
            object_array = msg.objects
            tracks = {}
            
            # Get track data
            for track in object_array:
                
                # Transform data
                box_size = [track.box_size.x, track.box_size.y, track.box_size.z]
                position = [track.pose.position.x, track.pose.position.y, track.pose.position.z]
                orientation = [track.pose.orientation.x, track.pose.orientation.y, track.pose.orientation.z, track.pose.orientation.w]
                speed_linear = [track.speed.linear.x, track.speed.linear.y, track.speed.linear.z]
                speed_angular = [track.speed.angular.x, track.speed.angular.y, track.speed.angular.z]
                
                # Save data
                tracks.update({track.id: {  "class_name": labels[track.object_class]['name'],
                                            "class_id": labels[track.object_class]['id'], 
                                            "orig_pose_position": position,
                                            "orig_pose_orientation": orientation,
                                            "xung_pose_position": self.transform_3d_point(np.array(position), to_xung_rotation_inv, to_xung_translation_inv).tolist(),
                                            "xung_pose_cuboid": self.get_cuboid(box_size=box_size, position=position, orientation=orientation),
                                            "box_size": box_size,
                                            "box_volume_m3": round(track.box_size.x * track.box_size.y* track.box_size.z, 2),
                                            "speed_linear": speed_linear,
                                            "speed_angular": speed_angular,
                                            "speed_kmh": round(3.6 * np.linalg.norm(np.array(speed_linear)), 2),
                                            "speed_ms": round(np.linalg.norm(np.array(speed_linear)), 2)
                                            }})
                
            # Increment id counter, monitor processing time, get first track id of current recording
            if self.first_track:
                
                # First step is zero
                alb_step = 0
                gps_step = 0
                self.first_track = False
            
            # Determine steps
            else:
                
                alb_step = ((alb_ts - self.previous_cloud_ts) / 1000)
                gps_step = ((gps_ts - self.previous_gps_track_ts) / 1000)
                
            # Recording, meta and synchro data
            self.track_data.update({gps_ts: {"id": str(self.traj_cnt).zfill(ID_LEADING_ZEROS),
                                            "gps_ts": gps_ts,
                                            "alb_ts": alb_ts, 
                                            "gps_step": gps_step, 
                                            "alb_step": alb_step,
                                            "num_tracks": len(tracks),
                                            "tracks": tracks
                                            }})
            
            # Increment counter and update times
            self.previous_alb_track_ts = alb_ts 
            self.previous_gps_track_ts = gps_ts     
            self.traj_cnt += 1
            
            
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
    
    
    def transform_3d_point(self, point_coordinates: np.ndarray, rotation: np.ndarray, translation: np.ndarray):
        """Apply a rotation then a translation to a coordinate vector.

        :param point_coordinates: 3D coordinate vector of a point
        :param rotation: matrix (3x3)
        :param translation: vector (1x3)
        :return: rotated then translated 3D vector
        """
        
        rotated_points = np.dot(rotation, point_coordinates)
        return rotated_points + translation
    
    
    def get_cuboid(self, box_size, position, orientation):
        
        # Get object descriptions
        """Return cuboid.
        
        Args:
            box_size (object): Input box size value.
            position (object): Input position value.
            orientation (object): Input orientation value.
        
        Returns:
            object: Result produced by the operation.
        """
        shape = np.asarray(box_size)
        translation = np.asarray(position)
        rotation = R.from_quat(orientation).as_matrix()
        
        # Calculate object cuboid with 12 egdes
        edges = []
        width = [-shape[0]/2, shape[0]/2]
        length = [-shape[1]/2, shape[1]/2]
        height = [-shape[2]/2, shape[2]/2]
        
        for corner_1, corner_2 in combinations(np.array(list(product(width, length, height))), 2):
            
            # A vector is an edge when it is along 1 axis. Example of edge along y: corner_1 - corner_2 = [0, X, 0]
            if len(np.where(np.abs(corner_1 - corner_2) == 0)[0]) == 2:
                
                # Create corners
                first_corner = self.transform_3d_point(corner_1, rotation, translation).reshape(3, 1)
                second_corner = self.transform_3d_point(corner_2, rotation, translation).reshape(3, 1)
                
                # Apply transormation from alb coord system into xung coord system
                first_corner = self.transform_3d_point(first_corner.reshape(3), to_xung_rotation_inv, to_xung_translation_inv).reshape(3, 1)
                second_corner = self.transform_3d_point(second_corner.reshape(3), to_xung_rotation_inv, to_xung_translation_inv).reshape(3, 1)
                
                # Stack
                edges.append([[first_corner[0][0], first_corner[1][0], first_corner[2][0]], [second_corner[0][0], second_corner[1][0], second_corner[2][0]]])
                
        return edges
        
        
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
            self.feedback_dict[self.name] = [self.pc_cnt, self.traj_cnt, self.pc_time]
            
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
                
        # Tracks
        if self.augmented_cloud_data:
            
            json_file = os.path.join(self.storage_path, self.name + '_tracked_objects.json')
                
            with open(json_file, 'w') as fp:
                json.dump(self.track_data, fp)
                
        # Feedback
        print(colored(f" {self.name} closed - PC: {self.pc_cnt}, Tracks: {self.traj_cnt}", 'green'))
        
        return