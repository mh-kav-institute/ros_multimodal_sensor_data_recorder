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
 * @file lsa.py for Xung data recording
 * @author Manuel Hetzel
 * @date 4.10.2023
 * @version 1.0
 *
 * @brief Records traffic-light signal state data and timestamps.
 *
*/
"""

import rospy
import json
import os
import time
import copy

from termcolor import colored
from std_msgs.msg import UInt32MultiArray


# class declaration
class XungLSARecorder(object):
    """ros xung traffic signal recorder node

    Args:
        object ([class]): []
    """
    
    # init class instance
    def __init__(self, dest_path, recording_signal=False, shutdown_signal=False, feedback_dict={}):
        
        # Recording params
        """Initialize the XungLSARecorder instance.
        
        Args:
            dest_path (object): Input dest path value.
            recording_signal (object): Input recording signal value.
            shutdown_signal (object): Input shutdown signal value.
            feedback_dict (object): Input feedback dict value.
        
        Returns:
            None
        """
        self.name = 'lsa'
        self.lsa_data_cnt = 0
        self.pc_time = 0
        self.dest_path = dest_path
        self.recording_signal = recording_signal
        self.shutdown_signal = shutdown_signal
        self.feedback_dict = feedback_dict
        self.lsa_data = {}
        
        # setup ros node
        rospy.init_node("xung_lsa_recorder_node", anonymous=False)      
        self.subscriber = rospy.Subscriber('xung_lsa_data', UInt32MultiArray, self.lsa_data_callback)
        
        # Data storage structure
        self.storage_path = os.path.join(self.dest_path, 'meta/lsa')
        os.makedirs(self.storage_path, exist_ok=True)
        
        
    def lsa_data_callback(self, msg):
        
        # Get augmented cloud data
        """Record data callback.
        
        Args:
            msg (object): Input msg value.
        
        Returns:
            None
        """
        if self.recording_signal.value:
            
            # Monitor processing time
            start_time = time.time()
            
            # Get data from subscriber
            data = copy.deepcopy(msg.data)
            gps_ts = (int(data[12]) << 32 | int(data[13]))
            
            # Save data
            self.lsa_data.update({gps_ts: { "k1": data[0],
                                            "k2": data[1],
                                            "k3": data[2],
                                            "k4": data[3],
                                            "k5": data[4],
                                            "k6": data[5],
                                            "f1": data[6],
                                            "f2": data[7],
                                            "f3": data[8],
                                            "b1": data[9],
                                            "b3": data[10],
                                            "b4": data[11],
                                            }})
            
            self.lsa_data_cnt += 1
            self.pc_time = round((time.time() - start_time) * 1000)
        
        
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
            self.feedback_dict[self.name] = [self.lsa_data_cnt, self.pc_time]
            
            # Else sleep for 5 ms and retest
            rospy.sleep(0.005)
            
        self.stop() 
        
        
    def stop(self):
        
        # Unregister from all subscriped topics   
        """Stop the configured operation.
        
        Returns:
            None
        """
        self.subscriber.unregister()
            
        # Sleep for 250 ms to finish all callbacks
        rospy.sleep(0.25)
            
        # Save sync and meta data
        # Augmented cloud
        if self.lsa_data:
            
            json_file = os.path.join(self.storage_path, 'lsa_data.json')
            
            with open(json_file, 'w') as fp:
                json.dump(self.lsa_data, fp)
                
        # Feedback
        print(colored(f" {self.name} closed - Signals: {self.lsa_data_cnt}", 'green'))
        
        return