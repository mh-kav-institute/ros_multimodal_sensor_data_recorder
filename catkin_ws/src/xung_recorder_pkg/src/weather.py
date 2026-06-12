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
 * @file weather.py for Xung data recording
 * @author Manuel Hetzel
 * @date 4.10.2023
 * @version 1.0
 *
 * @brief Records synchronized weather station observations and metadata.
 *
*/
"""

import rospy
import json
import os
import time
import copy

from termcolor import colored
from std_msgs.msg import Float32MultiArray


UPDATE_RATE = 5


# class declaration
class XungWeatherRecorder(object):
    """ros xung weather recorder node

    Args:
        object ([class]): []
    """
    
    # init class instance
    def __init__(self, dest_path, recording_signal=False, shutdown_signal=False, feedback_dict={}):
        
        # Recording params
        """Initialize the XungWeatherRecorder instance.
        
        Args:
            dest_path (object): Input dest path value.
            recording_signal (object): Input recording signal value.
            shutdown_signal (object): Input shutdown signal value.
            feedback_dict (object): Input feedback dict value.
        
        Returns:
            None
        """
        self.name = 'weather'
        self.weather_data_cnt = 0
        self.pc_time = 0
        self.dest_path = dest_path
        self.recording_signal = recording_signal
        self.shutdown_signal = shutdown_signal
        self.feedback_dict = feedback_dict
        self.weather_data = {}
        self.update_cnt = 0
        
        # setup ros node
        rospy.init_node("xung_weather_recorder_node", anonymous=False)      
        self.subscriber = rospy.Subscriber('xung_weather_data', Float32MultiArray, self.weather_data_callback)
        
        # Data storage structure
        self.storage_path = os.path.join(self.dest_path, 'meta/weather')
        os.makedirs(self.storage_path, exist_ok=True)
        
        
    def weather_data_callback(self, msg):
        
        # Get augmented cloud data
        """Record data callback.
        
        Args:
            msg (object): Input msg value.
        
        Returns:
            None
        """
        if self.recording_signal.value:
            
            # Only update every x seconds
            if self.update_cnt == 0:
            
                # Monitor processing time
                start_time = time.time()
                
                # Get data from subscriber
                data = copy.deepcopy(msg.data)
                gps_ts = (int(data[17]) << 32 | int(data[18]))
                
                self.weather_data.update({gps_ts: { "air_pressure": round(data[10], 2),
                                                    "air_temperature": round(data[4], 2),
                                                    "relative_humidity": round(data[5], 2),
                                                    "wind_speed": round(data[12], 2),
                                                    "wind_direction": round(data[13], 2),
                                                    "precipitation_intensity": round(data[3], 2),
                                                    "precipitation_amount": round(data[1], 2),
                                                    "visibility": round(data[15], 2),
                                                    "weather_nws": int(data[1])
                                                }})
                
                self.weather_data_cnt += 1
                self.pc_time = round((time.time() - start_time) * 1000)
                self.update_cnt += 1
            
            # Check counter
            else:
                
                # Reset counter
                if self.update_cnt >= UPDATE_RATE:
                    
                    self.update_cnt = 0
                    
                # Increment counter
                else:
                    
                    self.update_cnt += 1
        
        
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
            self.feedback_dict[self.name] = [self.weather_data_cnt, self.pc_time]
            
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
        if self.weather_data:
            
            json_file = os.path.join(self.storage_path, 'weather_data.json')
            
            with open(json_file, 'w') as fp:
                json.dump(self.weather_data, fp)
                
        # Feedback
        print(colored(f" {self.name} closed - Weather: {self.weather_data_cnt}", 'green'))
        
        return