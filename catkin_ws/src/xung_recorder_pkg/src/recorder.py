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
 * @file recorder.py for Xung data recording
 * @author Manuel Hetzel
 * @date 4.10.2023
 * @version 1.0
 *
 * @brief Orchestrates parallel camera, lidar, weather, and traffic-light recording processes.
 *
*/
"""

import rospy
import os
import time
import json
import psutil
import multiprocessing as mp

from collections import OrderedDict
from termcolor import colored
from multiprocessing import Process
from std_msgs.msg import UInt64
from alb_edge import XungAlbEdgeRecorder
from alb_fusion import XungAlbFusionRecorder
from camera import XungCAMRecorder
from weather import XungWeatherRecorder
from lsa import XungLSARecorder

# Fore this process to use defined gpu device id
os.environ['CUDA_VISIBLE_DEVICES'] = "0,1,2"


GPU_ID_LIMIT = 2
MAX_ALB_PROCESSING_TIME = 40
MAX_CTX_PROCESSING_TIME = 40
MAX_CAM_PROCESSING_TIME = 30


def create_weather_recorder_instance(dest_path, cpuid, recording_signal, shutdown_signal, feedback_dict):
    
    # Force this process to run only on the defined cpu core(s)
    """Create weather recorder instance.
    
    Args:
        dest_path (object): Input dest path value.
        cpuid (object): Input cpuid value.
        recording_signal (object): Input recording signal value.
        shutdown_signal (object): Input shutdown signal value.
        feedback_dict (object): Input feedback dict value.
    
    Returns:
        None
    """
    p = psutil.Process()
    p.cpu_affinity([cpuid])
    
    # console feedback
    print(colored(f"   - Xung Recorder Node: Instance Weather on cpu: {cpuid}", 'green'))

    # Create instance
    instance = XungWeatherRecorder(dest_path=dest_path, recording_signal=recording_signal, shutdown_signal=shutdown_signal, feedback_dict=feedback_dict)  
    
    # Start instance in wait mode
    instance.start()
    
    
def create_lsa_recorder_instance(dest_path, cpuid, recording_signal, shutdown_signal, feedback_dict):
    
    # Force this process to run only on the defined cpu core(s)
    """Create lsa recorder instance.
    
    Args:
        dest_path (object): Input dest path value.
        cpuid (object): Input cpuid value.
        recording_signal (object): Input recording signal value.
        shutdown_signal (object): Input shutdown signal value.
        feedback_dict (object): Input feedback dict value.
    
    Returns:
        None
    """
    p = psutil.Process()
    p.cpu_affinity([cpuid])
    
    # console feedback
    print(colored(f"   - Xung Recorder Node: Instance LSA on cpu: {cpuid}", 'green'))

    # Create instance
    instance = XungLSARecorder(dest_path=dest_path, recording_signal=recording_signal, shutdown_signal=shutdown_signal, feedback_dict=feedback_dict)  
    
    # Start instance in wait mode
    instance.start()
    
    
def create_cam_recorder_instance(cam, dest_path, cpuid, gpuid, recording_signal, shutdown_signal, feedback_dict):
    
    # console feedback
    """Create cam recorder instance.
    
    Args:
        cam (object): Input cam value.
        dest_path (object): Input dest path value.
        cpuid (object): Input cpuid value.
        gpuid (object): Input gpuid value.
        recording_signal (object): Input recording signal value.
        shutdown_signal (object): Input shutdown signal value.
        feedback_dict (object): Input feedback dict value.
    
    Returns:
        None
    """
    print(colored(f"   - Xung CAM Recorder Node: Instance {cam} on cpu: {cpuid} - gpu: {gpuid}", 'green'))
    
    # Force this process to run only on the defined cpu core(s)
    p = psutil.Process()
    p.cpu_affinity([cpuid])
    
    # Create instance
    instance = XungCAMRecorder(name=cam, gpuid=gpuid, dest_path=dest_path, recording_signal=recording_signal, shutdown_signal=shutdown_signal, feedback_dict=feedback_dict)  
    
    # Start instance in wait mode
    instance.start()
    

def create_alb_recorder_instance(alb, dest_path, cpuid, recording_signal, shutdown_signal, feedback_dict):
    
    # console feedback
    """Create alb recorder instance.
    
    Args:
        alb (object): Input alb value.
        dest_path (object): Input dest path value.
        cpuid (object): Input cpuid value.
        recording_signal (object): Input recording signal value.
        shutdown_signal (object): Input shutdown signal value.
        feedback_dict (object): Input feedback dict value.
    
    Returns:
        None
    """
    print(colored(f"   - Xung ALB Recorder Node: Instance {alb} on cpu: {cpuid}", 'green'))
    
    # Force this process to run only on the defined cpu core(s)
    p = psutil.Process()
    p.cpu_affinity([cpuid])
    
    # Create instance
    if 'edge' in alb:
            
        instance = XungAlbEdgeRecorder(name=alb, dest_path=dest_path, recording_signal=recording_signal, shutdown_signal=shutdown_signal, feedback_dict=feedback_dict)  
    
    elif 'fusion in alb':
        
        instance = XungAlbFusionRecorder(name=alb, dest_path=dest_path, recording_signal=recording_signal, shutdown_signal=shutdown_signal, feedback_dict=feedback_dict)  
        
    # Start instance in wait mode
    instance.start()
    

def create_feedback_instance(cams, albs, context, cpuid, recording_signal, shutdown_signal, feedback_dict):
    
    # Force this process to run only on the defined cpu core(s)
    """Create feedback instance.
    
    Args:
        cams (object): Input cams value.
        albs (object): Input albs value.
        context (object): Input context value.
        cpuid (object): Input cpuid value.
        recording_signal (object): Input recording signal value.
        shutdown_signal (object): Input shutdown signal value.
        feedback_dict (object): Input feedback dict value.
    
    Returns:
        None
    """
    p = psutil.Process()
    p.cpu_affinity([cpuid])
    first = True
    
    # Only print feedback if not stopped and recording is active
    while not shutdown_signal.value:
        
        if recording_signal.value: 
            
            # For better console readability
            if first:
                
                time.sleep(0.5)
                print('\n')
                first = False
            
            # Create the feedback message
            cam_message = ''
            alb_message = ''
            ctx_message = ''
            cam_color = 'green'
            alb_color = 'green'
            ctx_color = 'green'
            
            # Camera recorder(s) feedback
            for cam in cams:
                
                # Get feedback data from camera recording process
                data = feedback_dict[cam]
                cam_message = cam_message + f'{cam} | {data[0]} | '
                
                # check processing time
                if data[1] > MAX_CAM_PROCESSING_TIME:
                    
                    cam_color = 'red'
                    
                elif data[1] > (MAX_CAM_PROCESSING_TIME - 0.1*MAX_CAM_PROCESSING_TIME):
                
                    cam_color = 'yellow'
                
            # ALB Recorder(s feedback)
            for alb in albs:
                
                # Get feedback data from alb recording process
                data = feedback_dict[alb]
                
                if 'edge' in alb:
                    
                    alb_message = alb_message + f'{alb} | {data[0]} | '
                
                    # check processing time
                    if data[1] > MAX_ALB_PROCESSING_TIME:
                        
                        alb_color = 'red'
                        
                    elif data[1] > (MAX_ALB_PROCESSING_TIME - 0.1*MAX_ALB_PROCESSING_TIME):
                    
                        alb_color = 'yellow'
                        
                elif 'fusion' in alb:
                    
                    alb_message = alb_message + f'{alb} | {data[0]} | {data[1]} | '
                
                    # check processing time
                    if data[2] > MAX_ALB_PROCESSING_TIME:
                        
                        alb_color = 'red'
                        
                    elif data[2] > (MAX_ALB_PROCESSING_TIME - 0.1*MAX_ALB_PROCESSING_TIME):
                    
                        alb_color = 'yellow'
                        
            # Context Recorder(s feedback)
            for ctx in context:
                
                # Get feedback data from context recording process
                data = feedback_dict[ctx]
                
                if ctx == 'weather':
                
                    ctx_message = ctx_message + f'we | {data[0]} | '
                    
                elif ctx == 'lsa':
                
                    ctx_message = ctx_message + f'lsa | {data[0]} | '
                
                # check processing time
                if data[1] > MAX_CTX_PROCESSING_TIME:
                    
                    ctx_color = 'red'
                    
                elif data[1] > (MAX_CTX_PROCESSING_TIME - 0.1*MAX_CTX_PROCESSING_TIME):
                
                    ctx_color = 'yellow'
                
                
            # Update loop is every 250 ms
            print(colored(cam_message, cam_color), '|', colored(alb_message, alb_color), '|', colored(ctx_message, ctx_color), end='\r')    
            time.sleep(0.25)
            
            
def create_master_ts_file(cfg, dest_path):
    """Create global master timestamp synchronization overview

    Args:
        cfg (dict): config data
        dest_path (str): sync file full destination path and file name
    """
    
    os.makedirs(os.path.join(dest_path, 'meta'), exist_ok=True)
    master_sync_file = os.path.join(dest_path, 'meta', 'ts_sync.json')
    
    # get lists of all sensor data objects
    alb_list = []
    cam_list = []
    context_list = []
    
    # cameras are the images
    for cam in cfg['cams']:
        
        cam_list.append([cam, cam])
    
    # albs are augmented cloud and track data
    for alb in cfg['albs']:
        
        if 'edge' in alb:
        
            alb_list.append([alb, alb + '_augmented_cloud'])
            
        if 'fusion' in alb:
        
            alb_list.append([alb, alb + '_augmented_cloud'])
            alb_list.append([alb, alb + '_tracked_objects'])
            
    # contexts
    for ctx in cfg['context']:
        
        context_list.append([ctx, ctx])
        
    # Both camera and alb data was recorded
    if cam_list and alb_list:
        
        master_dict = camera_and_albs(cam_list, alb_list, cfg, dest_path)
        
    # Only camera data was recorded
    elif cam_list and not alb_list:
        
        master_dict = camera_only(cam_list, cfg, dest_path)
    
    # Only alb data was recorded
    elif alb_list and not cam_list:
        
        master_dict = alb_only(alb_list, dest_path)
        
    # Error case
    else:
        
        print("No camera or alb data was recorded, not master ts synch file will be created")
        return
    
    # We have also recorded context data, append ts sync info to master dict
    if context_list:
        
        master_dict = context(context_list, dest_path, master_dict)
            
    # save to sync file                        
    with open(master_sync_file, 'w') as f:
        json.dump(master_dict, f)
        
        
def camera_and_albs(cam_list, alb_list, cfg, dest_path):
    
    """Build and albs.
    
    Args:
        cam_list (object): Input cam list value.
        alb_list (object): Input alb list value.
        cfg (object): Input cfg value.
        dest_path (object): Input dest path value.
    
    Returns:
        object: Result produced by the operation.
    """
    sensor_list = cam_list + alb_list
    master_dict = {}
    
    # first process cameras, because they use gps 25 Hz timestamps as trigger signal
    for cam in cfg['cams']:
        
        n = 'camera_' + cam
        p = os.path.join(dest_path, 'camera', 'raw', n + '.json')
        
        if os.path.exists(p):
            
            with open(p, 'r') as f:
                data = json.load(f)
                
            # do for every data timestamp
            for k, v in data.items():
                
                # ts is already in master
                if k in master_dict:
                    
                    r = master_dict[k]
                    r[cam] = True
                    
                # ts is not in master yet 
                else:
                    
                    s = {key[1]: False for key in sensor_list}
                    s[cam] = True
                
                    master_dict.update({k: s})
                    
    # order by timestamp
    master_dict = OrderedDict(sorted(master_dict.items()))   
    first_key = list(master_dict.keys())[0]
    last_key = list(master_dict.keys())[-1]
    keys = list(master_dict.keys())
    
    # second process alb data(s) and find best matching camera ts for matching
    for alb in alb_list:
    
        p = os.path.join(dest_path, 'alb', alb[0], alb[1] + '.json')
        
        if os.path.exists(p):
            
            with open(p, 'r') as f:
                data = json.load(f)
                
            # Add new entry to every yet existing key
            for k in master_dict:
                
                r = master_dict[k]              
                r.update({alb[1]: False})  
                
            # do for every data timestamp
            for k, v in data.items():
                
                # ts is already in master
                if k in master_dict:
                    
                    r = master_dict[k]
                    r[alb[1]] = True
                
                # ts is not in master yet
                else:
                    
                    # not in between current timestamps -> expand
                    if k < first_key or k > last_key:
                        
                        s = {key[1]: False for key in sensor_list}
                        s[alb[1]] = True
                        master_dict.update({k: s})
                    
                    # in between current timestamps -> find closest match
                    else:
                        
                        diff_list = [abs(int(k) - int(key)) for key in keys]
                        diff = min(diff_list)
                        idx = diff_list.index(diff)                   
                        r = master_dict[keys[idx]]
                        r[alb[1]] = True
                        
    # Order once again         
    master_dict = OrderedDict(sorted(master_dict.items()))              
    return master_dict
    
    
def camera_only(cam_list, cfg, dest_path):
    
    """Build only.
    
    Args:
        cam_list (object): Input cam list value.
        cfg (object): Input cfg value.
        dest_path (object): Input dest path value.
    
    Returns:
        object: Result produced by the operation.
    """
    sensor_list = cam_list
    master_dict = {}
    
    # first process cameras, because they use gps 25 Hz timestamps as trigger signal
    for cam in cfg['cams']:
        
        n = 'camera_' + cam
        p = os.path.join(dest_path, 'camera', 'raw', n + '.json')
        
        if os.path.exists(p):
            
            with open(p, 'r') as f:
                data = json.load(f)
                
            # do for every data timestamp
            for k, v in data.items():
                
                # ts is already in master
                if k in master_dict:
                    
                    r = master_dict[k]
                    r[cam] = True
                    
                # ts is not in master yet 
                else:
                    
                    s = {key[1]: False for key in sensor_list}
                    s[cam] = True
                
                    master_dict.update({k: s})
                    
    # order by timestamp
    master_dict = OrderedDict(sorted(master_dict.items()))   
                        
    return master_dict


def alb_only(alb_list, dest_path):
    
    """Build only.
    
    Args:
        alb_list (object): Input alb list value.
        dest_path (object): Input dest path value.
    
    Returns:
        object: Result produced by the operation.
    """
    sensor_list = alb_list
    master_dict = {}
    
    # first process cameras, because they use gps 25 Hz timestamps as trigger signal
    for alb in alb_list:
        
        p = os.path.join(dest_path, 'alb', alb[0], alb[1] + '.json')
        
        if os.path.exists(p):
            
            with open(p, 'r') as f:
                data = json.load(f)
                
            # do for every data timestamp
            for k, v in data.items():
                
                # ts is already in master
                if k in master_dict:
                    
                    r = master_dict[k]
                    r[alb[1]] = True
                    
                # ts is not in master yet 
                else:
                    
                    s = {key[1]: False for key in sensor_list}
                    s[alb[1]] = True
                
                    master_dict.update({k: s})
                    
    # Order by timestamp
    master_dict = OrderedDict(sorted(master_dict.items()))
    return master_dict   


def context(context_list, dest_path, master_dict):
    
    """Merge the configured operation.
    
    Args:
        context_list (object): Input context list value.
        dest_path (object): Input dest path value.
        master_dict (object): Input master dict value.
    
    Returns:
        object: Result produced by the operation.
    """
    sensor_list = context_list
    
    # order by timestamp
    master_dict = OrderedDict(sorted(master_dict.items()))   
    first_key = list(master_dict.keys())[0]
    last_key = list(master_dict.keys())[-1]
    keys = list(master_dict.keys())
    
    # second process alb data(s) and find best matching camera ts for matching
    for ctx in sensor_list:
    
        p = os.path.join(dest_path, 'meta',  ctx[0], ctx[0] + '_data.json')
        
        if os.path.exists(p):
            
            with open(p, 'r') as f:
                data = json.load(f)
            
            # Add new entry to every yet existing key
            for k in master_dict:
                
                r = master_dict[k]              
                r.update({ctx[0]: False})        
                
            # do for every data timestamp
            for k, v in data.items():
                
                # ts is already in master
                if k in master_dict:
                    
                    r = master_dict[k]
                    r[ctx[0]] = True
                
                # ts is not in master yet
                else:
                    
                    # not in between current timestamps -> expand
                    if k < first_key or k > last_key:
                        
                        # s = {key[1]: None for key in sensor_list}
                        # s[ctx[0]] = True
                        # master_dict.update({k: s})
                        continue
                    
                    # in between current timestamps -> find closest match
                    else:
                        
                        diff_list = [abs(int(k) - int(key)) for key in keys]
                        diff = min(diff_list)
                        idx = diff_list.index(diff)                   
                        r = master_dict[keys[idx]]
                        r[ctx[0]] = True
                    
    # Order by timestamp
    master_dict = OrderedDict(sorted(master_dict.items()))
    return master_dict 
    
    

if __name__ == '__main__':
    
    # For Debugging and vs code attaching
    DEBUG = False
    
    if DEBUG:
        
        import debugpy
        
    if DEBUG:
        
        debugpy.listen(5680)
        print("Waiting for debugger attach")
        debugpy.wait_for_client()
        
    # Program starts here   
    # Load config params
    with open('/workspace/repos/catkin_ws/src/xung_recorder_pkg/config/config.json') as json_file:
        
        cfg = json.load(json_file)
        
    # Force this process to run only on the defined cpu cores
    cpu_cnt = 1
    gpu_cnt = 0
    p = psutil.Process()
    p.cpu_affinity([cfg['cpu_id']])
    
    dest_path = os.path.join(cfg['recording_dir'], 'current_recording')
    os.makedirs(dest_path, exist_ok=True)
    recorders = []
    manager = mp.Manager()
    recording_signal = manager.Value('b', False)
    shutdown_signal = manager.Value('b', False)
    feedback_dict = manager.dict()
    
    # Create feedback shared object dict for contexts
    for ctx in cfg['context']:
        
        feedback_dict[ctx] = [0, 0]
    
    # Create feedback shared object dict for albs
    for alb in cfg['albs']:
        
        feedback_dict[alb] = [0, 0]
        
    # Create feedback shared object dict for cameras
    for cam in cfg['cams']:
        
        feedback_dict[cam] = [0, 0]
        
    
    # Create multiple recorder instances for context targets
    for ctx in cfg['context']:
        
        cpuid = cfg['cpu_id']
        
        if ctx == 'weather':
            
            recorders.append(Process(target=create_weather_recorder_instance, args=(dest_path, cpuid, recording_signal, shutdown_signal, feedback_dict)))
            
        elif ctx == 'lsa':
            
            recorders.append(Process(target=create_lsa_recorder_instance, args=(dest_path, cpuid, recording_signal, shutdown_signal, feedback_dict)))
            
        # Create multiple recorder instances for camera targets
    for cam in cfg['cams']:
        
        # Manage cpu core allocation (multithreading)
        gpuid = gpu_cnt
        cpuid = cfg['cpu_id'] + cpu_cnt
        cpu_cnt += 1
        gpu_cnt += 1
        
        # Setup recorder instance for current camera
        recorders.append(Process(target=create_cam_recorder_instance, args=(cam, dest_path, cpuid, gpuid, recording_signal, shutdown_signal, feedback_dict)))
        
        # reset gpu id counter
        if gpu_cnt > GPU_ID_LIMIT:
            
            gpu_cnt = 0
            
    # Create multiple recorder instances for alb targets
    for alb in cfg['albs']:
        
        # Manage topic name and cpu core allocation (multithreading)
        cpuid = cfg['cpu_id'] + cpu_cnt
        cpu_cnt += 1
        
        # Setup recorder instance for current alb
        recorders.append(Process(target=create_alb_recorder_instance, args=(alb, dest_path, cpuid, recording_signal, shutdown_signal, feedback_dict)))
        
    # Create feedback instance
    recorders.append(Process(target=create_feedback_instance, args=(cfg['cams'], cfg['albs'], cfg['context'], cfg['cpu_id'], recording_signal, shutdown_signal, feedback_dict)))
        
    # Start all recorders in wait mode
    for p in recorders:
        
        p.start()
        
    # Wait for user input to start recording, then set recording flag to true
    time.sleep(0.5)
    input(colored("\nPress ENTER to start recording...\n\n", 'green')) 
    
    # Get current timestamp of recording start for scene naming
    rospy.init_node("xung_alb_recorder_node", anonymous=False)
    ct = (rospy.wait_for_message('xung_timestamp_node', UInt64)).data
    ct = time.localtime(ct/1000000)
    scene_name = str(ct.tm_year) + str(ct.tm_mon).zfill(2) + str(ct.tm_mday).zfill(2) + '_' + str(ct.tm_hour).zfill(2) + str(ct.tm_min).zfill(2) + str(ct.tm_sec).zfill(2)
    
    # Set recording flag to start recorders
    recording_signal.value = True
    
    # Wait for user input to stop current ongoing recording, then set corresponding flags
    input(colored("Press ENTER to stop current recording...\n\n", 'green'))    
    recording_signal.value = False
    shutdown_signal.value = True
    
    # Wait till all recorder instances completed and stopped themselves
    for p in recorders:
        
        p.join()
        
    # Create Master sync list
    create_master_ts_file(cfg, dest_path)
        
    # rename recording from temp name to real scene name
    os.rename(dest_path, os.path.join(cfg['recording_dir'], scene_name))

    # Console feedback
    print(colored("\nXung Recorder Node: exit", 'green'))