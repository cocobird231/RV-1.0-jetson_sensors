#ver=2.0
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import json
import yaml
from yaml import load, Loader

pkgName = 'cpp_zedcam'

def generate_launch_description():
    commonFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/common.yaml')
    with open(commonFilePath, 'r') as f:
        data = yaml.load(f, Loader=Loader)

    serviceFilePath = os.path.join(get_package_share_directory(pkgName), 'launch/service.json')
    with open(serviceFilePath, 'r') as f:
        serviceData = json.load(f)

    return LaunchDescription([
        Node(
            package=pkgName,
            node_namespace=data['generic_prop']['namespace'],
            node_executable="pub",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "topic_ZEDCam_RGB_nodeName" : data['topic_ZEDCam_RGB']['nodeName'], 
                    "topic_ZEDCam_RGB_topicName" : data['topic_ZEDCam_RGB']['topicName'], 
                    "topic_ZEDCam_RGB_pubInterval_s" : data['topic_ZEDCam_RGB']['publishInterval_s'], 
                    "topic_ZEDCam_RGB_width" : data['topic_ZEDCam_RGB']['width'], 
                    "topic_ZEDCam_RGB_height" : data['topic_ZEDCam_RGB']['height'], 
                    "topic_ZEDCam_Depth_nodeName" : data['topic_ZEDCam_Depth']['nodeName'], 
                    "topic_ZEDCam_Depth_topicName" : data['topic_ZEDCam_Depth']['topicName'], 
                    "topic_ZEDCam_Depth_pubInterval_s" : data['topic_ZEDCam_Depth']['publishInterval_s'], 
                    "topic_ZEDCam_Depth_width" : data['topic_ZEDCam_Depth']['width'], 
                    "topic_ZEDCam_Depth_height" : data['topic_ZEDCam_Depth']['height'], 
                    "topicIDs" : data['camera_prop']['topicIDs'], 
                    "camera_caps" : data['camera_prop']['caps'], 
                    "camera_fps" : data['camera_prop']['fps'], 
                    "camera_width" : data['camera_prop']['width'], 
                    "camera_height" : data['camera_prop']['height'], 
                    "camera_sensing_mode" : data['camera_prop']['sensing_mode'], 
                    "camera_depth_quality" : data['camera_prop']['depth_quality'], 
                    "camera_depth_unit" : data['camera_prop']['depth_unit'], 
                    "camera_use_color" : data['camera_prop']['use_color'], 
                    "camera_use_depth" : data['camera_prop']['use_depth'], 

                    # Settings for Params class under vehicle_interfaces/params.h
                    # Do not change the settings rashly
                    "nodeName" : data['generic_prop']['nodeName'] + '_' + str(data['generic_prop']['id']) + '_node', 
                    "id" : data['generic_prop']['id'], 
                    "devInfoService" : serviceData['devInfoService'], 
                    "devInterface" : serviceData['devInterface'], 
                    "devMultiNode" : serviceData['devMultiNode'], 
                    "qosService" : serviceData['qosService'], 
                    "qosDirPath" : serviceData['qosDirPath'], 
                    "safetyService" : serviceData['safetyService'], 
                    "timesyncService" : serviceData['timesyncService'], 
                    "timesyncPeriod_ms" : serviceData['timesyncPeriod_ms'], 
                    "timesyncAccuracy_ms" : serviceData['timesyncAccuracy_ms'], 
                    "timesyncWaitService" : serviceData['timesyncWaitService'], 
                }
            ]
        )
    ])