#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 12 12:49:49 2018

@author: ilias
"""
import boto3
from inotify_simple import INotify, flags
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from time import sleep
from datetime import date, datetime
import json
import cv2
#AWS S3 client configuration


# MQTT Connection establishement
myMQTTClient = AWSIoTMQTTClient("Litterbug_desktop")
myMQTTClient.configureEndpoint("a1oa9tg9lcso0.iot.eu-west-1.amazonaws.com", 8883)
myMQTTClient.configureCredentials("certs/VeriSign-Class 3-Public-Primary-Certification-Authority-G5.pem", "certs/b426e422b6-private.pem.key", "certs/b426e422b6-certificate.pem.crt")
myMQTTClient.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
myMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
myMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
myMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec
#connect and publish
myMQTTClient.connect()
myMQTTClient.publish("mytopic", "connected", 0)

# inotify initialization 
inotify = INotify()
watch_flags = flags.CREATE | flags.MODIFY 
wd = inotify.add_watch('detections', watch_flags)
while 1:
       events= inotify.read(3000,2000)
       print ("evnets "+ str(events))
       if events: 
           #if litter event send detection image to S3 bucket "littercam"
            image=cv2.imread("detections/litter.bmp");
            cv2.imwrite('detections/litter.jpg',image)
            s3 = boto3.client('s3')          
            s3.upload_file("detections/litter.jpg", "littercam", "litter.jpg")         
           # and send message to the MQTT topic 
            message = {}
            message['message'] = "Littering is detected"
            message['imageKey'] = events[-1].name
            messageJson = json.dumps(message)
            myMQTTClient.publish("mytopic", messageJson, 1)
            event=events[-1] 
            print(event)
            for flag in flags.from_mask(event.mask):
               print('    ' + str(flag))


