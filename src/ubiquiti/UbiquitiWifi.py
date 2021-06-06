#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Val Schmidt
Center for Coastal and Ocean Mapping
University of New Hampshire
Copyright 2021, all rights reservered.
"""
import os
import datetime
import json

class UbiquitiWifi():
    
    def __init__(self,ipaddr=None):
        
        self.ipaddr = ipaddr
        self.name = None
        self.airOSVersion = None
        
        self.statusraw = None
        self.status = None
        self.statustime = None
        self.signalraw = None
        self.signal = None
        self.signaltime = None
        
        self.username = None
        self.passwd = None

    def getAirOSVersion(self):
    
        if self.username is None or self.passwd is None:
            print('ERROR: Set username and password!')
            return
            
        try:
            cmd = ("sshpass -p" + self.passwd + " ssh " + self.username +
                   "@" + self.ipaddr + " cat /etc/version")
            #print(cmd)
            stream = os.popen(cmd)
            tmp = stream.read()
            self.airOSVersion = tmp[(tmp.find('v')+1):].rstrip()
            print('Got airOS Version: %s' % self.airOSVersion)
        except Exception as e:
            print("Could not get airOS Version")
            print(e)
            print("Attempting default: '8.7.0'")
            self.airOSVersion = '8.7.0'

    def getstatus(self):

        if int(self.airOSVersion[0]) >= 5:
            cmd = ("sshpass -p" + self.passwd + " ssh " + self.username +
                   "@" + self.ipaddr + " /sbin/ubntbox status")
        
        if int(self.airOSVersion[0]) >=8:
            cmd = ("sshpass -p" + self.passwd + " ssh " + self.username +
                   "@" + self.ipaddr + " /sbin/ubntbox status.cgi")
        # print(cmd)
        try:
            stream = os.popen(cmd)
            statustime = datetime.datetime.utcnow().timestamp()
          
            self.statusraw = stream.read()
            startidx = self.statusraw.find('{')
            self.status = json.loads(self.statusraw[startidx:])
            # print(self.status)
        except:
            print("Could not get status info.")
            self.statusraw = None
            self.status = None

    def getsignal(self):
        
        try:
            cmd = ("sshpass -p" + self.passwd + " ssh " + self.username +
                   "@" + self.ipaddr + " /sbin/ubntbox signal.cgi")
            print(cmd)
            stream = os.popen(cmd)
            self.signaltime = datetime.datetime.utcnow().timestamp()
            self.signalraw = stream.read()
            startidx = self.signalraw.find('{')
            print(self.signalraw[startidx:])
            self.signal = json.loads(self.signalraw[startidx:])
        except: 
            print("Could not get signal")
            self.signalraw = None
            self.signal = None

if __name__ == '__main__':
    
    U = UbiquitiWifi(ipaddr = '192.168.100.74')
    U.username = 'field'
    U.passwd = 'Chas3ocean!'
    U.getAirOSVersion()
    U.getstatus()
    print(U.status)

    ''' Fields to log:
    
    # Local radio information
    self.status['host']['hostname']
    self.status['host']['uptime']
    self.status['host']['time']
    self.status['host']['temperature']
    self.status['wireless']['frequency']
    self.status['wireless']['distance']
    self.status['wireless']['noisef']
    self.status['wireless']['txpower']
    self.status['wireless']['chanbw']
    self.status['wireless']['throughput']['tx']
    self.status['wireless']['throughput']['rx']
    self.status['wireless']['tx_use']
    self.status['wireless']['rx_use']
    
    # Distant radio information
    self.status['wireless']['sta'][0]['signal']
    self.status['wireless']['sta'][0]['rssi']
    self.status['wireless']['sta'][0]['noisefloor']
    self.status['wireless']['sta'][0]['distance']
    self.status['wireless']['sta'][0]['airmax']['uplink_capacity']
    self.status['wireless']['sta'][0]['airmax']['downlink_capacity']
    self.status['wireless']['sta'][0]['remote']['hostname']
    self.status['wireless']['sta'][0]['remote']['temperature']
    self.status['wireless']['sta'][0]['remote']['uptime']
    self.status['wireless']['sta'][0]['remote']['tx_throughput']
    self.status['wireless']['sta'][0]['remote']['rx_throughput']
    self.status['wireless']['sta'][0]['remote']['signal']
    self.status['wireless']['sta'][0]['remote']['rssi']    
    self.status['wireless']['sta'][0]['remote']['noisefloor']
    self.status['wireless']['sta'][0]['remote']['tx_power']
    self.status['wireless']['sta'][0]['remote']['distance']
    '''
    
