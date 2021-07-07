# -*- coding: utf-8 -*-
"""
Created on Wed Jul  7 13:50:10 2021

@author: Patrick
"""

import statistics
import math
from bluepy.btle import Scanner

# Print iterations progress
def printProgressBar (iteration, total, prefix = '', suffix = '', decimals = 1, length = 100, fill = '█', printEnd = "\r"):
    """
    Call in a loop to create terminal progress bar
    @params:
        iteration   - Required  : current iteration (Int)
        total       - Required  : total iterations (Int)
        prefix      - Optional  : prefix string (Str)
        suffix      - Optional  : suffix string (Str)
        decimals    - Optional  : positive number of decimals in percent complete (Int)
        length      - Optional  : character length of bar (Int)
        fill        - Optional  : bar fill character (Str)
        printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
    """
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filledLength = int(length * iteration // total)
    bar = fill * filledLength + '-' * (length - filledLength)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end = printEnd)
    # Print New Line on Complete
    if iteration == total: 
        print()


# Reads RSSI of a given MAC address
def readRSSI(mac, timeOut = 0.5):
    deviceList = Scanner().scan(timeOut)
    for device in deviceList:
        if device.addr == mac.lower():
            return device.rssi
    return None


def getMeanRSSI(mac, numSamples=10):
    print("Measuring RSSIs")
    rssiList = list()
    for i in range(numSamples):
        printProgressBar(i, numSamples)
        RSSI = None
        while RSSI == None:
            RSSI = readRSSI(mac)
        rssiList.append(RSSI)
    printProgressBar(numSamples,numSamples)
    rssi = statistics.mean(rssiList)
    rssiStdev = statistics.stdev(rssiList)
    return (rssi,rssiStdev) 


def getMeasuredPower(mac, numSamples = 1000):
    print("Getting Measured Power")
    return getMeanRSSI(mac, numSamples)
         
    
def getEnvironmentFactor(mac,measuredPower,distanceAvailable):
    print("Getting Environment Factor")
    
    environmentalFactorList = list()
    distance = 1.0
    distanceIncrement = 0.5
    while distance <= distanceAvailable:
        print('Place Beacon '+str(distance)+'m away')
        input('Press enter to continue:')
        
        rssi = getMeanRSSI(mac)
        environmentalFactorList.append((measuredPower - rssi) / (10 * math.log(distance,10)))
        distance = distance + distanceIncrement
    
    environmentalFactor = statistics.mean(environmentalFactorList)
    environmentalFactorStdev = statistics.stdev(environmentalFactorList)
    return(environmentalFactor,environmentalFactorStdev)


def calibrate():
    while True:
        mac = input('Enter MAC Address (including colons): ')
        distanceAvailable = input('How much space do you have in meters?: ')
        input('Place beacon 1m away and press enter to continue:')
        (measuredPower,measuredPowerStdev) = getMeasuredPower(mac)
        (environmentalFactor,environmentalFactorStdev) = getEnvironmentFactor(mac,measuredPower,distanceAvailable)
        print("-----------------------------")
        print("Measured Power: " + str(measuredPower))
        print("Measured Power stdev: " + str(measuredPowerStdev))
        print("Environment Factor: " + str(environmentalFactor))
        print("Measured Power stdev: " + str(environmentalFactorStdev))
        print("-----------------------------")

if __name__ == '__main__':
    calibrate()