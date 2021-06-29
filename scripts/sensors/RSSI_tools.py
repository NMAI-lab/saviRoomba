#!/usr/bin/env python

import os
import math
from bluepy.btle import Scanner

class RSSI_Tools:
    def __init__(self):
        self.__measured_power = -59
        self.__enviromental = {}
        self.__size=10

    '''
        Reads RSSI of a given MAC address
    '''
    def __read_RSSI(self,MAC):
        ble_list = Scanner().scan(0.5)
        for dev in ble_list:
            #print(dev.addr)
            if dev.addr == MAC.lower():
                return dev.rssi
        return None


    '''
        returns the distance of beacon
    '''
    def get_enviromental(self,MAC):
        self.__calobrate_enviromental(MAC)



    def get_mean_RSSI(self,MAC):
        sum = 0
        samples = 10
        rng = 10
        for i in range(rng):
            RSSI = self.__read_RSSI(MAC)
            if RSSI is None:
                samples = samples - 1
                continue
            sum = sum + RSSI
            #print(sum)
        if samples == 0:
            return 0
        else:
            return sum / samples



    def __calobrate_enviromental(self,MAC):
        sum_of_n = 0
        for i in range(2,self.__size + 1):
            print('Place Beacon '+str(i)+'m away')
            input('Press enter to continue:')
            RSSI = self.get_mean_RSSI(MAC)
            #print(RSSI)
            sum_of_n = sum_of_n + (self.__measured_power - RSSI)/(10*math.log(i,10))
        self.__enviromental[MAC] = sum_of_n/(self.__size)
        print("Enviromental Factor: " + str(sum_of_n/(self.__size)))
        
        
    '''
        Calculates the measured power at 1m
    '''   
    def calculate_measured(self,MAC):
        sum = 0
        samples = 1000
        rng = 1000
        for i in range(rng):
            RSSI = self.__read_RSSI(MAC)
            if RSSI is None:
                samples = samples - 1
                continue
            sum = sum + RSSI
            
        mean = sum / samples
        self.__measured_power = mean
        print(mean)
    
    
    '''
        Reads addr of a given MAC address
    '''
    def read_RSSI(self,MAC):
        ble_list = Scanner().scan(1)
        for dev in ble_list:
            if dev.addr == MAC.lower():
                print(dev.addr)
        return None

    def set_size(self,n):
        self.__size = int(n)
        
        
    def set_power(self,n):
        self.__measured_power = int(n)
        
tools = RSSI_Tools()

while True:
    mac = input('Enter MAC Address: ')
    tools.set_size(input('How much space do you have in meters?: '))
    inp = input('Calculate measured value? y or n : ')
    if inp == 'y':
        input("Place at 1m then press enter")
        tools.calculate_measured(mac)
    else:
        tools.set_power(input("What is 1m measured power? (default -59) : "))
    tools.get_enviromental(mac)



