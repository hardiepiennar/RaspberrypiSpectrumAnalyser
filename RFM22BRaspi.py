#!/usr/bin/python
#Raspberry pi spectrum analyzer function
#Hardie Pienaar - 5 Nov 2013
#
#This code will interface to the RFM22B board via SPI and deliver functionality for a simple spectrum analyzer
#
#Note:
#1 - The spi needs to be enabled on the raspberry pi by commenting out (with a # tag) the blacklist spi-bmc2708 line in the file 
#    /etc/modprobe.d/raspi-blacklist.conf. This then needs to be followed by a reboot
#2 - Make sure the SPI wrapper and pyhton dev is installed, if not it can be installed with the following commands
#     sudo apt-get install python-dev
#     mkdir python-spi
#     wget https://raw.github.com/doceme/py-spidev/master/setup.py
#     wget https://raw.github.com/doceme/py-spidev/master/spidev_module.c
#     sudo python setup.py install
#    Copy the created file to your script directory /build/lib.linux-armv6l-2.7

import sys
import numpy as np		#For all the number crunching
import matplotlib.pyplot as plt #For the nice plots
import time as t		#For keeping time between events
import csv as csv		#For writing to files
import RPi.GPIO as GPIO		#For working with GPIO header
import spidev as spidev		#For working with spi channels


#Global variables
#Port allocations
LEDPort = 18
NIRQPort = 25
TXAntPort = 24
RXAntPort = 23
GPIO0Port = 17
GPIO1Port = 27
GPIO2Port = 22

#IF Filter lookup table
IFFilter = [2600,2800,3100,3200,3700,4200,4500,4900,5400,5900,6100,7200,8200,8800,9500,10600,11500,12100,14200,16200,17500,18900,21000,22700,24000,28200,32200,34700,37700,41700,45200,47900,56200,64100,69200,75200,83200,90000,95300,112100,127900,137900,142800,167800,181100,191500,225100,248800,269300,284900,335500,361800,420200,469400,518800,577000,620700]
IFFilterNDecExp = [5,5,5,5,5,5,5,4,4,4,4,4,4,4,3,3,3,3,3,3,3,2,2,2,2,2,2,2,1,1,1,1,1,1,1,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0]
IFFilterDwn3Bypass = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
IFFilterFilset = [1,2,3,4,5,6,7,1,2,3,4,5,6,7,1,2,3,4,5,6,7,1,2,3,4,5,6,7,1,2,3,4,5,6,7,1,2,3,4,5,6,7,4,5,9,15,1,2,3,4,8,9,10,11,12,13,14]

#Initializes GPIO header pins
def initializeGPIO():
	#Turn off GPIO warnings
	GPIO.setwarnings(False)

	#Set GPIO mode
	GPIO.setmode(GPIO.BCM)

	#RFM22B GPIO Ports needs to be allocated below
	#Set output ports
	GPIO.setup(LEDPort,GPIO.OUT)
	GPIO.setup(TXAntPort,GPIO.OUT)
	GPIO.setup(RXAntPort,GPIO.OUT)	

	#Set input ports
	GPIO.setup(NIRQPort,GPIO.IN)



#Turns LED on or off according to given state
#state = True:    LED will turn on
#state = False:   LED will turn off
def setLED(state):
	if state:
		#print("Turning LED on")
		GPIO.output(LEDPort,True)
	else:
		#print("Turning LED off")
		GPIO.output(LEDPort,False)

#Flashes the LED at the given period in seconds
def flashLED(period):
	setLED(True)
	t.sleep(period/2)
	setLED(False)
	t.sleep(period/2)

#Sets up the RFM22B to a initialized state
#Returns the spi communication handle
def initializeRFM22B(verbose=True,channel=0):
	if(verbose): print("=============RFM22B device initialization===========")	
	if(verbose): print("Assuming device was on")
	
	if(verbose): print("Setting device to RX State")
	#Put the RFM22B in a RX State
	GPIO.output(TXAntPort,False)
	GPIO.output(RXAntPort,True)
	t.sleep(0.1) #Wait for device to assume state

	if(verbose): print("Initializing SPI communication")
	#Initialize SPI channel
	spi = spidev.SpiDev() 	#Creates spi object
	spi.open(0,channel)		#Opens channel with CE0 as enable port

	if(verbose): print("Turning on PLL and entering tune mode ")
	#Turn on PLL to enter tune mode
	writeRFM22BRegister(spi,0x07,0b00000110)

	if(verbose): print("Turning off all gains")
	#Turn off agc and set LNA gain to 5dB minimum and pga gain to 0
	writeRFM22BRegister(spi,0x69,0b00000000)
	
	if(verbose): print("====================================================")	

	return spi		#Return a handle to the spi communication handle

#Returns the value at a specific address on the RFM22B
#address must be specified in hex eg. 0x0A
#spi is the spi communication channel which must be configured beforehand
def readRFM22BRegister(spi,address):		
	resp = spi.xfer2([address,0x00]) 	
	return resp[1]

#Writes the given data to the given address on the RFM22B
#address must be specified in hex eg. 0x0A
#spi is the spi communication channel which must be configured beforehand
#data is the 8 bit data in hex format that needs to be stored at the address
def writeRFM22BRegister(spi,address,data):
	#Add write bit to address
	address += 128
	#Send adress and data to be written
	resp = spi.xfer2([address,data])

#Prints out the current status of the RFM22B module
#If verbose is true the output will be written to the console
#Returns True if module is working
#Returns False if module is not working
def checkRFM22BStatus(spi,verbose=True):
	if(verbose): print("==============RFM22B device status check============")	

	#Read the device type,version and status
	deviceType = readRFM22BRegister(spi,0x00)
	deviceVersion = readRFM22BRegister(spi,0x01)
	deviceStatus = readRFM22BRegister(spi,0x02)

	#Check if the device type is read correctly, this will determine if communication is and the device is working
	if(deviceType != 8): #Check if device is not of tranceiver type or if communication is broken
		if(verbose): print("Device is not of tranceiver type or SPI communication channel is faulty")
		return False
	else:
		if(verbose): print("Tranceiver device detected")

	#Check device version code
	if(deviceVersion == 1):
		if(verbose): print("Version: X4")
	elif(deviceVersion == 2):
		if(verbose): print("Version: V2")
	elif(deviceVersion == 3):
		if(verbose): print("Version: A0")
	else:
		if(verbose): print("Version: Unknown ("+str(deviceVersion)+")")
	
	#Check device state
	deviceStatus = deviceStatus%4 #Leave only device state bits
	if(deviceStatus == 0):
		if(verbose): print("Status: Idle")
	elif(deviceStatus == 1):
		if(verbose): print("Status: RX State")
	elif(deviceStatus == 2):
		if(verbose): print("Status: TX State")
	else:
		if(verbose): print("Status: Unknown")

	if(verbose): print("====================================================")
	
	return True
			

#Sets up the RFM22B registers to listen or transmit at the given frequency
#Returns True if the command was succesfull
#freq is specified in MHz
def setFrequency(spi,freq):
	#Check if frequency is in range
	if(freq > 960 or freq < 240):
		return False

	#Some variables that will be used
	bandSelect = 0b01000000
	fb = 0b00000000
	fc = 0
	xtalFreq = 30000
	
	#Calculate register according to formula found in excel sheet
	if(freq >= 480):
		bandSelect |= 0b00100000
		temp = freq/(10*(xtalFreq/30000)*(1+1))
	else:
		temp = freq/(10*(xtalFreq/30000)*(1+0))
	fb = int(np.floor(temp)) - 24
	fc = int(np.floor((temp - np.floor(temp))*64000+0.49999))
	
	bandSelect |= fb
	fc1 = fc>>8
	fc2 = fc&0b0000000011111111
	
#	print(bandSelect)
#	print(fc1)
#	print(fc2)
	#Write the calculated registers to the device
	writeRFM22BRegister(spi,0x75,bandSelect)
	writeRFM22BRegister(spi,0x76,fc1)
	writeRFM22BRegister(spi,0x77,fc2)

	#print("Freq Register "+str(readRFM22BRegister(spi,0x76)))
		
	return True
#Sets the IF filter register for the specified bandwidth by using a lookup table
#rbw is the bandwidth in Hz
#returns True if filter was found in table and set
def setRBW(spi,rbw):
	#Run through list to find IF chosen index
	index = -1
	for i in np.arange(0,len(IFFilter)):
		if(int(rbw) == int(IFFilter[i])):
			index = i

	#Return Flase if rbw was not found in list
	if index < 0:
		print("Failed to set RBW")
		return False

	#Construct registers out of lookup table
	ifReg = 0b00000000
	ifReg |= (IFFilterDwn3Bypass[index]<<7)
	ifReg |= (IFFilterNDecExp[index]<<4)
	ifReg |= (IFFilterFilset[index])

	#Write registers to RFM22B
	writeRFM22BRegister(spi,0x1C,ifReg)
	
	#Check
	#print("IF Register "+str(readRFM22BRegister(spi,0x1C)))
	return True		

#Sets the gains in the RFM22B by using a lookup table for the pga gains and a boolean for the LNA gain
#Returns True if the lookup table entry was found
def setGain(spi,lna):
	lnaGain = -1
	pgaGain = 0	#TODO: PGA Gain is not yet implemented
	
	#set LNA to 25dB:
	if lna:
		lnaGain = 1
	else: #Set LNA to 5dB
		lnaGain = 0

	#Create register value
	gainReg = 0b00000000
	gainReg |= lnaGain<<4
	gainReg |= pgaGain
	
	#Write register
	writeRFM22BRegister(spi,0x69,gainReg)
	#print("Gain Register "+str(readRFM22BRegister(spi,0x69)))
	return True

#Reads, scales and calibrates the RSSI level from the RFM22B
#dwell specifies the amount of ms to measure
def getRSSI(spi,dwell):
	maxValue = 0
	minValue = 300
	rawValue = 0
	averageCount = 0

	#Start averaging loop until dwell time is reached
	startTime = t.time()
	while t.time() - startTime <= dwell/1000.0 or averageCount == 0:		
		#Read 8 bit value from RFM22B
		value = readRFM22BRegister(spi,0x26)
	
		#Store the minimum and maximum values
		if(value > maxValue):
			maxValue = value	
		if(value < minValue):
			minValue = value

		#build average
		rawValue += value
		averageCount += 1
 	rawValue = rawValue/averageCount
		
	#Scale value to dBm
	unCalRSSIAve = 0.5*rawValue - 120
	unCalRSSIMax = 0.5*maxValue - 120
	unCalRSSIMin = 0.5*minValue - 120

	#Apply calibration
	#TODO
	RSSIAve = unCalRSSIAve
	RSSIMax = unCalRSSIMax
	RSSIMin = unCalRSSIMin	

	return RSSIMin, RSSIAve, RSSIMax, averageCount

################################################# Start of main program
#print("-----------------EMURaspi starting--------------------")

#print("Initializing...")
#initializeGPIO()
#print("Initializing Radio "+str(sys.argv[3]))
#spi = initializeRFM22B(False,int(sys.argv[3]))
#checkRFM22BStatus(spi)


#print("Setting Frequency to "+str(sys.argv[1])+" MHz")
#setFrequency(spi,float(sys.argv[1]))
#print("Setting RBW to "+str(sys.argv[2])+" Hz")
#setRBW(spi,int(sys.argv[2]))
#print("Turning on LNA")
#setGain(spi,True)
#while True:
#	print(getRSSI(spi,20))
#	flashLED(0.25)


#print("-----------------EMURaspi stopping--------------------") 

 
 	
