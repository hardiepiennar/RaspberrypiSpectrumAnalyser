#!/usr/bin/pythonSerated 

import threading
import time
import os,struct,sys
import RFM22BRaspi as rfm


#Threading switches, used to turn threads off at the end
#of their respective loops
flightLoggerRunning = True
sensorLoggerRunning = True
guiRunning = True
mainThreadRunning = True

#Mutexes
flightDataMutex = threading.Lock()
sensorDataMutex = threading.Lock()

#Thread sleep times in seconds
flightLoggerSleep = 0.001
sensorLoggerSleep = 0.001
guiSleep = 3
mainLoopSleep = 0.001

#Flight Data Variables
missionSeq = -999
batVoltage = -999
batCurrent = -999
batRemaining = -999

roll = -999
pitch = -999
yaw = -999

heading = -999
barAlt = -999
pressure = -999
temp = -999
	
noOfSats = -999
gpsFix = -999
gpsAlt = -999
lat = -999
lon = -999
timeStamp = -999

#Sensor Data Variables
level0Max = -999
level0Min = -999
level0Ave = -999
level1Max = -999
level1Min = -999
level1Ave = -999

#Control switches
verboseThreading = False
verboseAPM = False
showGUI = True
newDataAvailable = False

#Sensor settings
frequency = 430.0
rbw = 21000
preAmp = True
dwell = 100

#File settings
filename = "KAPBRoom1430.csv"

#################################### Flight Logger Thread

def FlightLogger():
	global flightLoggerRunning	

	#Link global variables
	global missionSeq
	global batVoltage
	global batCurrent
	global batRemaining

	global roll
	global pitch
	global yaw

	global heading
	global barAlt
	global pressure
	global temp 	
	
	global noOfSats
	global gpsFix 
	global gpsAlt 
	global lat 
	global lon
	global timeStamp

	#Setup Connection with APM
	'''
	Backend of uav data collection.
	This class will interface via a serial link on a MAVLink protocol 
	to a UAV drone to extract the neccesary data
	and make it available for external use.
	This class will make extensive use of the already existing pymavlink software.

	'''

	'''
	set stream rate on an APM 
	'''
	if(verboseAPM): print("Setting up connection to APM")
	# allow import from the parent directory, where mavlink.py is
	sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

	from optparse import OptionParser
	parser = OptionParser("apmsetrate.py [options]")

	parser.add_option("--baudrate", dest="baudrate", type='int',
			  help="master port baud rate", default=115200)
	parser.add_option("--device", dest="device", default=None, help="serial device")
	parser.add_option("--rate", dest="rate", default=4, type='int', help="requested stream rate")
	parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
			  default=255, help='MAVLink source system for this GCS')
	parser.add_option("--showmessages", dest="showmessages", action='store_true',
			  help="show incoming messages", default=False)
	(opts, args) = parser.parse_args()

	opts.device = "/dev/ttyACM0"
	#opts.device = "/dev/ttyUSB0"

	import mavutil

	if opts.device is None:
	    if(verboseAPM): print("You must specify a serial device")
	    sys.exit(1)

	def wait_heartbeat(m):
	    '''wait for a heartbeat so we know the target system IDs'''
	    if(verboseAPM): print("Waiting for APM heartbeat")
	    m.wait_heartbeat()
	    if(verboseAPM): print("Heartbeat from APM (system %u component %u)" % (m.target_system, m.target_system))

	def get_messages(m):
	    '''show incoming mavlink messages'''
	    while True:
		msg = m.recv_match(blocking=True)
		if not msg:
		    return 'NULL'
		if msg.get_type() == "BAD_DATA":
		    if mavutil.all_printable(msg.data):
			sys.stdout.write(msg.data)
			sys.stdout.flush()
		else:
		    return msg                    
			
	# create a mavlink serial instance
	master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate)

	# wait for the heartbeat msg to find the system ID
	wait_heartbeat(master)

	if(verboseAPM): print("Sending all stream request for rate %u" % opts.rate)
	for i in range(0, 3):
	    master.mav.request_data_stream_send(master.target_system, master.target_component,
			                        mavutil.mavlink.MAV_DATA_STREAM_ALL, opts.rate, 1)

	if(verboseAPM): print("Flight logger thread started")
	
	while(flightLoggerRunning):
		flightDataMutex.acquire()

		if(verboseAPM): print("Starting parameter read")
	    	message = str(get_messages(master))
		#Check type of message
		if('SCALED_PRESSURE' in message):
			if(verboseAPM): print("Scaled pressure packet received")
			data = message.split(':')
			pressure = float(data[2].split(',')[0])
			temp = int(data[4].split('}')[0])			
		elif('VFR_HUD' in message):
			if(verboseAPM): print('VFR Hud packet received')
			data = message.split(':')
			barAlt = float(data[5].split(',')[0])
			heading = int(data[3].split(',')[0])		
			
		elif('ATTITUDE' in message):
			if(verboseAPM): print('Attitude packet received')
			data = message.split(':')
			roll = float(data[2].split(',')[0])
			pitch = float(data[3].split(',')[0])
			yaw = float(data[4].split(',')[0])			
		elif('GLOBAL_POSITION_INT' in message):
			if(verboseAPM): print('GPS packet received')
			data = message.split(':')
			timeStamp = int(data[1].split(',')[0])
			lat = float(data[2].split(',')[0])
			lon = float(data[3].split(',')[0])
			gpsAlt = float(data[4].split(',')[0])				
		elif('GPS_RAW_INT ' in message):
			if(verboseAPM): print('Raw gps packet received')
			data = message.split(':')
			noOfSats = int(data[10].split('}')[0])
			gpsFix = int(data[2].split(',')[0])
		elif('MISSION_CURRENT' in message):
			if(verboseAPM): print('Mission current packet received')
			data = message.split(':')
			missionSeq = int(data[1].split('}')[0])
		elif('SYS_STATUS' in message):
			if(verboseAPM): print('System status packet received')
			data = message.split(':')
			batVoltage = int(data[5].split(',')[0])
			batCurrent = int(data[6].split(',')[0])
			batRemaining = int(data[7].split(',')[0])
		else:
			if(verboseAPM): print('Unknown packet received')
		flightDataMutex.release()
		
		#Rest thread
		time.sleep(flightLoggerSleep)
		
		
	#Close down serial link wheb thread exits
	master.close()
	
	if(verboseAPM): print("Flight logger thread stopped")

#################################### End of Flight Logger Thread

#################################### Sensor Logger Thread

def SensorLogger():
	#Initialize sensor logger global variables
	global level0Max
	global level0Min
	global level0Ave
	global level1Max
	global level1Min
	global level1Ave
	global sensorLoggerRunning
	global frequency
	global newDataAvailable

	if(verboseThreading): print("Connecting to sensor")
	rfm.initializeGPIO()
	spi0 = rfm.initializeRFM22B(False,0)
	spi1 = rfm.initializeRFM22B(False,1)	

	#Check and handle a non connection
	if(not rfm.checkRFM22BStatus(spi0) or not rfm.checkRFM22BStatus(spi1)):
		print("Connection to sensor failed")
		killProgram()
	rfm.setFrequency(spi0,frequency)
	rfm.setFrequency(spi1,frequency)
	rfm.setRBW(spi0,rbw)
	rfm.setRBW(spi1,rbw)
	rfm.setGain(spi0,preAmp)
	rfm.setGain(spi1,preAmp)
	
	if(verboseThreading): print("Sensor logger thread started")

	while(sensorLoggerRunning):
		rfm.setLED(True)					
		sensorDataMutex.acquire()
		level0Min,level0Ave,level0Max,noOfMeas = rfm.getRSSI(spi0,dwell)
		level1Min,level1Ave,level1Max,noOfMeas = rfm.getRSSI(spi1,dwell)	
		newDataAvailable = True
		sensorDataMutex.release()
		rfm.setLED(False)
		#Rest thread	
		time.sleep(sensorLoggerSleep)

	if(verboseThreading): print("Sensor logger thread stopped")

#################################### End of Sensor Logger Thread		

#################################### GUI Thread
def GUI():
	global guiRunning
	
	allDataAvailable = False

	if(verboseThreading): print("GUI thread started")

	while(guiRunning):

		if(showGUI):
			#Clear screen
			#os.system('clear')

			#Print out flight controller data		
			flightDataMutex.acquire()
			if(allDataAvailable):
				print("#####Flight Controller Data#####")
				print("Position:")
				print("Time Stamp:   "+str(timeStamp))
				print("No Of Sats:   "+str(noOfSats))
				print("GPS Fix:      "+str(gpsFix))
				print("Lat:          "+str(lat))
				print("Lon:          "+str(lon))
				print("GPS Alt:      "+str(gpsAlt))
				print("Baro Alt:     "+str(barAlt))
				print("Heading:      "+str(heading))
				print("Orientation:\n")
				print("Roll:         "+str(roll))
				print("Pitch:        "+str(pitch))
				print("Yaw:          "+str(yaw))
				print("Other:\n")
				print("Abs Pressure: "+str(pressure))
				print("Temperature:  "+str(temp))
				print("MissionSeq:   "+str(missionSeq))
				print("Bat Voltage:  "+str(batVoltage))
				print("Bat Remaining:"+str(batRemaining))
				print("Bat Current:  "+str(batCurrent))
				print("\n#####Sensor Data#####")
				print("RSSI 0 Max:     "+str(level0Max))
				print("RSSI 0 Average: "+str(level0Ave))
				print("RSSI 0 Min:     "+str(level0Min))
				print("RSSI 1 Max:     "+str(level1Max))
				print("RSSI 1 Average: "+str(level1Ave))
				print("RSSI 1 Min:     "+str(level1Min))
				
			else:
				allDataAvailable = True
				print("Waiting for the following packets:")
				if(timeStamp == -999):
					print("Global position")
					allDataAvailable = False
				if(noOfSats == -999):
					print("GPS Raw")
					allDataAvailable = False
				if(pressure == -999):
					print("Scaled Pressure")
					allDataAvailable = False
				if(barAlt == -999):
					print("VFR HUD")
					allDataAvailable = False
				if(roll == -999):
					print("Attitude")
					allDataAvailable = False
				if(missionSeq == -999):
					print("Mission Current")
					allDataAvailable = False
				if(batVoltage == -999):
					print("System Status")
					allDataAvailable = False
				if(level0Max == -999):
					print("Sensor Data")
					allDataAvailable = False
			flightDataMutex.release()
				
		#Rest thread
		time.sleep(guiSleep)
				
	if(verboseThreading): print("GUI thread stopped")

#################################### End of GUI Thread

#################################### General functions

#Store current available data in log file TODO:
def storeCurrentData():
	print("Storing current data")

def killProgram():
	global flightLoggerRunning
	global sensorLoggerRunning
	global guiRunning
	global verboseThreading
	global sLThread
	global fLThread
	global guiThread
	global file

	#Send stop signal to all threads
	if(verboseThreading): print("Stopping all threads")
	flightLoggerRunning = False
	sensorLoggerRunning = False
	guiRunning = False

	#Wait for threads to stop befor doing final operations
	if(verboseThreading): print("Waiting for all threads to stop")
	while(sLThread.isAlive() or fLThread.isAlive() or guiThread.isAlive()):
		if(verboseThreading): print("Still waiting for threads to stop...")	
		time.sleep(0.5)
	if(verboseThreading): print("All threads stopped")

	#Close file
	file.close()

	#Shut down program
	print("Exiting program")
	sys.exit()



#################################### End of General function

#Create main threads	
if(verboseThreading): print("Creating main threads")
fLThread = threading.Thread(target = FlightLogger)
sLThread = threading.Thread(target = SensorLogger)
guiThread = threading.Thread(target = GUI)	

#Start main threads

if(verboseThreading): print("Starting all threads")
fLThread.start()
sLThread.start()
guiThread.start()

#Run main program
if(verboseThreading): print("Setting up logging file")
file = open(filename,'w')

#Write file header
file.write("fEMu 2.0 flight signal file\n")
file.write("Frequency [MHz]:, "+str(frequency)+"\n")
file.write("RBW [Hz]:, "+str(rbw)+"\n")
file.write("Dwell [ms]:, "+str(dwell)+"\n")
file.write("time,No of sats,GPS fix,Lat,Lon,GPS Alt,Barometer Alt,Heading,Roll,Pitch,Yaw,Abs Pressure,Temperature,Mission Seq,Battery Voltage,Battery Remaining,Battery Current,RSSI 0 Max,RSSI 0 Ave,RSSI 0 Min,RSSI 1 Max,RSSI 1 Ave, RSSI 1 Min\n")

if(verboseThreading): print("Starting main thread")
try:
	while(mainThreadRunning):
		#Check if new data is available 
		if(newDataAvailable):
			flightDataMutex.acquire()
			sensorDataMutex.acquire() 
			file.write(str(timeStamp)+","+str(noOfSats)+","+str(gpsFix)+","+str(lat)+","+str(lon)+","+str(gpsAlt)+","+str(barAlt)+","+str(heading)+","+str(roll)+","+str(pitch)+","+str(yaw)+","+str(pressure)+","+str(temp)+","+str(missionSeq)+","+str(batVoltage)+","+str(batRemaining)+","+str(batCurrent)+","+str(level0Max)+","+str(level0Ave)+","+str(level0Min)+", "+str(level1Max)+","+str(level1Ave)+","+str(level1Min)+"\n")
			newDataAvailable = False
			flightDataMutex.release()
			sensorDataMutex.release() 
 

		#Give an error if a system could not connect
		#Attempt a reconnect after a disconnection
		#Only start logging if all systems are on and connected
		#Check if any new data is available, if so, store in log
		#Implement sensor logger
		time.sleep(mainLoopSleep)
except(KeyboardInterrupt,SystemExit):
	print("Keyboard Interrupt")
	killProgram()
