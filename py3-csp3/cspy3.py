#python3
"""
cspy3.py version 0.1
####################

This program was written in order to communicate with and control an iRobot Create
in a variety of different ways-- directly from a computer or over SSH, for example.

It is neither complete nor perfect, but suffices for moving the Create around and 
getting access to certain sensors without having to write a separate program.

Usage:
	python cspy3.py <serial_port> <pckt_id_#1> [<pckt_id_#2>, [...]]

Example:
	#(Battery Information)
	python cspy3.py /dev/ttyUSB0 24 25 26 


--bab 2014 
"""

import array
import json
import numpy as np 
import os
import random
import select
import serial
import struct
import sys
import termios
import time
import tty

from time import sleep

# A partial enumeration of opcodes (necessary to command the Create to do things)
OP_PASSIVE = 128
OP_CONTROL = 130
OP_FULL = 132

OP_DRIVE = 145

OP_LEDS    = 139
OP_LSD     = 138
OP_PWM_LSD = 144


OP_BAUD = 129

OP_STREAM = 148
OP_PAUSE  = 150
OP_QUERY  = 142
OP_QUERY_LIST = 149

# The sensor packets are described in a dict, based on the Open Interface spec

PacketDct = \
{7: {'ValueRange': [0, 31],
  'dtype': 'B',
  'id': 7,
  'name': 'BumpsAndWheelDrops',
  'size': 1,
  'units': None},
 8: {'ValueRange': [0, 1],
  'dtype': 'B',
  'id': 8,
  'name': 'Wall',
  'size': 1,
  'units': None},
 9: {'ValueRange': [0, 1],
  'dtype': 'B',
  'id': 9,
  'name': 'CliffLeft',
  'size': 1,
  'units': None},
 10: {'ValueRange': [0, 1],
  'dtype': 'B',
  'id': 10,
  'name': 'CliffFrontLeft',
  'size': 1,
  'units': None},
 11: {'ValueRange': [0, 1],
  'dtype': 'B',
  'id': 11,
  'name': 'CliffFrontRight',
  'size': 1,
  'units': None},
 12: {'ValueRange': [0, 1],
  'dtype': 'B',
  'id': 12,
  'name': 'CliffRight',
  'size': 1,
  'units': None},
 13: {'ValueRange': [0, 1],
  'dtype': 'B',
  'id': 13,
  'name': 'VirtualWall',
  'size': 1,
  'units': None},
 14: {'ValueRange': [0, 31],
  'dtype': 'B',
  'id': 14,
  'name': 'Overcurrents',
  'size': 1,
  'units': None},
 15: {'ValueRange': [0, 0],
  'dtype': 'B',
  'id': 15,
  'name': 'Unused',
  'size': 1,
  'units': None},
 16: {'ValueRange': [0, 0],
  'dtype': 'B',
  'id': 16,
  'name': 'Unused',
  'size': 1,
  'units': None},
 17: {'ValueRange': [0, 255],
  'dtype': 'B',
  'id': 17,
  'name': 'IRByte',
  'size': 1,
  'units': None},
 18: {'ValueRange': [0, 15],
  'dtype': 'B',
  'id': 18,
  'name': 'Buttons',
  'size': 1,
  'units': None},
 19: {'ValueRange': [-32768, 32767],
  'dtype': 'h',
  'id': 19,
  'name': 'Distance',
  'size': 2,
  'units': None},
 20: {'ValueRange': [0, 5],
  'dtype': 'h',
  'id': 20,
  'name': 'Angle',
  'size': 2,
  'units': None},
 21: {'ValueRange': [0, 65535],
  'dtype': 'B',
  'id': 21,
  'name': 'ChargingState',
  'size': 1,
  'units': None},
 22: {'ValueRange': [-37268, 37268],
  'dtype': 'H',
  'id': 22,
  'name': 'Voltage',
  'size': 2,
  'units': None},
 23: {'ValueRange': [-128, 127],
  'dtype': 'h',
  'id': 23,
  'name': 'Current',
  'size': 2,
  'units': None},
 24: {'ValueRange': [0, 65535],
  'dtype': 'b',
  'id': 24,
  'name': 'BatteryTemperature',
  'size': 1,
  'units': None},
 25: {'ValueRange': [0, 65535],
  'dtype': 'H',
  'id': 25,
  'name': 'BatteryCharge',
  'size': 2,
  'units': None},
 26: {'ValueRange': [0, 4095],
  'dtype': 'H',
  'id': 26,
  'name': 'BatteryCapacity',
  'size': 2,
  'units': None},
 27: {'ValueRange': [0, 4095],
  'dtype': 'H',
  'id': 27,
  'name': 'WallSignal',
  'size': 2,
  'units': None},
 28: {'ValueRange': [0, 4095],
  'dtype': 'H',
  'id': 28,
  'name': 'CliffLeftSignal',
  'size': 2,
  'units': None},
 29: {'ValueRange': [0, 4095],
  'dtype': 'H',
  'id': 29,
  'name': 'CliffFrontLeftSignal',
  'size': 2,
  'units': None},
 30: {'ValueRange': [0, 4095],
  'dtype': 'H',
  'id': 30,
  'name': 'CliffFrontRightSignal',
  'size': 2,
  'units': None},
 31: {'ValueRange': [0, 4095],
  'dtype': 'H',
  'id': 31,
  'name': 'CliffRightSignal',
  'size': 2,
  'units': None},
 32: {'ValueRange': [0, 31],
  'dtype': 'B',
  'id': 32,
  'name': 'UserDigitalInputs',
  'size': 1,
  'units': None},
 33: {'ValueRange': [0, 1023],
  'dtype': 'B',
  'id': 33,
  'name': 'UserAnalogInput',
  'size': 2,
  'units': None},
 34: {'ValueRange': [0, 3],
  'dtype': 'B',
  'id': 34,
  'name': 'ChargingSourcesAvailable',
  'size': 1,
  'units': None},
 35: {'ValueRange': [0, 3],
  'dtype': 'B',
  'id': 35,
  'name': 'OIMode',
  'size': 1,
  'units': None},
 36: {'ValueRange': [0, 15],
  'dtype': 'B',
  'id': 36,
  'name': 'SongNumber',
  'size': 1,
  'units': None},
 37: {'ValueRange': [0, 1],
  'dtype': 'B',
  'id': 37,
  'name': 'SongPlaying',
  'size': 1,
  'units': None},
 38: {'ValueRange': [0, 42],
  'dtype': 'B',
  'id': 38,
  'name': 'NumberOfStreamPackets',
  'size': 1,
  'units': None},
 39: {'ValueRange': [-500, 500],
  'dtype': 'h',
  'id': 39,
  'name': 'Velocity',
  'size': 2,
  'units': None},
 40: {'ValueRange': [-32768, 32767],
  'dtype': 'h',
  'id': 40,
  'name': 'Radius',
  'size': 2,
  'units': None},
 41: {'ValueRange': [-500, 500],
  'dtype': 'h',
  'id': 41,
  'name': 'RightVelocity',
  'size': 2,
  'units': None},
 42: {'ValueRange': [-500, 500],
  'dtype': 'h',
  'id': 42,
  'name': 'LeftVelocity',
  'size': 2,
  'units': None}}


SERIAL_PARAMS = {"baudrate":    57600,
				 "timeout": 0,
				 "parity": serial.PARITY_NONE,
				 "bytesize":serial.EIGHTBITS}


def openPort(portname, configDct):
	""" Open a port with the requested properties, return a pyserial object """
	try:
		print("[DEBUG]: Opening port: {}".format(portname))
		ser = serial.Serial(portname, **configDct)
		# The below might not be necessary on some platforms
		ser.setRTS(0)
		time.sleep(0.25)
		ser.setRTS(1)
		time.sleep(0.5)
		ser.flushOutput()
		time.sleep(0.25)
	except Exception as e:
		print(e)
		ser = None

	return ser

def sendCmd(ser, cmdLst):
	tmp = struct.pack('B'*len(cmdLst), *cmdLst)
	try:
		print("[DEBUG]: Sending: {}".format(tmp))
		sent = ser.write(tmp)
		ser.flush()
		return sent
	except Exception as e:
		print(e)

def drive(ser, left, right):
	""" Drive a robot's (w/ pyserial object "ser") individual wheels """
	print("[DEBUG]: drive() called: {0} {1}".format(left, right))
	lval  = max(min(left,  500), -500)
	rval  = max(min(right, 500), -500)
	bytes = struct.unpack('4B',struct.pack('>2h', lval, rval))
	ret   = sendCmd(ser, [OP_DRIVE] + list(bytes))
	return ret

def setModePassive(ser):
	ret = sendCmd(ser, [OP_PASSIVE])
	return ret

def setModeFull(ser):
	ret  = sendCmd(ser, [OP_FULL])
	return ret

def pauseStream(ser):
	ret  = sendCmd(ser, [OP_PAUSE, 0])
	return ret

def setLEDs(ser, playOn=False, advOn=False, powColor=0, powIntensity=0):
	# Set up byte for the Advance and Play LEDs
	tmp = 0
	if playOn: tmp += 1
	if advOn:  tmp += 8
	# Limit the power LED intensities
	powIntensity = max(0, min(powIntensity, 255))
	powColor     = max(0, min(powColor, 255))
	# Build list and send the commands
	cmdLst = [OP_LEDS, tmp, powColor, powIntensity] 
	ret = sendCmd(ser, cmdLst)
	return ret

def lowSideDrivers(ser, d0=False, d1=False, d2=False):
	""" Set the low side drivers to be on (True) or off (False)
		d0 and d1 (corresponding to pins 22 & 23) have a maximum current
		of 0.5 A, d2 (pin 24) has a maximum current of 1.5 """
	# Build the byte to be sent to the Create
	tmp = 0
	if d0: tmp += 1
	if d1: tmp += 2
	if d2: tmp += 4
	cmdLst = [OP_LSD, tmp]
	ret = sendCmd(ser, cmdLst)
	return ret

def pwmLowSideDrivers(ser, pct0=0, pct1=0, pct2=0):
	""" Set up pulse-width modulation on the low side drivers by specifying
		the percentage of maximum power (w/ 7 bit resolution)."""
	# Get the integer representation of the desired power
	d0 = max(0, min(int(128 * pct0), 128))
	d1 = max(0, min(int(128 * pct1), 128))
	d2 = max(0, min(int(128 * pct2), 128))
	
	# Send the command 
	cmdLst = [OP_PWM_LSD, d0, d1, d2]
	print("[DEBUG]: pwmLowSideDrivers:", cmdLst)
	ret = sendCmd(ser, cmdLst)
	return ret


def requestStream(ser, sensorLst):
	length = len(sensorLst)
	cmdLst = [OP_STREAM, length] + sensorLst
	print(cmdLst)
	print(cmdLst)
	ret = sendCmd(ser, cmdLst)
	return ret

def shutdownRobot(ser):
	print("[DEBUG]: Shutting Down Robot")
	try:
		pauseStream(ser)
		sleep(1.5)
		ser.flushOutput()
		setModePassive(ser)
		sleep(1.5)
	except Exception as e:
		print("Oh no!")
		print(e)
		ser.flush()
		ser.flushOutput()
		ser.flushInput()
		ser.close()



def makeBytes(lst):
	return struct.pack("B"*len(lst), *lst)


(WAIT_HEADER, IN_MSG) = range(2)



class csp3():
	def __init__(self, sensorLst):
		print("[DEBUG]: csp3.__init__() called")
		self.numSensors  = len(sensorLst)
		self.packetInfo  = [PacketDct[i] for i in sensorLst]
		self.packetNames = [i["name"]  for i in self.packetInfo]
		self.packetTypes = [i["dtype"] for i in self.packetInfo]
		self.sizeLst     = np.array([i["size"] for i in self.packetInfo])
		print("[DEBUG]: Relevant sensor data loaded")
		
		# Formatter for converting bytes to different data types
		self.dataFormat  = ">" + "".join(self.packetTypes)


		# Total size is based on the number of sensors, the size 
		# of the data from the sensors, plus 3 bytes for checking integrity
		self.numBytes    = np.sum(self.sizeLst)
		self.totalSize   = len(sensorLst) + self.numBytes + 3

		self.firstByte   = 19

		# We want the indices where the sensor IDs (not their data) is located
		self.idIx        = np.cumsum(np.append(np.array(2), self.sizeLst + 1))[:-1]
		self.idMask      = np.in1d(np.arange(self.totalSize), self.idIx)
		# The indices where non-data (i.e., header, packet id, checksum) is located
		self.nonDataIx   = np.concatenate((np.array([0,1]), self.idIx, np.array([self.totalSize-1,])))
		# An index array for where non-data bytes appear
		self.nonDataMask = np.in1d(np.arange(self.totalSize - 1), self.nonDataIx)
		# The indices where data appears
		self.dataIx      = np.arange(self.totalSize - 1)[~self.nonDataMask]
		# Make an array of the checkbits, of size equal to total length of packet
		tmp 	 		 = np.zeros(self.totalSize)
		tmp[self.idMask] = np.array(sensorLst)
		self.packetCheck = tmp

		# Initialize the actual packet construction machinery
		self.lastPacket = []
		self.curPacket  = []
		self.count      = 0
		self.checksum   = 0
		self.state      = WAIT_HEADER

		# Timing the packets
		self.tick = 0
		self.tock = time.time()
		self.packetCount = 0
		self.avgTime = 0
		self.totalTime = 0

		print("[DEBUG]: csp3() finished initializing")

	def printParams(self):
		""" Print parameters (and much else) for the csp3 packet handler """
		print("Total size:",  self.totalSize)
		print("Packet info:", self.packetInfo)
		print("sizeLst:", self.sizeLst)
		print("idIx:", self.idIx)
		print("nonDataIx", self.nonDataIx)
		print("nonDataMask", self.nonDataMask)
		print("packetCheck", self.packetCheck)
		print("dataIx", self.dataIx)
		print("packetTypes", self.packetTypes)
		print("dataFormat", self.dataFormat)

	def inputLst(self, data):
		while len(data) > 0:
			b = data.pop(0)
			#print(self.count, b, self.checksum) # See the bytes as they come in!
			# Once we receive the header, begin constructing the packet
			if (self.state == WAIT_HEADER) and (b == self.firstByte):
				self.count += 1
				self.checksum += b
				self.state =  IN_MSG
				self.curPacket = [b]
			
			# While we receive the message, we need to check that it is well-formed
			elif self.state == IN_MSG:
				# If the count corresponds to an index packet, perform a check
				""" Although I haven't extensively tested, I find that
					we don't have to check all the check bytes in order to 
					ensure the packet is properly aligned """
				if self.count in self.idIx:
					if b != self.packetCheck[self.count]:
						print("\nPacket out of alignment:")
						print(self.curPacket, b, self.packetCheck[self.count])
						self.curPacket = []
						self.count = 0
						self.checksum = 0
						self.state = WAIT_HEADER
						continue

				# After perhaps performing checks, add to packet
				self.curPacket.append(b)
				self.checksum += b
				self.count += 1

			# Once we have enough bytes, try to form packet
			if self.count == self.totalSize:
				# Condition for complete packet
				if ((self.checksum % 256) == 0): # Success!
					#print("Packet:", self.decoder(self.curPacket))
					#print("\r", self.wrapper(self.decoder(self.curPacket)), end = "")
					print(self.wrapper(self.decoder(self.curPacket)))
					self.lastPacket = list(self.curPacket) # Improve this
				else:
					print("Misaligned packet:", self.curPacket)
				
				# Either way, reset the packet
				self.curPacket  = []
				self.count      = 0
				self.checksum   = 0
				self.state      = WAIT_HEADER
				


	def decoder(self, packet): 
		""" Assuming the packet is well formed, convert it to bytes,
			and then format according to the specification """
		# Get the data bytes from the packet, via numpy tricks
		tmp = np.array(packet)[~self.nonDataMask]
		#print(bytes(packet))
		#print(packet)
		#print(tmp)

		# Timing for comparison
		self.packetCount += 1
		self.tick = time.time()
		tmpTime = (self.tick - self.tock)
		self.totalTime += tmpTime
		self.avgTime = self.totalTime/self.packetCount
		self.tock = self.tick
		#print("Last Packet Time: {1:2.8f} \t Avg Time: {0:2.8f}".format(self.avgTime, tmpTime))
		
		tmp = struct.pack("B"*self.numBytes, *tmp)
		ret = struct.unpack(self.dataFormat, tmp)
		return ret
		

	def wrapper(self, tup):
		tmp = {self.packetNames[i]: tup[i] for i in range(self.numSensors)}
		return tmp 


def main():
	# Specify the port from command line
	if len(sys.argv) <= 1:
		print("Specify serial port, por favor")
		exit()
	elif len(sys.argv) > 2: 
		packets = [int(i) for i in sys.argv[2:]]
	else:
		packets = [24, 25, 26]  # Arbitrary choices of packets

	#oldCWD = os.getcwd()
	#os.chdir(os.chdir(os.path.dirname(sys.argv[0])))
	# with open("SensorPackets.json", "r") as f:
	# 	tmpDct = json.load(f)
	# 	# Handle the fact that JSON cannot have integer keys
	# 	global PacketDct
	# 	PacketDct = {int(k):v for k, v in tmpDct.items()}
	#os.chdir(oldCWD)


	port  = sys.argv[1]  
	ser   = openPort(port, SERIAL_PARAMS)
	serfd = ser.fileno()
	iofd  = sys.stdin.fileno()
	print("[DEBUG]: connecting to port: {}".format(port))

	try:
		# Set up the robot
		setModePassive(ser)
		setModeFull(ser)
		handler = csp3(packets) # Set up the handler
		handler.printParams()   # Print some information about handler
		requestStream(ser, packets)

		tick = time.time()

		SPEED_LEFT  = 200
		SPEED_RIGHT = 200
		on_w_press  = lambda : drive(ser,  SPEED_LEFT,  SPEED_RIGHT)
		on_s_press  = lambda : drive(ser, -SPEED_LEFT, -SPEED_RIGHT)
		on_a_press  = lambda : drive(ser,  SPEED_LEFT, -SPEED_RIGHT)
		on_d_press  = lambda : drive(ser, -SPEED_LEFT,  SPEED_RIGHT)
		on_x_press  = lambda : drive(ser,    0,    0)
		on_l_press  = lambda : setLEDs(ser, playOn=True,  advOn=True,  powIntensity=0, powColor=0)
		on_o_press  = lambda : setLEDs(ser, playOn=False, advOn=False, powIntensity=0, powColor=0)
		on_p_press  = lambda : pwmLowSideDrivers(ser, 1, 1, 1)
		on_k_press  = lambda : pwmLowSideDrivers(ser, 0, 0 ,0)

		# The keymap
		keymap = {"w": on_w_press, 
				  "s": on_s_press,
				  "a": on_a_press,
				  "d": on_d_press,
				  "x": on_x_press,
				  "l": on_l_press,
				  "o": on_o_press,
				  "p": on_p_press,
				  "k": on_k_press}

		# Where we're going, we don't need line buffering
		old_stdin_settings = termios.tcgetattr(sys.stdin) 
		tty.setcbreak(sys.stdin.fileno())


		print("[DEBUG]: about to begin loop")
		while True:
			# Set up the select
			sel = select.select([serfd, iofd], [], [], 0)
			if serfd in sel[0]:
				# Essentially, wait and read. 
				numWait = ser.inWaiting()
				data    = ser.read(numWait)
				handler.inputLst(list(data))
				tock = time.time()
				#print(tock - tick)
				tick = tock
			if iofd in sel[0]:
				key = sys.stdin.read(1) # Read the key
				key = key[0]      # Truncate it
				
				
				if key in keymap:
					keymap[key]()
					print(key)
			else:
				pass

			#time.sleep(0.1)
	
	# Perform exceptional exception handling
	except Exception as e:
		print(e)

	# Ensure that the robot is shut down properly
	finally:
		# Restore the terminal
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_stdin_settings)

		# Shutdown the robot
		shutdownRobot(ser)
		ser.close()
		sleep(0.5) # Superstition, because OS X serial hangs require a reboot 



if __name__ == "__main__":
	main()