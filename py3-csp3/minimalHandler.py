import array
import json
import numpy as np 
import select
import serial
import struct
import sys
import time

from time import sleep

# Neat idea: specify the opcode followed by the keyword arguments
# needed so that a function factory can make easily used functions
# for each command that you want to send to the Create

OP_PASSIVE = 128
OP_CONTROL = 130
OP_FULL = 132

OP_BAUD = 129

OP_STREAM = 148
OP_PAUSE  = 150
OP_QUERY  = 142
OP_QUERY_LIST = 149


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

def setModePassive(ser):
	ret = sendCmd(ser, [OP_PASSIVE])
	return ret

def setModeFull(ser):
	ret = sendCmd(ser, [OP_FULL])
	return ret

def pauseStream(ser):
	ret = sendCmd(ser, [OP_PAUSE, 0])
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

with open("SensorPackets.json", "r") as f:
	tmpDct = json.load(f)
	# Handle the fact that JSON cannot have integer keys
	PacketDct = {int(k):v for k, v in tmpDct.items()}


class csp3():
	def __init__(self, sensorLst):
		self.firstByte   = 19
		self.packetInfo  = [PacketDct[i] for i in sensorLst]
		self.sizeLst     = np.array([i["size"] for i in self.packetInfo])
		self.numBytes    = np.sum(self.sizeLst)
		self.packetTypes = [i["dtype"] for i in self.packetInfo]
		self.dataFormat  = ">" + "".join(self.packetTypes)

		# Total size is based on the number of sensors, the size 
		# of the data from the sensors, plus 3 bytes for checking integrity
		self.totalSize  = len(sensorLst) + self.numBytes + 3

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
				# if self.count in self.idIx:
				# 	if b != self.packetCheck[self.count]:
				# 		print("\nPacket out of alignment:")
				# 		print(self.curPacket, b, self.packetCheck[self.count])
				# 		self.curPacket = []
				# 		self.count = 0
				# 		self.checksum = 0
				# 		self.state = WAIT_HEADER
				# 		continue

				# After perhaps performing checks, add to packet
				self.curPacket.append(b)
				self.checksum += b
				self.count += 1

			# Once we have enough bytes, try to form packet
			if self.count == self.totalSize:
				# Condition for complete packet
				if ((self.checksum % 256) == 0):
					print("\nPacket:", self.wrapper(self.curPacket))
					self.lastPacket = list(self.curPacket) # Improve this
				else:
					print("Misaligned packet:", self.curPacket)
				
				# Either way, reset the packet
				self.curPacket  = []
				self.count      = 0
				self.checksum   = 0
				self.state      = WAIT_HEADER
				


	def wrapper(self, packet): 
		""" Assuming the packet is well formed, convert it to bytes,
			and then format according to the specification """
		# Get the data bytes from the packet, via numpy tricks
		tmp = np.array(packet)[~self.nonDataMask]
		#print(bytes(packet))
		#print(packet)
		#print(tmp)
		tmp = struct.pack("B"*self.numBytes, *tmp)
		ret = struct.unpack(self.dataFormat, tmp)
		return ret
		


def main():
	# Specify the port from command line
	port = sys.argv[1]  
	ser  = openPort(port, SERIAL_PARAMS)
	fd   = ser.fileno() # Not used in this version
	print(port)
	try:
		# Set up the robot
		setModePassive(ser)
		setModeFull(ser)
		packets = [24, 25, 26]  # Arbitrary choices of packet
		handler = csp3(packets) # Set up the handler
		handler.printParams()   # Print some information about handler
		requestStream(ser, packets)


		while True:
			# Essentially, wait and read. Could easily use select instead.
			numWait = ser.inWaiting()
			data    = ser.read(numWait)
			handler.inputLst(list(data))
			time.sleep(0.1)
	
	# Perform exceptional exception handling
	except Exception as e:
		print(e)

	# Ensure that the robot is shut down properly
	finally:
		shutdownRobot(ser)
		ser.close()
		sleep(0.5) # Superstition, because OS X serial hangs require a reboot 



if __name__ == "__main__":
	main()