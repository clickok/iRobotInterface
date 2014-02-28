# To add another sensor
Changes need to be made in
* B (the define referring to byte array size)
* To the second byte (under "request stream mode") in setupSerialPort()
* By adding the packet's opcode to the bytes (also under "request stream mode")
* In check packet, to account for the additional byte (corresponding to the packet's id number)
