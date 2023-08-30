import serial
import struct
import time
import sys

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

wheel_r = 0.1
wheel_d = 0.436
GearRa = 50
pi = 3.1415926
shaft_radius = 0.006
RW_rpm = None
LW_rpm = None
accel_deccel_value = 50 #1000

max_wheel_rpm = 85
usb_port = "/dev/ttyUSB2"
# usb_port = "/dev/motor"
r_pub = None
l_pub = None
go_prev_time = 0
current_time = 0
motion_init = False



class motor_communication:
    modem_device = usb_port
    baud_rate = 115200
    timex = None
    ser = None
    start = b'\x53'
    end = b'\x45'
    set = b'\x02'
    get = b'\x01'
    node1 = b'\x01'
    node2 = b'\x02'

    def check_conn(self):
        try:
            self.ser = serial.Serial(self.modem_device, self.baud_rate, timeout=self.timex)
            print("Serial details params: ", self.ser)
        except:
            print("Could not connect to usb port")
    
    def initialize_driver(self):
        global accel_deccel_value, motion_init

        ini0 = [0x53, 0x07, 0x01, 0x01, 0x18, 0x10, 0x04, 0xF4, 0x45]
        ini00 = [0x53, 0x07, 0x02, 0x01, 0x18, 0x10, 0x04, 0xA2, 0x45]
        ini1 = [0x53, 0x07, 0x00, 0x08, 0x0A, 0x10, 0x00, 0xEA, 0x45]
        ini2 = [0x53, 0x07, 0x00, 0x01, 0x29, 0x23, 0x0B, 0x52, 0x45]
        ini3 = [0x53, 0x07, 0x00, 0x01, 0x8F, 0x60, 0x01, 0xE8, 0x45]
        ini4 = [0x53, 0x07, 0x00, 0x01, 0x91, 0x60, 0x01, 0xF6, 0x45]
        ini5 = [0x53, 0x07, 0x00, 0x01, 0x91, 0x60, 0x02, 0x5F, 0x45]
        ini6 = [0x53, 0x07, 0x00, 0x01, 0x92, 0x60, 0x01, 0xF5, 0x45]
        ini7 = [0x53, 0x07, 0x00, 0x01, 0x92, 0x60, 0x02, 0x5C, 0x45]
        ini8 = [0x53, 0x07, 0x00, 0x01, 0x96, 0x60, 0x01, 0xF1, 0x45]
        ini9 = [0x53, 0x07, 0x00, 0x01, 0x96, 0x60, 0x02, 0x58, 0x45]
        ini10 = [0x53, 0x07, 0x00, 0x01, 0x28, 0x23, 0x01, 0xF3, 0x45]
        ini11 = [0x53, 0x07, 0x00, 0x01, 0x07, 0x24, 0x00, 0x25, 0x45]
        ini12 = [0x53, 0x07, 0x00, 0x08, 0x09, 0x10, 0x00, 0xE9, 0x45]
        ini13 = [0x53, 0x07, 0x00, 0x01, 0x02, 0x65, 0x00, 0x61, 0x45]
        ini14 = [0x53, 0x07, 0x00, 0x01, 0x60, 0x60, 0x00, 0xF9, 0x45]
        ini15 = [0x53, 0x07, 0x00, 0x01, 0x01, 0x30, 0x03, 0x34, 0x45]
        ini16 = [0x53, 0x07, 0x00, 0x01, 0x01, 0x30, 0x04, 0x33, 0x45]
        ini17 = [0x53, 0x08, 0x00, 0x02, 0x60, 0x60, 0x00, 0x03, 0xA3, 0x45]
        ini18 = [0x53, 0x07, 0x00, 0x01, 0x60, 0x60, 0x00, 0xF9, 0x45]
        ini19 = [0x53, 0x07, 0x00, 0x01, 0x61, 0x60, 0x00, 0xF8, 0x45]
        ini20 = [0x53, 0x09, 0x00, 0x02, 0x40, 0x60, 0x00, 0x06, 0x00, 0xD2, 0x45]
        ini21 = [0x53, 0x09, 0x00, 0x02, 0x40, 0x60, 0x00, 0x0F, 0x00, 0xDB, 0x45]
        ini22 = [0x53, 0x07, 0x00, 0x01, 0x60, 0x60, 0x00, 0xF9, 0x45]
        ini23 = [0x53, 0x07, 0x00, 0x01, 0x61, 0x60, 0x00, 0xF8, 0x45]
        
        # print(ini0)
        try:
            res = self.ser.write(ini0)
            res = self.ser.write(ini00)
            res = self.ser.write(ini1)
            res = self.ser.write(ini2)
            res = self.ser.write(ini3)
            res = self.ser.write(ini4)
            res = self.ser.write(ini5)
            res = self.ser.write(ini6)
            res = self.ser.write(ini7)
            res = self.ser.write(ini8)
            res = self.ser.write(ini9)
            res = self.ser.write(ini10)
            res = self.ser.write(ini11)
            res = self.ser.write(ini12)
            res = self.ser.write(ini13)
            res = self.ser.write(ini14)
            res = self.ser.write(ini15)
            res = self.ser.write(ini16)
            res = self.ser.write(ini17)
            res = self.ser.write(ini18)
            res = self.ser.write(ini19)
            res = self.ser.write(ini20)
            res = self.ser.write(ini21)
            res = self.ser.write(ini22)
            res = self.ser.write(ini23)

            accin1 = accel_deccel_value
            decin1 = accel_deccel_value
            accin2 = accel_deccel_value
            decin2 = accel_deccel_value
            acc_address = 0x6083
            dec_address = 0x6084
            subindex = 0x00

            # set acceleration
            accin1_value = self.speed_to_byte_command(accin1)
            accin1_command = self.create_command(self.node1, self.set, acc_address, subindex, accin1_value, 4)
            
            accin2_value = self.speed_to_byte_command(accin2)
            accin2_command = self.create_command(self.node2, self.set, acc_address, subindex, accin2_value, 4)

            # set deceleration
            decin1_value = self.speed_to_byte_command(decin1)
            decin1_command = self.create_command(self.node1, self.set, dec_address, subindex, decin1_value, 4)
            
            decin2_value = self.speed_to_byte_command(decin2)
            decin2_command = self.create_command(self.node2, self.set, dec_address, subindex, decin2_value, 4)
            
            # write them to driver
            # print("\nRSpeed: " + str(RSpeed) + "\n")
            # print("Accin1 command: " + str(accin1_command) + "\n")
            res = self.ser.write(accin1_command)
            res = self.ser.write(accin2_command)
            res = self.ser.write(decin1_command)
            res = self.ser.write(decin2_command)
            motion_init = True
            print("Initialization complete")
        except:
            print("Initialization failed")
        
        time.sleep(3)

    def s16(self, value):
        return -(value & 0x8000) | (value & 0x7fff)

    def increment_reader(self):
        global GearRa, RW_rpm, LW_rpm, shaft_radius
        actual_increment_node1 = [0x53, 0x07, 0x01, 0x01, 0x6C, 0x60, 0x00, 0x5E, 0x45]
        actual_increment_node2 = [0x53, 0x07, 0x02, 0x01, 0x6C, 0x60, 0x00, 0x08, 0x45]

        # address 0x608f, index 1
        L_Register = self.readRegister(0x606C, node = self.node1, subindex = 0)
        if (L_Register != 0):
            get_R_rpm = hex(int.from_bytes(L_Register,byteorder='little'))
            # print("HEX of Actual R_RPM= ", get_R_rpm)
            R_rpm = self.s16(int(get_R_rpm, 16))
            # RW_rpm = (0-R_rpm * shaft_radius) / wheel_r
            RW_rpm = (0-R_rpm ) / GearRa
            RW_rpm = round(RW_rpm, 2)
            # print("Shaft_R_RPM= ", R_rpm)
            # print("RW_RPM= ", RW_rpm)
        else:
            RW_rpm = None
        
        # get_R_tick = hex(int.from_bytes(self.readRegister(0x2315, node = self.node1, subindex = 3), byteorder='little'))
        # print("Right encoder tick= ", int(get_R_tick, 16))

        ## -----
        R_Register = self.readRegister(0x606C, node = self.node2, subindex = 0)
        if (R_Register != 0):
            get_L_rpm = hex(int.from_bytes(R_Register, byteorder='little'))
            # print("HEX of Actual L_RPM= ", get_L_rpm)
            L_rpm = self.s16(int(get_L_rpm, 16))
            # LW_rpm = (L_rpm * shaft_radius) / wheel_r
            LW_rpm = (L_rpm ) / GearRa
            LW_rpm = round(LW_rpm, 2)
            # print("Shaft_L_RPM= ", L_rpm)
            # print("LW_RPM= ", LW_rpm)
        else:
            LW_rpm = None

        # print ("===============End of increment reader==================")
    
    def readRegister(self, address, node = b'\x01', subindex = 0, debug = False):
        """Read Register 
        address: address of register to be read
        node = b'\x01' optional node
        sudindex = 0 optional subindex
        """    
        command = node + self.get + int.to_bytes(address, 2, 'little') + int.to_bytes(subindex, 1, 'little')
        # print("Read register command = ", command)
        return self.write(command)
    
    def write(self, command):
        global motion_init
        """Write command. The length of the command is 
        length of the argument  + 1 for the length byte + 1 for the CRC byte"""

        command = struct.pack("B", len(command) + 2) + command
        command = self.start + command + self.CRC(command) + self.end
        # print("command = ", command)

        try:
            self.ser.flushOutput()
            self.ser.flushInput()
            time.sleep(0.)
            self.ser.write(command)
            time.sleep(0.)
            res = self.read()
        except:
            res = 0
            motion_init = False

        return res
    
    def read(self):
        """First read the start bit and the length,
        then read the rest of the transmission."""

        ans = self.ser.read(2)   
        try:
            length = ans[1]
        except:
            print("Error:  Ans: ", ans)

        # print("Ans = ", ans)
        # print("Length = ", length)
        if (length > 11):
            return 0
        else:
            ansAll = ans + self.ser.read(length) # 2 nodes connected error occurs here
        #print(dump(ansAll))

        #check CRC is correct
        # assert self.CRC(ansAll[1:-2]) == struct.pack("B", ansAll[-2])

        # ansAll includes self.S, so data starts at position 7
        return ansAll[7:-2]
    
    def CRC(self, msg):
        """Calculate Cyclic Redundancy Check for message msg.
        msg is the entire command without SOF and EOF."""

        poly = 0xd5
        crc = 0xff

        for byte in msg:
            crc = crc ^ byte
            for _ in range(8):
                if crc & 0x01:
                    crc = ((crc >> 1) ^ poly) 
                else:
                    crc >>= 1

        return struct.pack("B", crc)                
    
    def CalcCRCByte(self, u8Byte, u8CRC):
        poly = 0xD5
        u8CRC = u8CRC ^ u8Byte
        for x in range(8):
            if(u8CRC & 0x01):
                u8CRC = (u8CRC >> 1) ^ poly
            else:
                u8CRC >>= 1
        return u8CRC

    def speed_to_byte_command(self,speed):
        # print("Current speed : ", str(speed))
        hex_speed = self.to_hex(speed, 32)
        # print("Hex speed = ", hex_speed)
        value = hex_speed
        return int.to_bytes(speed, 4, 'little', signed=True)

    def to_hex(self, val, nbits):
        if val == 0:
            return 0
        else:
            return hex((val + (1 << nbits)) % (1 << nbits)).lstrip('0x')

    def create_command(self, nodeID, action, address, subindex, value, value_length):
        address_command = int.to_bytes(address, 2, 'little')
        subindex_command = int.to_bytes(subindex, 1, 'little')
        # value_command = int.to_bytes(value, value_length, 'little',signed=True)
        value_command = value

        command = (nodeID + action + address_command + subindex_command + value_command)
        command = struct.pack("B", len(command) + 2) + command
        return self.start + command + self.CRC(command) + self.end 

    def go(self, Vel, Rot_speed): 
        global r_pub, l_pub, go_prev_time
        address = 0x60FF
        subindex = 0x00


        l_rpm = int((Vel-wheel_d*Rot_speed)/wheel_r/pi*30*GearRa)
        r_rpm = int((Vel+wheel_d*Rot_speed)/wheel_r/pi*30*GearRa)
        print("Vel: " + str(Vel) + " Rot: " + str(Rot_speed) + " l_rpm: " + str(l_rpm) + " r_rpm: " + str(r_rpm) + "\n")
        # print("wheel_d: " + str(wheel_d) + " wheel_r: " + str(wheel_r) + " GearRa: " + str(GearRa) + " pi: " + str(pi) + "\n")
        # print("RW_rpm: " + str((RW_rpm*wheel_r*-1)/shaft_radius) + " LW_rpm: " + str((LW_rpm*wheel_r*-1)/shaft_radius) + "\n")
        # print(" SIZE OF RSPEED = " + str(len(RSpeed)))
        Rvalue = self.speed_to_byte_command(r_rpm)
        RSpeed = self.create_command(self.node1, self.set, address, subindex, Rvalue, 4)
        Lvalue = self.speed_to_byte_command(0-l_rpm)
        LSpeed = self.create_command(self.node2, self.set, address, subindex, Lvalue, 4)
        # print("\nRSpeed: " + str(RSpeed) + "\n")
        # print("LSpeed: " + str(LSpeed) + "\n")
        try:
            res = self.ser.write(RSpeed)
            res = self.ser.write(LSpeed)
        except:
            print("Unable to write in nav go")
        # rospy.sleep(0.05)
        handle.increment_reader()


    def close_driver(self):
        print("Driver disconnect")
        Rshutdown = [0x53, 0x09, 0x01, 0x02, 0x40, 0x60, 0x00, 0x0D, 0x00, 0xD8, 0x45]
        Lshutdown = [0x53, 0x09, 0x02, 0x02, 0x40, 0x60, 0x00, 0x00, 0x00, 0x29, 0x45]
        
        try:
            res = self.ser.write(Lshutdown)
            res = self.ser.write(Rshutdown)
        except:
            print("Unable to close driver")

    def close_port(self):
        try:
            self.ser.close()
        except:
            print("Unable to close port")

    def travel(self, Vel, Rot_speed, time):
        """Travel with velocity "Vel", rotation speed "Rot_speed", 
        and with execution time "time" """
        self.go(Vel=Vel, Rot_speed=Rot_speed)
        time.sleep(time)



handle = motor_communication()


if __name__ == "__main__":
    handle.check_conn()
    handle.initialize_driver()
    # # hz = 5
    # # time_limit = 1/hz
    # # handle.increment_reader()
    # handle.go(0.03, 0)
    # time.sleep(2)
    handle.travel(0.03, 0, 2)
    # try:
    #     # handle.go(1, 0)
    #     # time.sleep(5)
    #     # handle.go(0, 0)
    #     print("ok")
    # except:
    #     print()
    # finally:
    #     handle.close_driver()
    #     handle.close_port()


    handle.close_driver()
    handle.close_port()    