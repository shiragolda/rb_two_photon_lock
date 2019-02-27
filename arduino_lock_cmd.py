import serial
import struct
import time
import numpy as np
from datetime import datetime
import os
#import zmq
import datetime
import sys
import threading


#N_STEPS = 800  # number of voltage steps per scan

def ToVoltage(bits):
    return float((bits)/6553.6)

def ToBits(voltage):
    return int((voltage*6553.6))

ZEROV = 32768

params_default = (1,
                  0.05,0.05,0.005,
                  ZEROV,
                  1,
                  20.547,
                  0.1,
                  ToBits(3.0)+ZEROV)

params_struct_size = 4*9
params_struct_fmt = '<'+'ffffiiffi'

"""
params =    float ramp amplitude [0]
            float gain_p, gain_i, gain_i2 [1],[2],[3]
            int output_offset [4]
            int scan_state [5]
            float ramp_frequency [6]
            fwhm [7]
            lock_point [8]
"""

p_state = 0
i_state = 0
i2_state = 0


# float ToVoltage(float bits) {
#   return (bits-32768)/6553.6;
# }

# float ToBits(float voltage) {
#   return voltage*6553.6+32768;
# }



class zmq_pub_dict:
    """Publishes a python dictionary on a port with a given topic."""

    def __init__(self, port=5580, topic='arduino_lock'):
        zmq_context = zmq.Context()
        self.topic = topic
        self.pub_socket = zmq_context.socket(zmq.PUB)

        self.pub_socket.bind("tcp://*:%s" % port)
        print('Broadcasting on port {0} with topic {1}'.format(port,
                                                               topic))

    def send(self, data_dict):
        timestamp = time.time()
        send_string = "%s %f %s" % (self.topic, timestamp, repr(data_dict))
        #print(send_string)
        self.pub_socket.send(send_string)

    def close(self):
        self.pub_socket.close()

    def publish_data(self,data):
        try:
            err,corr,msg = data
            data_dict = {'Error (V)': err, 'Correction (V)': corr, 'Message': msg}
            dt = str(time.time())
            self.send(data_dict)
        except:
            print('error retrieving/parsing data')


# ## Threading
# kbd_input = ''
# new_input = True

# def commandListener():
#     global kbd_input, new_input
#     kbd_input = input()
#     new_input = True


# publish = False

# def publishOn():
#     cslock.ser.write(b'm')
#     global publish
#     publish = True
#     print("Publishing...")

# def publishOff():
#     cslock.ser.write(b'm')
#     global publish
#     publish = False
#     print("Publishing off")


#publisher = zmq_pub_dict(5553,'cs_laser')



class CsLock:
    """Control the arduino set up for laser locking.

    Requires:
    An arduino Uno microcontroller flashed with the .ino code
    """


    def __init__(self, serialport='/dev/ttyUSB0'):
        self.serialport = serialport
        self.ser = serial.Serial(serialport, baudrate=115200, timeout=6.0)
        print("Connection to Arduino established at port %s"%self.ser.name)
        time.sleep(4)  # wait for microcontroller to reboot
        self.params = list(params_default)
        self.set_params()
        print(self.ser.isOpen())
        time.sleep(0.1)
    
    def default_params(self):
        self.params = list(params_default)
        self.set_params()

    def idn(self):
        """Read id of the microcontroller and return read string."""
        self.ser.write(b'i')
        return self.ser.readline()

    def load_from_eeprom(self):
        self.ser.write(b'r')

    def save_to_eeprom(self):
        self.ser.write(b'w')

    def get_params(self):
        """Get params structure from the microcontroller and store it locally."""
        write_string = b'g'
        self.ser.write(write_string)
        data = self.ser.read(params_struct_size)
        #print(data)
        data_tuple = struct.unpack(params_struct_fmt, data)
        #print(data_tuple)
        self.params = list(data_tuple)
        return data_tuple

    def set_params(self):
        """Set params on the microcontroller."""
        data = struct.pack(params_struct_fmt, *self.params)
        self.ser.write(b's'+data)

    def set_scan_state(self,scan_state):
        self.params[5] = int(scan_state)
        self.set_params()

    def get_data(self):
        s = self.ser.readline()
        try:
            err,corr = s.split(',')
            err = float(err)
            corr = float(corr)
        except:
            err = 0
            corr = 0
        return err,corr


    def close(self):
        self.ser.close()


def print_params():
        """ Print the current parameters on the microcontroller. """
        print('Ramp amplitude = {0:.3f} V'.format(cslock.params[0]))
        print('Ramp frequency = {0:.3f} Hz'.format(cslock.params[6]))
        print("Output offset = {0:.3f} V".format(ToVoltage(cslock.params[4]-ZEROV)))
        print('Servo Gain Parameters: P = {0:.3f}, I = {1:.3f}, I2 = {2:.3f}'.format(cslock.params[1],cslock.params[2],cslock.params[3]))
        print('Scan On/Off: {0:.0f}'.format(cslock.params[5]))
        print('Lock point: {0:.2f} V'.format(ToVoltage(cslock.params[8]-ZEROV)))
        print("FWHM: %.3f V"%cslock.params[7])

def scan_amp(new_amplitude):
    """Set scan amplitude on the microcontroller in Volts."""
    cslock.params[0] = new_amplitude
    cslock.set_params()

def scan_freq(new_freq):
    """Set scan frequency on the microscontroller in Hz """
    cslock.params[6] = new_freq
    cslock.set_params()
    
def set_lock_point(new_lp):
    cslock.params[8] = ToBits(new_lp)+ZEROV
    cslock.set_params()

def lock():
    global p_state,i_state,i2_state
    cslock.set_scan_state(0)
    lockpoint = ToVoltage(int(cslock.ser.readline())-ZEROV)
    print(lockpoint)
    print("Locked")
    cslock.get_params()

def unlock():
    cslock.set_scan_state(1)
    global local_accumulator
    local_accumulator = 0
    print("Unlocked - Scanning")

def gain_p( p_gain):
    """ Set the proportional gain. Use a positive gain to lock to a positive slope error signal.  """
    cslock.params[1] = p_gain
    cslock.set_params()
    print("Proportional gain updated to:" + str(p_gain))

def gain_i(i_gain):
    """ Set the integral gain """
    cslock.params[2] = i_gain
    cslock.set_params()
    print("Integral gain updated to:" + str(i_gain))

def gain_i2(i2_gain):
    """ Set the integral^2 gain """
    cslock.params[3] = i2_gain
    cslock.set_params()
    print("Integral^2 gain updated to:" + str(i2_gain))

def output_offset(offset):
    """ Set the output offset in V """
    cslock.params[4] = int(ToBits(offset)+ZEROV)
    cslock.set_params()

def increase_offset():
    cslock.params[4] += int(ToBits(0.1))
    cslock.set_params()

def decrease_offset():
    cslock.params[4] -= int(ToBits(0.1))
    cslock.set_params()

def fwhm(new_fwhm):
    cslock.params[7] = new_fwhm
    cslock.set_params()

#def scan_freq_nsteps(n_steps):
#    """ Set the number of steps"""
#    cslock.params[6] = int(n_steps)
#    cslock.set_params()
    
# def scan_freq(frequency):
#     """ Set the frequecncy in Hz """
#     freq_to_steps = 10.5263157895 #conversion between frequency and n steps in units steps/Hz
#     new_steps = int(round(freq_to_steps*frequency))
#     cslock.params[6] = int(new_steps)
#     cslock.set_params()

# def publish_data():
#     publisher = zmq_pub_dict(5553,'cs_laser')
#     cslock.ser.write(b'm')
#     local_accumulator = 0
#     try:
#         while True:
#             err,corr = cslock.get_data()
#             data = (err,corr)
#             publisher.publish_data(data)

#     except KeyboardInterrupt:
#         pass
#     cslock.ser.write(b'm')
#     publisher.close()


# def quit():
#     print("Loop exited")
#     publishOff()
#     loop_running = False

# def start():
#     print("Loop starting")
#     publishOn()
#     loop_running = True

#if __name__ == '__main__':
#    cslock = CsLock()

cslock = CsLock()

# loop_running = True
# local_accumulator = 0
# while(loop_running):
#     try:
#         if(publish):
#             try:
#                 err,corr = cslock.get_data()
#                 local_accumulator+=err
#                 msg = 'ok'
#                 if(np.abs(local_accumulator)>10000.0):
#                     msg = 'out of lock'
#                 data = (err,corr,msg)
#                 publisher.publish_data(data)
#             except:
#                 pass

#         if(new_input):
#             try:
#                 exec(kbd_input)
#             except:
#                 print("Invalid input")
#             new_input = False
#             listener = threading.Thread(target=commandListener, daemon=True)
#             listener.start()

#     except(KeyboardInterrupt):
#         print("Loop exited")
#         cslock.ser.write(b'm')
#         loop_running = False
#         break

#publisher.close()
#cslock.close()
