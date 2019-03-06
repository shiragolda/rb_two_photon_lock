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

params_default = (1.0,
                  0.05,1e-3,0.01,
                  ZEROV,
                  1,
                  201.21,
                  0.300,
                  ToBits(3.0)+ZEROV,
                  4,
                  0.05)

params_struct_size = 4*11
params_struct_fmt = '<'+'ffffiiffiif'

"""
params =    float ramp amplitude [0]
            float gain_p, gain_i, gain_i2 [1],[2],[3]
            int output_offset [4]
            int scan_state [5]
            float ramp_frequency [6]
            fwhm [7]
            lock_point [8]
            n_averages [9]
            alpha [10]
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



class ArduinoLocker:
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
        time.sleep(0.1)
        self.print_params()
    
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
        #sanity check: sometimes the arduino returns garbage
        if(data_tuple[4]<65536 and data_tuple[4]>0 and data_tuple[5]<=1 and data_tuple[5]>=0 and data_tuple[6]<65536 and data_tuple[6]>0):
            self.params = list(data_tuple)
            return data_tuple
        else:
            print("Inappropriate data fetched")
            self.set_params()
            return self.params
        return data_tuple

    def set_params(self):
        """Set params on the microcontroller."""
        data = struct.pack(params_struct_fmt, *self.params)
        self.ser.write(b's'+data)
        time.sleep(0.1)
        s = self.ser.readline()
        print(s)

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

        
    def get_sampling_rate(self):
        self.ser.write(b't')
        sampling_time = self.ser.readline()
        print("Loop period: %i us"%int(sampling_time))

    def close(self):
        self.ser.close()


    def print_params(self):
        self.get_params()
        """ Print the current parameters on the microcontroller. """
        print('Ramp amplitude = {0:.3f} V'.format(self.params[0]))
        print('Ramp frequency = {0:.3f} Hz'.format(self.params[6]))
        print("Output offset = {0:.3f} V".format(ToVoltage(self.params[4]-ZEROV)))
        print('Servo Gain Parameters: P = {0:.3f}, I = {1:.3e}, I2 = {2:.3e}'.format(self.params[1],self.params[2],self.params[3]))
        print('Scan On/Off: {0:.0f}'.format(self.params[5]))
        print('Lock point: {0:.2f} V'.format(ToVoltage(self.params[8]-ZEROV)))
        print("FWHM: %.3f V"%self.params[7])
        print("N averages: %i"%self.params[9])
        print("Alpha : %.3f"%(self.params[10]))
    
    def scan_amp(self,new_amplitude):
        """Set scan amplitude on the microcontroller in Volts."""
        self.params[0] = new_amplitude
        self.set_params()
    
    def scan_freq(self,new_freq):
        """Set scan frequency on the microscontroller in Hz """
        self.params[6] = new_freq
        self.set_params()
        
    def set_lock_point(self,new_lp):
        self.params[8] = ToBits(new_lp)+ZEROV
        self.set_params()
    
    def lock(self):
        global p_state,i_state,i2_state
        self.set_scan_state(0)
        #lockpoint = ToVoltage(int(self.ser.readline())-ZEROV)
        #print(lockpoint)
        print("Locked")
    
    def unlock(self):
        self.set_scan_state(1)
        global local_accumulator
        local_accumulator = 0
        print("Unlocked - Scanning")
    
    def gain_p(self,p_gain):
        """ Set the proportional gain. Use a positive gain to lock to a positive slope error signal.  """
        self.params[1] = p_gain
        self.set_params()
        print("Proportional gain updated to:" + str(p_gain))
    
    def gain_i(self,i_gain):
        """ Set the integral gain """
        self.params[2] = i_gain
        self.set_params()
        print("Integral gain updated to:" + str(i_gain))
    
    def gain_i2(self,i2_gain):
        """ Set the integral^2 gain """
        self.params[3] = i2_gain
        self.set_params()
        print("Integral^2 gain updated to:" + str(i2_gain))
    
    def output_offset(self,offset):
        """ Set the output offset in V """
        self.params[4] = int(ToBits(offset)+ZEROV)
        self.set_params()
    
    def increase_offset(self):
        self.params[4] += int(ToBits(0.1))
        self.set_params()
    
    def decrease_offset(self):
        self.params[4] -= int(ToBits(0.1))
        self.set_params()
    
    def fwhm(self,new_fwhm):
        self.params[7] = new_fwhm
        self.set_params()
    
    def n_averages(self,new_n):
        self.params[9] = new_n
        self.set_params()

    def low_pass(self,new_lp):
        """ Change low-pass cutoff frequency in Hz """
        del_t = 1e-6*(10.9*self.params[9]+20.6) #s
        rc = 1.0/new_lp
        alpha = del_t/(rc+del_t)
        self.params[10] = alpha
        self.set_params()
        print("Low-pass cutoff changed to %.0f Hz (alpha= %.3f)"%(new_lp,alpha))
        

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

#cslock = CsLock()

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
