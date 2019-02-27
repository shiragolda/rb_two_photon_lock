/* Code written by Shreyas Fall 2017
 *  Modified by Shira Summer 2018
 *  Further Modified for single output by Shira Fall 2018
 *  
 *  Code operates in scanning mode/locking mode
 *  Scanning: sends out a ramp to the current of user-specified amplitude
 *  Locking: uses the centre-of-scan input voltage as a lockpoint, calculate PI^2 correction signal to current modulation
 *  
 *  In0 = photodiode signal
 *  Out0 = current modulation
 *  
 *  Communicates via serial - use with python cmd line program to update parameters and log data
 */

#include <EEPROM.h>
#include <analogShield.h>
#include <math.h>
#include<SPI.h>  // for ChipKit uc32 (SPI.h has to be included **after** analogShield.h)

#define UNO_ID "rb_lock\r\n"
#define ZEROV 32768 //Zero volts
#define V2P5 49512  // 2.5 V

#define STATE_LOCKING 0
#define STATE_SCANNING 1

#define ACCUMULATOR2_MAX 2000 //when accumulator is greater than this may be out of lock - code will pause accumulator
#define ACCUMULATOR1_MAX 800

#define INTEGRATOR_HOLD_TIME 500000  // microseconds - how long to pause accumulator for

struct Params {
  float ramp_amplitude;
  float gain_p, gain_i, gain_i2;
  long output_offset;
  long scan_state;
  float ramp_frequency;
  float fwhm;
  long lock_point;
};

Params params;

long in0;
long out0, out1, out2;

float half_period;
float ramp_slope;

bool ramp_direction;
long ramp_offset;
float error_signal;
float accumulator;
float accumulator2;
bool pause_accumulator;
long pause_time;

long loop_counter = 0;


bool write_to_serial_on_unlock;
bool serial_log;


float ToVoltage(float bits) {
  return (bits-32768)/6553.6;
}

float ToBits(float voltage) {
  return voltage*6553.6+32768;
}


void setup() {
  //SPI.setClockDivider(SPI_CLOCK_DIV2);
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  in0 = ZEROV;
  
  out0 = ZEROV;
  out1 = ZEROV;
  out2 = ZEROV;

  params.ramp_amplitude = 1.0; //volts
  params.gain_i = 10000; // integral gain
  params.gain_p = 10000; //proportional gain - current
  params.gain_i2 = 0.01; //integral^2 gain
  
  params.output_offset = ZEROV; //offset voltage to current modulation

  params.fwhm = 0.02; //V
  params.lock_point = ToBits(1.5); //V
  
  accumulator = 0.0; //will pause accumulation if gets too big (sudden jump to laser, such as bang on optics table)
  accumulator2 = 0.0; //will not pause, used for i^2 gain
  
  params.scan_state = STATE_SCANNING; //code is in scan mode / lock mode
  params.ramp_frequency = 20; // Hz
  
  processParams();
  error_signal = ZEROV;//0.0;

  pause_accumulator = true;
  serial_log = false;

}

void processParams() {
  ramp_slope = 4*params.ramp_amplitude*params.ramp_frequency*0.000001; // V/us
  half_period = 1000000/(2*params.ramp_frequency); // us
  accumulator = 0.0;
  accumulator2=0.0;
  if(params.scan_state==STATE_LOCKING){
    rampCycle(true);
    Serial.println(params.lock_point);
  }
}
void rampCycle(bool find_lock_point) {
  long ramp_time;
  long start_time;
  float offset = ToVoltage(params.output_offset);

  long sig_in;
  long max_sig = 0;
  long max_sig_out = params.output_offset;
  float lock_point_offset = ZEROV;
  
  start_time = micros();
  ramp_time = 0;
  while(ramp_time<half_period){
    ramp_time = micros() - start_time;
    out0 = ToBits(ramp_slope*ramp_time - params.ramp_amplitude + offset);
    analog.write(0,out0); 

    if(find_lock_point){
      sig_in = analog.read(0,false);
      if(sig_in>max_sig){
        max_sig = sig_in;
        max_sig_out = out0;
      }
    }
    
  }
  start_time = micros();
  ramp_time = 0;
  while(ramp_time<half_period){
    ramp_time = micros() - start_time;
    out0 = ToBits(-ramp_slope*ramp_time + params.ramp_amplitude + offset);
    analog.write(0,out0); 
  }

  if(find_lock_point){
    //lock_point= ToBits(1.5) ;//ToVoltage(max_sig)/2);
    lock_point_offset = max_sig_out-(ToBits(5*params.fwhm)-ZEROV);
    params.output_offset =lock_point_offset; //lock_point_offset;
  }
}

void loop() {
  loop_counter += 1;
  if(Serial.available()){
    parseSerial();
    //analog.write(0,params.output_offset);
  }
    
  in0 = analog.read(0, false); //read the voltage on channel 0

  float err_curr = ((float)(in0 - params.lock_point)); //current signal reading, in ??... in0 and lock_point are both in Bits
  error_signal = 0.95*error_signal + 0.05*err_curr; //low-passed error signal, in V
  
  out0 = ZEROV;
  out0 = params.output_offset; //add voltage offset to piezo

  // -----------------------------------------------------------------
  if(params.scan_state == STATE_SCANNING) {

    rampCycle(false);

  }
  // -------------------------------------------------------------------
  if(params.scan_state == STATE_LOCKING) {
  
    if(!pause_accumulator) { //...and the accumulator is not paused for a signal spike
       accumulator += params.gain_i*error_signal; //add the integral gain term to the accumulator
       accumulator2 += params.gain_i2*accumulator; //add the integral squared term to the accumualator2
    }
    if(abs(accumulator) > ACCUMULATOR1_MAX and !pause_accumulator) { //if the accumulator is too big, pause the accumulation!
      pause_accumulator = true;
      pause_time = micros();
    }
    
    if(abs(accumulator2) > ACCUMULATOR2_MAX) { //if accumulator 2 gets too big, out of lock
      // this could mean that we are out of lock... reset
      //params.scan_state = STATE_SCANNING; //return to scanning mode
      if(write_to_serial_on_unlock) { //send an "out of lock" message to the serial
        Serial.write('1');
      }
      processParams();
      
    }
    if(pause_accumulator) { //if the accumulator is paused
      if(micros() - pause_time > INTEGRATOR_HOLD_TIME) { //wait until the integrator hold time has passed
        pause_accumulator = false;
        accumulator *= 0.9; //reset the accumulator
      }
    }
  
   out0 -= params.gain_p*error_signal+accumulator2+accumulator; // note the overall sign on the gain
  }
  // -----------------------------------------------------------------
  
  /* Output voltages are the following: 
   *  out0 = V_Poff + V_ramp    in scanning mode
   *  out0 = V_Poff - accumulator 2 - accumulator1 - Pp*error     offset, proportional gain, integral gain, integral^2 gain
   *  
   *  accumulator1: accumulate I*error
   *  accumulator2: accumulate I2*accumulator1
   */
  
  analog.write(0,out0);  //WRITE OUT THE CORRECTION SIGNAL


  /*Write to serial for data logging in python */
  if(serial_log && loop_counter%50000 == 0) {
    Serial.print(((float)(in0 - ZEROV))/6553.6,4); //V
    Serial.print(',');
    Serial.println(((float)(out0 - ZEROV))/6553.6,4); //V
  }

}

/*
 * g - get params
 * w - write to eeprom
 * r - read from eeprom
 * i - return UNO_ID
 * s - set params
 */
void parseSerial() { //function to read and interpret characters passed from the python interface to the serial port
  char byte_read = Serial.read();

  switch(byte_read) {
    case 'g':
      // get params, send the entire struct in one go
      Serial.write((const uint8_t*)&params, sizeof(Params));
      break;
    case 'w':
      // write to EEPROM // save new default values
      EEPROM_writeAnything(0, params);
      break;
    case 'r':
      EEPROM_readAnything(0, params); //reset to default values
      // EEPROM_readAnything(sizeof(params), logger);      
      break;
    case 'i':
      // return ID
      Serial.write(UNO_ID); //not sure..........
      break;
    case 's':
      // set params struct
      Serial.readBytes((char *) &params, sizeof(Params));
      processParams(); //update the parameters as they've been changed in the python interface
      break;
    case 'm':
      // toggle serial log
      serial_log = !serial_log;
      break;
  }
}

template <class T> long EEPROM_writeAnything(long ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(ee++, *p++);
    return i;
}

template <class T> long EEPROM_readAnything(long ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(ee++);
    return i;
}
