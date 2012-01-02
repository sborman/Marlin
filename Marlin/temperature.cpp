/*
  temperature.c - temperature control
  Part of Marlin
  
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)
 
 It has preliminary support for Matthew Roberts advance algorithm 
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html

 */


#include "Marlin.h"
#include "ultralcd.h"
#include "temperature.h"
#include "watchdog.h"

//===========================================================================
//=============================public variables============================
//===========================================================================
int target_raw[EXTRUDERS] = { 0 };
int temp_target_bed_raw = 0;

#ifdef BED_LIMIT_SWITCHING
int target_bed_low_temp = 0;  
int target_bed_high_temp = 0;
#endif

int temp_current_raw[EXTRUDERS] = { 0 };
int temp_current_bed_raw = 0;

#ifdef PIDTEMP
  // used external
  float temp_setpoint[EXTRUDERS] = { 0.0 };
  
  float Kp = DEFAULT_Kp; // Proportional
  float Ki = DEFAULT_Ki; // Integral
  float Kd = DEFAULT_Kd; // Derivative

  #ifdef PID_ADD_EXTRUSION_RATE
  float Kc = DEFAULT_Kc; // Adjustment for extrusion rate (experimental)
  #endif
#endif //PIDTEMP 
  
//===========================================================================
//=============================private variables============================
//===========================================================================
static bool temp_measurement_avail = false; // is a reliable temperature measurement available?

static unsigned long  previous_millis_bed_heater;
//static unsigned long previous_millis_heater;

#ifdef PIDTEMP
  //static cannot be external:
  static float temp_iState[EXTRUDERS] = { 0 };
  static float temp_dState[EXTRUDERS] = { 0 };
  static float pTerm[EXTRUDERS];
  static float iTerm[EXTRUDERS];
  static float dTerm[EXTRUDERS];
  //int output;
  static float temp_error[EXTRUDERS];
  static float temp_iState_min[EXTRUDERS];
  static float temp_iState_max[EXTRUDERS];
  // static float temp_current[EXTRUDERS];
  // static float heat_output[EXTRUDERS];
  static bool pid_reset[EXTRUDERS];
#endif //PIDTEMP

  static uint8_t soft_pwm[EXTRUDERS]; // Software PWM value for extruder heaters
  
#ifdef WATCHPERIOD
  static int watch_raw[EXTRUDERS] = { -1000 }; // the first value used for all
  static int watch_oldtemp[3] = {0,0,0};
  static unsigned long watchmillis = 0;
#endif //WATCHPERIOD

// Init min and max temp with extreme values to prevent false errors during startup
  static int mintemp[EXTRUDERS] = { 0 };
  static int maxttemp[EXTRUDERS] = { 16383 }; // the first value used for all
  static int bed_mintemp = 0;
  static int bed_maxttemp = 16383;
  static int heater_pin_map[EXTRUDERS] = { HEATER_0_PIN
#if EXTRUDERS > 1
                                         , HEATER_1_PIN
#endif
#if EXTRUDERS > 2
                                         , HEATER_2_PIN
#endif
#if EXTRUDERS > 3
  #error Unsupported number of extruders
#endif
  };
  static void *heater_ttbl_map[EXTRUDERS] = { (void *)heater_0_temptable
#if EXTRUDERS > 1
                                            , (void *)heater_1_temptable
#endif
#if EXTRUDERS > 2
                                            , (void *)heater_2_temptable
#endif
#if EXTRUDERS > 3
  #error Unsupported number of extruders
#endif
  };
  static int heater_ttbllen_map[EXTRUDERS] = { heater_0_temptable_len
#if EXTRUDERS > 1
                                             , heater_1_temptable_len
#endif
#if EXTRUDERS > 2
                                             , heater_2_temptable_len
#endif
#if EXTRUDERS > 3
  #error Unsupported number of extruders
#endif
  };

//===========================================================================
//=============================   functions      ============================
//===========================================================================
  
void updatePID() {
#ifdef PIDTEMP
  for (uint8_t e = 0; e < EXTRUDERS; e++) { 
     temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;
  }
#endif
}

// Get heater power level
int getHeaterPower(int heater) {
  return soft_pwm[heater];
}


// This is the main function doing the PID loop
// The PID loop uses temperature as the control variable
void manage_heater() {
 
  #ifdef PID_DEBUG
    static unsigned long visits = 0;
  #endif
  
  #ifdef USE_WATCHDOG
    wd_reset();
  #endif
  
  float temp_current; // temperature of the hotend in deg C
  float heat_output; // drive signal for the heater

  if (!temp_measurement_avail) 
    return;
  
  CRITICAL_SECTION_START;
  temp_measurement_avail = false;
  CRITICAL_SECTION_END;

  for (uint8_t e=0; e < EXTRUDERS; e++) {

  #ifdef PIDTEMP

    // The PID controller uses temperature, not raw measurements
    // as the control variable, so convert to temperature [deg C]
    temp_current = raw2temp(temp_current_raw[e], e);

    #ifndef PID_OPENLOOP
    
        // The error is the difference between the setpoint (desired)
        // and the measured temperature of the hotend
        temp_error[e] = temp_setpoint[e] - temp_current; // e(t) = u(t) - y(t)
        
        // If the error is large {+,-} set output to {max,0} respectively
        if (temp_error[e] > PID_ERR_BIG) {
           // Hotend is much colder than setpoint, max power to heater
           #ifdef PID_DEBUG
             if (visits%20 == 0) {
             SERIAL_ECHO_START;
             SERIAL_ECHOLN("manage_heater(): Temperature error > PID_ERR_BIG: Setting heater to maximum power.");
	     }
           #endif
           heat_output = PID_MAX;
           pid_reset[e] = true;
        }
        else if (temp_error[e] < -PID_ERR_BIG) {
           // Hotend is much hotter than setpoint, shut off heater
           #ifdef PID_DEBUG
             if (visits%20 == 0) {
             SERIAL_ECHO_START;
             SERIAL_ECHOLN("manage_heater(): Temperature error < -PID_ERR_BIG: Turning off heater.");
	     }
           #endif
          heat_output = 0;
          pid_reset[e] = true;
        }
        else {
          // Normal: Temperature is within +/- PID_ERR_BIG degrees of the setpoint
          if (pid_reset[e] == true) {
            temp_iState[e] = 0.0;
            pid_reset[e] = false;
          }

          // Proportional term
          pTerm[e] = Kp * temp_error[e];

          // Integral term
          temp_iState[e] += temp_error[e];
          temp_iState[e] = constrain(temp_iState[e], temp_iState_min[e], temp_iState_max[e]);
          iTerm[e] = Ki * temp_iState[e];

          // Derivative term
          // PID_K1 defined in Configuration.h in the PID settings
          // #define K2 (1.0 - PID_K1)
          dTerm[e] = PID_K1 * dTerm[e] + (1.0-PID_K1) * (Kd * (temp_current - temp_dState[e]));
          temp_dState[e] = temp_current;

          // controller output 
          heat_output = constrain(pTerm[e] + iTerm[e] - dTerm[e], 0, PID_MAX);
        }
    #endif //PID_OPENLOOP

    #ifdef PID_DEBUG
      if (visits%20 == 0) {
        SERIAL_ECHO_START;
        SERIAL_ECHO(millis());
        SERIAL_ECHO(" E: ");
        SERIAL_ECHO((unsigned int)e);
        SERIAL_ECHO(" Raw: ");
        SERIAL_ECHO(temp_current_raw[e]);
        SERIAL_ECHO(" DegC: ");
        SERIAL_ECHO(temp_current);
        SERIAL_ECHO(" Setpoint: ");
        SERIAL_ECHO(temp_setpoint[e]);
        SERIAL_ECHO(" Error: ");
        SERIAL_ECHO(temp_error[e]);
        SERIAL_ECHO(" P: ");
        SERIAL_ECHO(pTerm[e]);
        SERIAL_ECHO(" I: ");
        SERIAL_ECHO(iTerm[e]);
        SERIAL_ECHO(" D: ");
        SERIAL_ECHO(dTerm[e]);
        SERIAL_ECHO(" Out: ");
        SERIAL_ECHOLN(heat_output);
      }
    #endif
 
    #ifdef PID_DEBUG
    //SERIAL_ECHOLN(" PIDDEBUG "<<e<<": Input "<<temp_current<<" Output "<<heat_output" pTerm "<<pTerm[e]<<" iTerm "<<iTerm[e]<<" dTerm "<<dTerm[e]);  
    #endif //PID_DEBUG

  #else // #ifdef PIDTEMP 

    // PIDTEMP is not defined so use a simple bang-bang controller
    // This is not safe on its own!
    if ( temp_current_raw[e] < target_raw[e] )
       heat_output = PID_MAX;
    else
       heat_output 0;

  #endif // PIDTEMP

    // Temperature safety check, then set output
    if ( (temp_current_raw[e] > mintemp[e]) && (temp_current_raw[e] < maxttemp[e]) ) {
      //analogWrite(heater_pin_map[e], heat_output);
      soft_pwm[e] = (int)heat_output >> 1;
    }
    else {
      //analogWrite(heater_pin_map[e], 0);
      soft_pwm[e] = 0;
    }

  } // End extruder for loop

  // FIXME: Make this work for all extruders, not just the first one
  //        Also reconcile this with the hotbed watch period stuff below
  #ifdef WATCHPERIOD
  if (watchmillis && millis() - watchmillis > WATCHPERIOD) {
     if (watch_oldtemp[TEMPSENSOR_HOTEND_0] >= degHotend(active_extruder)) {
        setTargetHotend(0,active_extruder);
        LCD_MESSAGEPGM("Heating failed");
        SERIAL_ECHO_START;
        SERIAL_ECHOLN("Heating failed");
     }
     else {
        watchmillis = 0;
     }
  }
  #endif // WATCHPERIOD

  //============================
  // Hot bed temperature control
  //============================
  //
  // FIXME: Why isn't the hotbed handled as just another extruder?
 
  // Check hot-bed every BED_CHECK_INTERVAL milliseconds
  if (millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL)
    return;
 
  previous_millis_bed_heater = millis();
  
  #if TEMP_BED_PIN > -1
  
    #ifndef BED_LIMIT_SWITCHING

      // No bed limit switching uses 2 limits and a setpoint
      //   bed_mintemp < temp_target_bed_raw < bed_maxttemp
      // The controller will try to keep the temperature around temp_target_bed_raw 
      // and within the range (bed_mintemp, bed_maxttemp)

      if ((temp_current_bed_raw > bed_mintemp) && (temp_current_bed_raw < bed_maxttemp)) {
        // Temperate is within (min,max) range, turn heater on or off as needed
        if (temp_current_bed_raw >= temp_target_bed_raw)
            WRITE(HEATER_BED_PIN,LOW);  
         else
            WRITE(HEATER_BED_PIN,HIGH);
      }
      else
        // Shut off the heater if the current temperature is less than the minimun or greater than the maximum
        WRITE(HEATER_BED_PIN,LOW);

    #else // BED_LIMIT_SWITCHING

      // Bed limit switching uses 4 limits:
      // bed_mintemp < target_bed_low_temp < target_bed_high_temp < bed<maxttemp
      // The controller will turn on and off the heater to try to keep the
      // temperature in the range (target_bed_low_temp, target_bed_high_temp)

      if ((temp_current_bed_raw > bed_mintemp) && (temp_current_bed_raw < bed_maxttemp)) {
        // Temperate is within (min,max) range, turn heater on or off as needed
        if (temp_current_bed_raw > target_bed_high_temp)
           WRITE(HEATER_BED_PIN,LOW);
        else if (temp_current_bed_raw < target_bed_low_temp)
             WRITE(HEATER_BED_PIN,HIGH);
        else
             WRITE(HEATER_BED_PIN,LOW);
      }
      else
        // Shut off the heater if the current temperature is less than the minimun or greater than the maximum
        WRITE(HEATER_BED_PIN,LOW);
                
    #endif // BED_LIMIT_SWITCHING
  #endif // #if TEMP_BED_PIN > -1

  #ifdef PID_DEBUG
      visits++;
  #endif

}

// Macro to streamline reading tables in program (not data) address space
// See: http://www.nongnu.org/avr-libc/user-manual/pgmspace.html
#define PGM_RD_W(x) (short)pgm_read_word(&x)

// Takes hot end temperature value as input and returns corresponding raw value. 
// For a thermistor, it uses the RepRap thermistor temp table.
// This is needed because PID in hydra firmware hovers around a given A/D (raw) value, not a temperature value.
// This function is derived by inverting a portion of getTemperature() in FiveD RepRap firmware.
//
int temp2raw(int celsius, uint8_t e) {
  if(e >= EXTRUDERS)
  {
      SERIAL_ERROR_START;
      SERIAL_ERROR((int)e);
      SERIAL_ERRORLNPGM(" - Invalid extruder number!");
      kill();
  }
  if(heater_ttbl_map[e] != 0)
  {
    int raw = 0;
    byte i;
    short (*tt)[][2] = (short (*)[][2])(heater_ttbl_map[e]);

    for (i=1; i<heater_ttbllen_map[e]; i++)
    {
      if (PGM_RD_W((*tt)[i][1]) < celsius)
      {
        raw = PGM_RD_W((*tt)[i-1][0]) + 
          (celsius - PGM_RD_W((*tt)[i-1][1])) * 
          (PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i-1][0])) /
          (PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i-1][1]));  
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e]) raw = PGM_RD_W((*tt)[i-1][0]);

    return (1023 * OVERSAMPLENR) - raw;
  }
  return celsius * (1024.0 / (5.0 * 100.0) ) * OVERSAMPLENR;
}

// Takes bed temperature value as input and returns corresponding raw value. 
// For a thermistor, it uses the RepRap thermistor temp table.
// This is needed because PID in hydra firmware hovers around a given analog value, not a temp value.
// This function is derived from inverting the logic from a portion of getTemperature() in FiveD RepRap firmware.
int temp2rawBed(int celsius) {
#ifdef BED_USES_THERMISTOR
    int raw = 0;
    byte i;
    
    for (i=1; i<bedtemptable_len; i++)
    {
      if (PGM_RD_W(bedtemptable[i][1]) < celsius)
      {
        raw = PGM_RD_W(bedtemptable[i-1][0]) + 
          (celsius - PGM_RD_W(bedtemptable[i-1][1])) * 
          (PGM_RD_W(bedtemptable[i][0]) - PGM_RD_W(bedtemptable[i-1][0])) /
          (PGM_RD_W(bedtemptable[i][1]) - PGM_RD_W(bedtemptable[i-1][1]));
      
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == bedtemptable_len) raw = PGM_RD_W(bedtemptable[i-1][0]);

    return (1023 * OVERSAMPLENR) - raw;
#elif defined BED_USES_AD595
    return lround(celsius * (1024.0 * OVERSAMPLENR/ (5.0 * 100.0) ) );
#else
    #warning No heater-type defined for the bed.
    return 0;
#endif
}

// Convert raw measurements to temperature using lookup tables
// Derived from RepRap FiveD extruder::getTemperature()
// for hot end temperature measurement
float raw2temp(int raw, uint8_t e) {

  float celsius; // temperture return value
  uint8_t i;
  
  if (e >= EXTRUDERS) {
     SERIAL_ERROR_START;
     SERIAL_ERROR((int)e);
     SERIAL_ERRORLNPGM(" - Invalid extruder number !");
     kill();
  }
  
  #ifdef PID_DEBUG_RAW2TEMP
    // dump the temperature lookup table as a sanity check
    SERIAL_ECHO_START;
    SERIAL_ECHOLN("Temperature lookup table");
    short (*tt)[][2] = (short (*)[][2])(heater_ttbl_map[e]);
    for (i=0; i < heater_ttbllen_map[e]; i++ )  {
        SERIAL_ECHO  (PGM_RD_W((*tt)[i][0]));
        SERIAL_ECHO  (",");
        SERIAL_ECHOLN(PGM_RD_W((*tt)[i][1]));
    }
  #endif
  
  if (heater_ttbl_map[e] != 0) {
     short (*tt)[][2] = (short (*)[][2])(heater_ttbl_map[e]);
     // FIXME: Why are we doing this?  Commenting it out for now.
     // raw = (1023 * OVERSAMPLENR) - raw; 
     for (i=1; i < heater_ttbllen_map[e]; i++ )  {
       // Linear interpolation to find the temperature that corresponds
       // to the raw ADC value
       if (PGM_RD_W((*tt)[i][0]) > raw) {
          celsius = PGM_RD_W((*tt)[i-1][1]) + 
            (raw - PGM_RD_W((*tt)[i-1][0])) * 
            (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i-1][1])) /
            (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i-1][0]));
          break;
       }
     }

     #ifdef PID_DEBUG_RAW2TEMP
     SERIAL_ECHO_START;
     SERIAL_ECHO("raw2temp(): ");
     SERIAL_ECHO(raw);
     SERIAL_ECHO("->");
     SERIAL_ECHOLN(celsius);
     #endif  

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map[e]) {
       celsius = PGM_RD_W((*tt)[i-1][1]);
       #ifdef PID_DEBUG_RAW2TEMP
       SERIAL_ECHO_START;
       SERIAL_ECHO("raw2temp(): OVERFLOW! ");
       SERIAL_ECHO(raw);
       SERIAL_ECHO("->");
       SERIAL_ECHOLN(celsius);
       #endif  
    }
    return celsius;
  }
  
  // No temperature lookup table, so it's probably a thermocouple on an AD595 or equivalent
  // AD595 outpus 10mV per degree C (or equivalently 100 deg C per volt)
  // 5.0 volt supply
  // 10-bit A/D converter giving output in range [0,1023] corresponding to [0,5] volts
  return (float)raw * 5.0 * 100.0 / (1023*OVERSAMPLENR);
  // return raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR;
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
float raw2tempBed(int raw) {
  #ifdef BED_USES_THERMISTOR
    int celsius = 0;
    byte i;

    raw = (1023 * OVERSAMPLENR) - raw;

    for (i=1; i<bedtemptable_len; i++)
    {
      if (PGM_RD_W(bedtemptable[i][0]) > raw)
      {
        celsius = PGM_RD_W(bedtemptable[i-1][1]) + 
          (raw - PGM_RD_W(bedtemptable[i-1][0])) * 
          (PGM_RD_W(bedtemptable[i][1]) - PGM_RD_W(bedtemptable[i-1][1])) /
          (PGM_RD_W(bedtemptable[i][0]) - PGM_RD_W(bedtemptable[i-1][0]));

        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == bedtemptable_len) celsius = PGM_RD_W(bedtemptable[i-1][1]);

    return celsius;
    
  #elif defined BED_USES_AD595
    return raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR;
  #else
    #warning No heater-type defined for the bed.
  #endif
  return 0;
}

void tp_init()
{
  // Finish init of mult extruder arrays 
  for(int e = 0; e < EXTRUDERS; e++) {
    // populate with the first value 
#ifdef WATCHPERIOD
    watch_raw[e] = watch_raw[0];
#endif
    maxttemp[e] = maxttemp[0];
#ifdef PIDTEMP
    temp_iState_min[e] = 0.0;
    temp_iState_max[e] = PID_INTEGRAL_DRIVE_MAX / Ki;
#endif //PIDTEMP
  }

  #if (HEATER_0_PIN > -1) 
    SET_OUTPUT(HEATER_0_PIN);
  #endif  
  #if (HEATER_1_PIN > -1) 
    SET_OUTPUT(HEATER_1_PIN);
  #endif  
  #if (HEATER_2_PIN > -1) 
    SET_OUTPUT(HEATER_2_PIN);
  #endif  
  #if (HEATER_BED_PIN > -1) 
    SET_OUTPUT(HEATER_BED_PIN);
  #endif  
  #if (FAN_PIN > -1) 
    SET_OUTPUT(FAN_PIN);
  #endif  

  // Set analog inputs
  ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIF | 0x07;
  DIDR0 = 0;
  #ifdef DIDR2
    DIDR2 = 0;
  #endif
  #if (TEMP_0_PIN > -1)
    #if TEMP_0_PIN < 8
       DIDR0 |= 1 << TEMP_0_PIN; 
    #else
       DIDR2 |= 1<<(TEMP_0_PIN - 8); 
    #endif
  #endif
  #if (TEMP_1_PIN > -1)
    #if TEMP_1_PIN < 8
       DIDR0 |= 1<<TEMP_1_PIN; 
    #else
       DIDR2 |= 1<<(TEMP_1_PIN - 8); 
    #endif
  #endif
  #if (TEMP_2_PIN > -1)
    #if TEMP_2_PIN < 8
       DIDR0 |= 1 << TEMP_2_PIN; 
    #else
       DIDR2 = 1<<(TEMP_2_PIN - 8); 
    #endif
  #endif
  #if (TEMP_BED_PIN > -1)
    #if TEMP_BED_PIN < 8
       DIDR0 |= 1<<TEMP_BED_PIN; 
    #else
       DIDR2 |= 1<<(TEMP_BED_PIN - 8); 
    #endif
  #endif
  
  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  OCR0B = 128;
  TIMSK0 |= (1<<OCIE0B);  
  
  // Wait for temperature measurement to settle
  delay(250);

#ifdef HEATER_0_MINTEMP
  mintemp[0] = temp2raw(HEATER_0_MINTEMP, 0);
#endif //MINTEMP
#ifdef HEATER_0_MAXTEMP
  maxttemp[0] = temp2raw(HEATER_0_MAXTEMP, 0);
#endif //MAXTEMP

#if (EXTRUDERS > 1) && defined(HEATER_1_MINTEMP)
  mintemp[1] = temp2raw(HEATER_1_MINTEMP, 1);
#endif // MINTEMP 1
#if (EXTRUDERS > 1) && defined(HEATER_1_MAXTEMP)
  maxttemp[1] = temp2raw(HEATER_1_MAXTEMP, 1);
#endif //MAXTEMP 1

#if (EXTRUDERS > 2) && defined(HEATER_2_MINTEMP)
  mintemp[2] = temp2raw(HEATER_2_MINTEMP, 2);
#endif //MINTEMP 2
#if (EXTRUDERS > 2) && defined(HEATER_2_MAXTEMP)
  maxttemp[2] = temp2raw(HEATER_2_MAXTEMP, 2);
#endif //MAXTEMP 2

#ifdef BED_MINTEMP
  bed_mintemp = temp2rawBed(BED_MINTEMP);
#endif //BED_MINTEMP
#ifdef BED_MAXTEMP
  bed_maxttemp = temp2rawBed(BED_MAXTEMP);
#endif //BED_MAXTEMP
}



void setWatch() 
{  
#ifdef WATCHPERIOD
  int t = 0;
  for (int e = 0; e < EXTRUDERS; e++)
  {
    if(isHeatingHotend(e))
    watch_oldtemp[TEMPSENSOR_HOTEND_0] = degHotend(0);
    {
      t = max(t,millis());
      watch_raw[e] = temp_current_raw[e];
    } 
  }
  watchmillis = t;
#endif 
}


void disable_heater()
{
  for(int i=0;i<EXTRUDERS;i++)
    setTargetHotend(0,i);
  setTargetBed(0);
  #if TEMP_0_PIN > -1
  target_raw[0]=0;
  soft_pwm[0]=0;
   #if HEATER_0_PIN > -1  
     digitalWrite(HEATER_0_PIN,LOW);
   #endif
  #endif
     
  #if TEMP_1_PIN > -1
    target_raw[1]=0;
    soft_pwm[1]=0;
    #if HEATER_1_PIN > -1 
      digitalWrite(HEATER_1_PIN,LOW);
    #endif
  #endif
      
  #if TEMP_2_PIN > -1
    target_raw[2]=0;
    soft_pwm[2]=0;
    #if HEATER_2_PIN > -1  
      digitalWrite(HEATER_2_PIN,LOW);
    #endif
  #endif 

  #if TEMP_BED_PIN > -1
    temp_target_bed_raw=0;
    #if HEATER_BED_PIN > -1  
      digitalWrite(HEATER_BED_PIN,LOW);
    #endif
  #endif 
}

void max_temp_error(uint8_t e) {
  digitalWrite(heater_pin_map[e], 0);
  SERIAL_ERROR_START;
  SERIAL_ERRORLN(e);
  SERIAL_ERRORLNPGM(": Extruder switched off. MAXTEMP triggered !");
}

void min_temp_error(uint8_t e) {
  digitalWrite(heater_pin_map[e], 0);
  SERIAL_ERROR_START;
  SERIAL_ERRORLN(e);
  SERIAL_ERRORLNPGM(": Extruder switched off. MINTEMP triggered !");
}

void bed_max_temp_error(void) {
  digitalWrite(HEATER_BED_PIN, 0);
  SERIAL_ERROR_START;
  SERIAL_ERRORLNPGM("Temperature heated bed switched off. MAXTEMP triggered !!");
}


// Interrupt service routine for doing temperature measurement
// Timer 0 is shared with millis
ISR(TIMER0_COMPB_vect)
{
  // These variables are only accessible from the ISR, but static, so they don't lose their value
  static uint8_t temp_measurements = 0;
  static uint8_t temp_state = 0;

  // Hotend temperature sensor raw measurement accumulator
  // FIXME: Could save a few bytes by #ifdef'ing out the unneeded ones
  static unsigned long temp_0_raw = 0;
  static unsigned long temp_1_raw = 0;
  static unsigned long temp_2_raw = 0;

  // Hotbed temperature sensor raw measurement accumulator
  static unsigned long temp_bed_raw = 0;

  #ifdef PID_DEBUG
    static unsigned long visits = 0;
  #endif

  // Software-based Pulse Width Modulation (PWM) output
  //
  // If the microcontroller doesn't support hardware PWM pins
  // then the heater output power can be controlled by implementing
  // PWM in software as follows:
  //
  // Output
  //
  //   1 ======+                   ======+
  //     |     |                   |     |
  //   0 +-----========/ /=========+-----========/ /=========> pwm_count (time)
  //     0     |                   0   
  //        soft_pwm            127
  //     |<--- 128 interrupts ---->|
  // 
  // The duty cycle % is given by (soft_pwm_*/128)*100
  // This ISR is entered every 1ms
  // 
  // Questions:
  // - Why was PWM implemented in software when all the AVRs in printer support HW PWM?
  // - Current versions of Sprinter use PWM outputs on the AVR MCUs, why not Marlin?
  // - Why is the period 128 cycles (not 256)?  We're halving the resolution of the power output.
  // - Why not make software PWM be a compile time option for the rare cases its needed?
  
  // Software PWM thresholds
  static uint8_t soft_pwm_0;
  static uint8_t soft_pwm_1;
  static uint8_t soft_pwm_2;
  // PWM counter, incremented each interrupt (every 1ms)
  static uint8_t pwm_count = 1;
  
  // Initialization or beginning of new period
  // Update the PWM thresholds for this period
  // Note that the threshholds are not updated until the next cycle

  if (pwm_count == 0) { 
    soft_pwm_0 = soft_pwm[0];
    if(soft_pwm_0 > 0) WRITE(HEATER_0_PIN,1);
    #if EXTRUDERS > 1
    soft_pwm_1 = soft_pwm[1];
    if(soft_pwm_1 > 0) WRITE(HEATER_1_PIN,1);
    #endif
    #if EXTRUDERS > 2
    soft_pwm_2 = soft_pwm[2];
    if(soft_pwm_2 > 0) WRITE(HEATER_2_PIN,1);
    #endif
  }
  if(soft_pwm_0 <= pwm_count) WRITE(HEATER_0_PIN,0);
  #if EXTRUDERS > 1
  if(soft_pwm_1 <= pwm_count) WRITE(HEATER_1_PIN,0);
  #endif
  #if EXTRUDERS > 2
  if(soft_pwm_2 <= pwm_count) WRITE(HEATER_2_PIN,0);
  #endif
  
  pwm_count++;
  pwm_count &= 0x7f; // Wrap to 0 at 128, making period 128 interrupts
 
  // On each pass through the ISR different functions are performed.
  // The select statement implements a simple state machine with 8 states
  // +-------+---------------------+
  // | State | Action              |
  // +-------+---------------------+
  // |   0   | Start ADC  TEMP_0   |
  // |   1   | Accumulate TEMP_0   |
  // |   2   | Start ADC  TEMP_BED |
  // |   3   | Accumulate TEMP_BED |
  // |   4   | Start ADC  TEMP_1   |
  // |   5   | Accumulate TEMP_1   |
  // |   6   | Start ADC  TEMP_2   |
  // |   7   | Accumulate TEMP_2   |
  // +-------+---------------------+
 
  switch (temp_state) {
    case 0:
      // Start ADC measurement for TEMP_0
      // Handle display (if present)
      #if (TEMP_0_PIN > -1)
        #if TEMP_0_PIN > 7
          ADCSRB = 1 << MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_0_PIN & 0x07));
        ADCSRA |= 1 << ADSC; // Start conversion
      #endif
      #ifdef ULTIPANEL
        buttons_check();
      #endif
      temp_state = 1;
      break;
    case 1:
      // Measure TEMP_0, accumulate raw value
      #if (TEMP_0_PIN > -1)
        temp_0_raw += ADC;
      #endif
      temp_state = 2;
      break;
    case 2:
      // Start ADC measurement for TEMP_BED
      // Handle display (if present)
      #if (TEMP_BED_PIN > -1)
        #if TEMP_BED_PIN > 7
          ADCSRB = 1 << MUX5;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_BED_PIN & 0x07));
        ADCSRA |= 1 << ADSC; // Start conversion
      #endif
      #ifdef ULTIPANEL
        buttons_check();
      #endif
      temp_state = 3;
      break;
    case 3:
      // Measure TEMP_BED, accumulate raw value
      #if (TEMP_BED_PIN > -1)
        temp_bed_raw += ADC;
      #endif
      temp_state = 4;
      break;
    case 4:
      // Start ADC measurement for TEMP_1
      // Handle display (if present)
      #if (TEMP_1_PIN > -1)
        #if TEMP_1_PIN > 7
          ADCSRB = 1 << MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_1_PIN & 0x07));
        ADCSRA |= 1 << ADSC; // Start conversion
      #endif
      #ifdef ULTIPANEL
        buttons_check();
      #endif
      temp_state = 5;
      break;
    case 5:
      // Measure TEMP_1, accumulate raw value
      #if (TEMP_1_PIN > -1)
        temp_1_raw += ADC;
      #endif
      temp_state = 6;
      break;
    case 6:
      // Start ADC measurement for TEMP_2
      // Handle display (if present)
      #if (TEMP_2_PIN > -1)
        #if TEMP_2_PIN > 7
          ADCSRB = 1 << MUX5;
        #else
          ADCSRB = 0;
        #endif
        ADMUX = ((1 << REFS0) | (TEMP_2_PIN & 0x07));
        ADCSRA |= 1 << ADSC; // Start conversion
      #endif
      #ifdef ULTIPANEL
        buttons_check();
      #endif
      temp_state = 7;
      break;
    case 7:
    // Measure TEMP_2, accumulate raw value
      #if (TEMP_2_PIN > -1)
        temp_2_raw += ADC;
      #endif
      temp_state = 0;
      temp_measurements++;
      break;
//    default:
//      SERIAL_ERROR_START;
//      SERIAL_ERRORLNPGM("Temp measurement error!");
//      break;
  } // switch

  // In an attempt to reduce measurement noise OVERSAMPLENR raw
  // ADC measurements are accumulated in successive ISR visits.
  // The accumulated raw values are not averaged.  Instead the
  // temperature lookup tables used to convert raw ADC values to
  // to temperature are scaled by OVERSAMPLENR (see thermistortables.h)
  //
  // Since the ADC measurements are 10-bit resolution [0-1023] 
  // then we can continue to use uint16_t as the data type for
  // the raw values in the lookup table as long as OVERSAMPLENR <=16.
  // 
  // It take 8ms to get measurements for all 3 hotends and the heatbed
  // With OVERSAMPLENR=16, we visit this every 8ms * 16 = 128ms
 
  if (temp_measurements >= OVERSAMPLENR) {
    #ifdef HEATER_0_USES_AD595
      temp_current_raw[0] = temp_0_raw;
    #else
      temp_current_raw[0] = temp_0_raw;
      //temp_current_raw[0] = OVERSAMPLENR * 1023 - temp_0_raw;
    #endif

    #if EXTRUDERS > 1    
      #ifdef HEATER_1_USES_AD595
        temp_current_raw[1] = temp_1_raw;
      #else
        temp_current_raw[1] = temp_1_raw;
        //temp_current_raw[1] = OVERSAMPLENR * 1023 - temp_1_raw;
      #endif
    #endif
    
    #if EXTRUDERS > 2
      #ifdef HEATER_2_USES_AD595
        temp_current_raw[2] = temp_2_raw;
      #else
        temp_current_raw[2] = temp_2_raw;
        //temp_current_raw[2] = OVERSAMPLENR * 1023 - temp_2_raw;
      #endif
    #endif
    
    #ifdef BED_USES_AD595
      temp_current_bed_raw = temp_bed_raw;
    #else
      // FIXME: This makes no sense at all...
      // The comment in the sprinter firmware (which presumably is where this comes from) is as follows:
      //   "If using thermistor, when the heater is colder than targer temp, we get a higher analog reading than target, 
      //   this switches it up so that the reading appears lower than target for the control logic."
      //  I think we should just be using the raw temperature, 
      //  so commenting this out for now
      temp_current_bed_raw = temp_bed_raw;
      // temp_current_bed_raw = OVERSAMPLENR * 1023 - temp_bed_raw;
    #endif

#if 0    
    // Over- and under-temperature protection for extruders
    for (uint8_t e = 0; e < EXTRUDERS; e++) {
      // Over-temperature
      if (temp_current_raw[e] >= maxttemp[e]) {
          target_raw[e] = 0;
          max_temp_error(e);
          kill();
       }
       // Under-temperature
       // This is useful in case the thermistor short circuits
       if (temp_current_raw[e] <= mintemp[e]) {
          target_raw[e] = 0;
          min_temp_error(e);
          kill();
       }
    }
#endif

   // Over- and under-temperature protection for heater bed
#if defined(BED_MAXTEMP) && (HEATER_BED_PIN > -1)
    if (temp_current_bed_raw >= bed_maxttemp) {
       temp_target_bed_raw = 0;
       bed_max_temp_error();
       kill();
    }
#endif

    // reset count and accumulators
    temp_measurements = 0;
    temp_0_raw = 0;
    temp_1_raw = 0;
    temp_2_raw = 0;
    temp_bed_raw = 0;

    temp_measurement_avail = true; // valid measurement available
  }
}
