#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

//===========================================================================
//======================== Serial Communications ============================
//===========================================================================

// Communication speed for the serial link to the printer
//#define BAUDRATE 250000 // Marlin default
#define BAUDRATE 115200 // eMaker Huxley
//#define BAUDRATE 230400

//// Frequency limit
// See nophead's blog for more info
// Not working O
//#define XY_FREQUENCY_LIMIT 15

//===========================================================================
//============================ Path Planner =================================
//===========================================================================

// Minimum planner junction speed
//
// Sets the default minimum speed the planner plans for at the end of the 
// buffer and all stops. This should not be much greater than zero and should
// only be changed if unwanted behavior is observed on a user's machine when 
// running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 2.0 // (mm/sec)

//// MACHINE CONFIGURAION
// BASIC SETTINGS: select your board type, thermistor type, axis scaling, and endstop configuration
//   - controller board type
//   - temperature sensor types (thermistors, thermocouples, etc)
//   - axis scaling
//   - endstop configuration


//===========================================================================
//========================== Controller Board ===============================
//===========================================================================

//// Controller Board Type
//
// The following define selects which controller board you have.
//  Arduino Mega / RAMPS (up to v1.2) = 3
//  Arduino Mega / RAMPS (v1.3) = 33
//  Gen6 = 5
//  Ultimaker = 7
//  Teensylu = 8
//  Sanguinololu (v1.2 and above) = 62

//#define MOTHERBOARD 7 // Marlin default
#define MOTHERBOARD 62 //  Sanguinololu 1.3 (eMaker Huxley, others)

//===========================================================================
//=================== Temperature Sensing and Control =======================
//===========================================================================

// Marlin currently supports
// - Up to 3 extruder heater elements (hotend)
// - 1 heated platform (hotbed)

#define EXTRUDERS 1 // Marlin default

// Hotend and hotbed temperature sensors types
//
// Two types of sensor are commonly used to measure the
// temperature of the hotend and the hotbed:
// - Thermistors   (used by RepRap Mendel, Huxley, etc), and
// - Thermocouples (used by Ultimaker and other newer printers)

//=============================
// Extruder temperature sensors
//=============================

// If your printer uses thermistors
//#define HEATER_0_USES_THERMISTOR
//#define HEATER_1_USES_THERMISTOR
//#define HEATER_2_USES_THERMISTOR

#define HEATER_0_USES_THERMISTOR // eMaker Huxley, others

// If your printer  uses thermocouples (using AD595/AD597 thermocouple amplifiers)
//#define HEATER_0_USES_AD595 // Marlin default
//#define HEATER_1_USES_AD595
//#define HEATER_2_USES_AD595

//=================
// Thermistor types
//=================
// A variety of thermistors are used by various printers
// Note that a printer may use more than one type
// for the hotend and the heated bed.  Be sure to select the
// right one for each.  The following thermistor types are supported:
//
// 1: 100K thermistor (FIXME: Add part number)
// 2: 200K thermistor (FIXME: Add part number)
// 3: Mendel-parts thermistor (FIXME: Add part number)
// 4: 10K thermistor (FIXME: Add part number)
// 5: ParCan supplied 104GT-2 100K
// 6: EPCOS 100K (FIXME: Add part number)
// 7: 100K Honeywell thermistor 135-104LAG-J01

//#define HEATER_0_THERMISTOR 3
//#define HEATER_1_THERMISTOR 1
//#define HEATER_2_THERMISTOR 1

#define HEATER_0_THERMISTOR 1 // eMaker Huxley

//===============================
// Hotbed temperature sensor type
//===============================
// Select only *one* of the following to define how the hotbed temp is read

#define BED_USES_THERMISTOR // eMaker Huxley
//#define BED_USES_AD595 // Marlin default

//=======================
// Hotbed thermistor type
//=======================
// If the hotbed uses a thermistor, select the thermistor type (see table above)
#define BED_THERMISTOR 1 // eMaker Huxley, Mendel

//=======================
// Hotbed limit switching
//=======================
// Only disable heating if T > target + BED_HYSTERESIS and
//       enable heating if T > target - BED_HYSTERESIS

//#define BED_LIMIT_SWITCHING
#ifdef BED_LIMIT_SWITCHING
  // hotbed temperature hysteresis band in degrees C
  #define BED_HYSTERESIS 2 // [degrees C]
#endif 

// Check hotbed temperature every so often
#define BED_CHECK_INTERVAL 5000 // [milliseconds]

//=====================
// Heating sanity check
//=====================
// This waits for the hotbed watch period in milliseconds whenever an M104 or M109 increases the target temperature
// If the temperature has not increased at the end of that period, the target temperature is set to zero. 
// It can be reset with another M104/M109
//#define WATCHPERIOD 20*1000 // [milliseconds]

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 30  // [seconds]
#define TEMP_HYSTERESIS 3       // [degrees C] range of +/- temperatures considered "close" to the target temp

//// The minimal temperature defines the temperature below which the heater will not be enabled
#define HEATER_0_MINTEMP 5
//#define HEATER_1_MINTEMP 5
//#define HEATER_2_MINTEMP 5
#define BED_MINTEMP 5

//===========================
// Maximum temperature cutoff
//===========================
// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 275
//#define HEATER_1_MAXTEMP 275
//#define HEATER_2_MAXTEMP 275
//#define BED_MAXTEMP 150

//==================
// Wait for cooldown
//==================
// This defines if the M109 call should not block if it is cooling down.
// Example: From a current temp of 220, you set M109 S200. 
// If COOL_DOWN_NO_WAIT is set to "true" M109 will not wait for the cooldown to finish
// FIXME: Should this be a #define or a const bool?
#define COOL_DOWN_NO_WAIT true

// Heating is finished if a temperature close to this degree shift is reached
#define HEATING_EARLY_FINISH_DEG_OFFSET 1 // [degree C]

//=============
// PID settings
//=============

#define PIDTEMP // Uncomment this line to enable PID support

#ifdef PIDTEMP
  #define PID_DEBUG // Sends debug data to the serial port. 
  //#define PID_OPENLOOP 1 // Puts PID in open loop. M104 sets the output power in %

  #define PID_INTEGRAL_DRIVE_MAX 255  // Limit for the integral term
  #define PID_K1 0.95 // Smoothing factor used in the PID

  // Sampling period of the PID [seconds]
  // FIXME: This really should be 8 * OVERSAMPLENR
  #define PID_dT 0.128 
                       
  #define PID_MAX 255 // Maximum heater output (255=full current)
  #define PID_ERR_BIG 10 // If temp error [deg C] is > +PID_ERR_BIG set output to PID_MAX
                         //                          < -PID_ERR_BIG set output to 0

  // To determine PID settings for your machine heuristically, you
  // can use the Ziegler-Nichols method.
  // See http://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
  // 
  // 1. Set Ki and Kd to zero 
  // 2. Heat with a defined Kp and see if the temperature stabilizes
  //    Ideally you do this graphically with ReplicatorG
  // 3. The PID_CRITICAL_GAIN should be the value of Kp at which temperature
  //    oscillations are not damped out / decrease in amplitude
  // 4. PID_SWING_AT_CRITICAL is the time for a full period of the oscillations
  //    at the critical Gain
  // 5. Usually further manual tuning is necessary.

  #define PID_CRITICAL_GAIN 50
  #define PID_SWING_AT_CRITICAL 47 // [seconds]
  
  //#define PID_PI // Use P,I controller
  #define PID_PID // Use P,I,D controller
  
  #ifdef PID_PID
    // PID according to Ziegler-Nichols method
    // See http://en.wikipedia.org/wiki/PID_controller
    // 
//    #define DEFAULT_Kp (0.6 * PID_CRITICAL_GAIN)
//    #define DEFAULT_Ki ((2 * Kp / PID_SWING_AT_CRITICAL) * PID_dT)  
//    FIXME: I think the following line is incorrect...
//    #define DEFAULT_Kd (       PID_SWING_AT_CRITICAL / 8. / PID_dT)  
//    It should instead be:
//    #define DEFAULT_Kd (((Kp * PID_SWING_AT_CRITICAL) / 8.0) / PID_dT)  

// Ultimaker
//    #define DEFAULT_Kp  22.2
//    #define DEFAULT_Ki ( 1.25 * PID_dT)  
//    #define DEFAULT_Kd (99.0  / PID_dT)  

// Makergear
//    #define  DEFAULT_Kp 7.0
//    #define  DEFAULT_Ki 0.1  
//    #define  DEFAULT_Kd 12  

// Mendel Parts V9 on 12V    
//    #define  DEFAULT_Kp  63.0
//    #define  DEFAULT_Ki (2.25*PID_dT)  
//    #define  DEFAULT_Kd (440/PID_dT)  

// eMaker Huxley
// FIXME: No idea what these should be just yet
//        Sprinter seems to use a different PID 
//        so the parameters used for sprinter are
//        probably incorrect here.
    #define  DEFAULT_Kp 20
    #define  DEFAULT_Ki 0  
    #define  DEFAULT_Kd 0  

  #endif
   
  #ifdef PID_PI
    // PI according to Ziegler-Nichols method
    #define  DEFAULT_Kp (PID_CRITICAL_GAIN/2.2) 
    #define  DEFAULT_Ki (1.2*Kp/PID_SWING_AT_CRITICAL*PID_dT)
    #define  DEFAULT_Kd (0)
  #endif
  
  // This adds an experimental additional term to the heating power, proportional to the extrusion speed.
  // If Kc is choosen well, the additional required power due to increased melting should be compensated.
  #define PID_ADD_EXTRUSION_RATE  
  #ifdef PID_ADD_EXTRUSION_RATE
    #define  DEFAULT_Kc (1) //heatingpower=Kc*(e_speed)
  #endif
#endif // PIDTEMP

//  extruder run-out prevention. 
//if the machine is idle, and the temperature over MINTEMP, every couple of SECONDS some filament is extruded
//#define EXTRUDER_RUNOUT_PREVENT  
#define EXTRUDER_RUNOUT_MINTEMP 190  
#define EXTRUDER_RUNOUT_SECONDS 30.
#define EXTRUDER_RUNOUT_ESTEPS 14. //mm filament
#define EXTRUDER_RUNOUT_SPEED 1500.  //extrusion speed
#define EXTRUDER_RUNOUT_EXTRUDE 100

//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// Endstop Settings

// Endstop pull-up resistors
// Comment this out (using // at the start of the line) to disable the endstop pullup resistors
// The pullups are needed if you directly connect a mechanical limit switch between the signal and ground pins.
// Required for eMaker Huxley
#define ENDSTOPPULLUPS 

// Invert logic of endstops
// 
// For Optos H21LOB set to true
// For Mendel-Parts, newer optos TCST2103, eMaker Huxley set to false

//const bool X_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops. 
//const bool Y_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops. 
//const bool Z_ENDSTOPS_INVERTING = true; // set to true to invert the logic of the endstops. 

const bool X_ENDSTOPS_INVERTING = false; // set to true to invert the logic of the endstops. 
const bool Y_ENDSTOPS_INVERTING = false; // set to true to invert the logic of the endstops. 
const bool Z_ENDSTOPS_INVERTING = false; // set to true to invert the logic of the endstops. 

#define ENDSTOPS_ONLY_FOR_HOMING // If defined the endstops will only be used for homing

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
// For Sanguinololu (including eMaker Huxley) these should all be defined as 0
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

//======================================
// Disable axis when it's not being used
//======================================
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false // This value applies to all extruders

#define DISABLE_X false // For Mendel, eMaker Huxley
#define DISABLE_Y false // For Mendel, eMaker Huxley
#define DISABLE_Z true  // For Mendel, eMaker Huxley
#define DISABLE_E false // For Mendel, eMaker Huxley

//=========================
// Inverting axis direction
//=========================

//#define INVERT_X_DIR false    // for Mendel set to false, for Orca set to true
//#define INVERT_Y_DIR true   // for Mendel set to true, for Orca set to false
//#define INVERT_Z_DIR false    // for Mendel set to false, for Orca set to true
//#define INVERT_E*_DIR true   // for direct drive extruder v9 set to true, for geared extruder set to false, used for all extruders

#define INVERT_X_DIR true    // for Mendel set to false, for Orca set to true
#define INVERT_Y_DIR false    // for Mendel set to true, for Orca set to false
#define INVERT_Z_DIR true     // for Mendel set to false, for Orca set to true
#define INVERT_E0_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E1_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E2_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false

// For eMaker Huxley
#define INVERT_X_DIR true
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false
#define INVERT_E0_DIR true

//=================
// Endstop settings
//=================

//======================================================
// Sets direction of endstops when homing; 1=MAX, -1=MIN
//======================================================
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

//==================
// Software endstops
//==================
#define min_software_endstops true // If true, axis won't move to coordinates less than zero
#define max_software_endstops true // If true, axis won't move to coordinates greater than those defined below

//=============================================
// Maximum X,Y,Z axis coordinates [millimeters]
//=============================================

// Ultimaker
#define X_MAX_LENGTH 205
#define Y_MAX_LENGTH 205
#define Z_MAX_LENGTH 200

// eMaker Huxley
#define X_MAX_LENGTH 150
#define Y_MAX_LENGTH 148
#define Z_MAX_LENGTH 100

//==================
// Movement settings
//==================

#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E
#define HOMING_FEEDRATE {50*60, 50*60, 4*60, 0}  // set the homing speeds (mm/min)

//==========================================
// Endstop retraction distance [millimeters]
//==========================================
// After finding the endstop, the axis retracts by this
// distance before returning to the endstop at low speed.
#define X_HOME_RETRACT_MM 5 
#define Y_HOME_RETRACT_MM 5 
#define Z_HOME_RETRACT_MM 1 

// If defined, a diagonal move will be performed if both X and X are to be homed
#define QUICK_HOME 

#define AXIS_RELATIVE_MODES {false, false, false, false}

//===========================================
// Maximum step frequency [pulses per second]
//===========================================
// Sets the maximum number of pulses that will be sent per second
// to any stepper motor
#define MAX_STEP_FREQUENCY 40000 // Max step frequency for Ultimaker (5000 pps / half step)

//=============
// Steps per mm
//=============
// Defines how many step signals are required to move 1mm
//#define DEFAULT_AXIS_STEPS_PER_UNIT   {78.7402, 78.7402, 200*8/3,   760*1.1 } // Ultimaker 
//#define DEFAULT_AXIS_STEPS_PER_UNIT   {40,      40,      3333.92,   360     } // Sells Mendel with v9 extruder
//#define DEFAULT_AXIS_STEPS_PER_UNIT   {80.3232, 80.8900, 2284.7651, 757.2218} // SAE Prusa w/ Wade extruder
#define DEFAULT_AXIS_STEPS_PER_UNIT   {91.4286, 91.4286, 4000,      910     } // eMaker Huxley

//=========================================
// Minimum and maximum feedrate [mm/second]
//=========================================
// The value for E-Axis applies to all extruders

#define DEFAULT_MINIMUMFEEDRATE       0.0 // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0

//#define DEFAULT_MAX_FEEDRATE          {500, 500, 5, 45 } // Ultimaker [mm/sec]
#define DEFAULT_MAX_FEEDRATE          {325, 325, 3, 33 }; // eMaker Huxley <<< FIXME

//==================================
// Maximum start speed [mm/second^2]
//==================================
// X, Y, Z, E maximum start speed for accelerated moves
// E default values are good for skeinforge 40+, for older versions raise them a lot.
#define DEFAULT_MAX_ACCELERATION      {9000, 9000, 100, 10000}   
#define DEFAULT_ACCELERATION          3000  // X, Y, Z and E max acceleration [mm/s^2] for printing moves 
#define DEFAULT_RETRACT_ACCELERATION  3000  // X, Y, Z and E max acceleration [mm/s^2] for r retracts


// Minimum time in microseconds that a movement needs to take if the buffer is emptied.
// Increase this number if you see blobs while printing high speed & high detail.
// It will slowdown on the detailed stuff.
#define DEFAULT_MINSEGMENTTIME        20000   // Obsolete delete this
#define DEFAULT_XYJERK                20.0    // [mm/sec]
#define DEFAULT_ZJERK                 0.4     // [mm/sec]

// If defined the movements slow down when the look ahead buffer is only half full
#define SLOWDOWN

//================================
// Default stepper release if idle
//================================
#define DEFAULT_STEPPER_DEACTIVE_TIME 60

//#define DEFAULT_STEPPER_DEACTIVE_COMMAND "M84 X Y E"  // Ultimaker. Z stays  powered
#define DEFAULT_STEPPER_DEACTIVE_COMMAND "M84 X Y Z E"  // eMaker Huxley


//===========================================================================
//=============================Additional Features===========================
//===========================================================================

//=======
// EEPROM
//=======
// The microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).  
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
// Define this to enable EEPROM support
#define EEPROM_SETTINGS
// To disable EEPROM serial responses and decrease program space by ~1700 bytes
// comment this out.  Please keep turned on if you can.
#define EEPROM_CHITCHAT

//==================
// Hardware watchdog
//==================
// The hardware watchdog should interrupt the microcontroller in case the firmware
// gets stuck somewhere. However, the watchdog is not working well, so please
// only enable this for testing.
//#define USE_WATCHDOG
//#ifdef USE_WATCHDOG
  // yYou can't reboot on a mega2560 due to a bug in he bootloader
  // Hence, you have to reset manually, and this is done hereby:
  //#define RESET_MANUAL
  //#define WATCHDOG_TIMEOUT 4 // [seconds]
//#endif

//===================================
// extruder advance constant (s2/mm3)
//===================================
//
// advance (steps) = STEPS_PER_CUBIC_MM_E * EXTUDER_ADVANCE_K * cubic mm per second ^ 2
//
// Hooke's law:           force = k * distance
// Bernoulli's principle: v^2/2 + g.h + pressure/density = constant
// so: v ^ 2 is proportional to number of steps we advance the extruder
//#define ADVANCE

#ifdef ADVANCE
  #define EXTRUDER_ADVANCE_K .0
  #define D_FILAMENT 2.85
  #define STEPS_MM_E 836
  #define EXTRUSION_AREA (0.25 * D_FILAMENT * D_FILAMENT * 3.14159)
  #define STEPS_PER_CUBIC_MM_E (axis_steps_per_unit[E_AXIS]/ EXTRUSION_AREA)
#endif // ADVANCE

//===================
// LCD and SD support
//===================
//#define ULTRA_LCD  //general lcd support, also 16x2
//#define SDSUPPORT // Enable SD Card Support in Hardware Console
#define SD_FINISHED_STEPPERRELEASE true  //if sd support and the file is finished: disable steppers?
#define SD_FINISHED_RELEASECOMMAND "M84 X Y E" // no z because of layer shift.

//#define ULTIPANEL
#ifdef ULTIPANEL
  //#define NEWPANEL  //enable this if you have a click-encoder panel
  #define SDSUPPORT
  #define ULTRA_LCD
  #define LCD_WIDTH 20
  #define LCD_HEIGHT 4
#else // No panel but just LCD
  #ifdef ULTRA_LCD
    #define LCD_WIDTH 16
    #define LCD_HEIGHT 2
  #endif
#endif

// A debugging feature to compare calculated vs performed steps, to see if steps are lost by the software.
//#define DEBUG_STEPS


// Arc interpretation settings:
#define MM_PER_ARC_SEGMENT 1
#define N_ARC_CORRECTION 25

//========================
// Automatic temperature
//========================
// The hot end target temperature is calculated by all the buffered lines of gcode.
// The maximum buffered steps/sec of the extruder motor are called "se".
// You enter the autotemp mode by a M109 S<mintemp> T<maxtemp> F<factor>
// the target temperature is set to mintemp+factor*se[steps/sec] and limited by mintemp and maxtemp
// you exit the value by any M109 without F*
// Also, if the temperature is set to a value <mintemp, it is not changed by autotemp.
// on an ultimaker, some initial testing worked with M109 S215 T260 F0.1 in the start.gcode
//#define AUTOTEMP
#ifdef AUTOTEMP
  #define AUTOTEMP_OLDWEIGHT 0.98
#endif

// This prevents dangerous Extruder moves, i.e. if the temperature is under the limit
// can be software-disabled for whatever purposes by
#define PREVENT_DANGEROUS_EXTRUDE
#define EXTRUDE_MINTEMP 190
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH + Y_MAX_LENGTH) //prevent extrusion of very large distances.

const int dropsegments=5; //everything with less than this number of steps will be ignored as move and joined with the next movement

// M240  Triggers a camera by emulating a Canon RC-1 Remote
// Data from: http://www.doc-diy.net/photo/rc-1_hacked/
// #define PHOTOGRAPH_PIN 23

//===========================================================================
//================================= Buffers =================================
//===========================================================================

// The number of linear motions that can be in the plan at any give time.  
// THE BLOCK_BUFFER_SIZE NEEDS TO BE A POWER OF 2, i.g. 8,16,32 because shifts and ors are used to do the ringbuffering.
#if defined SDSUPPORT
  #define BLOCK_BUFFER_SIZE 16   // SD,LCD,Buttons take more memory, block buffer needs to be smaller
#else
  #define BLOCK_BUFFER_SIZE 16 // maximize block buffer
#endif

//The ASCII buffer for receiving from the serial:
#define MAX_CMD_SIZE 96
#define BUFSIZE 4

#include "thermistortables.h"

#endif //__CONFIGURATION_H
