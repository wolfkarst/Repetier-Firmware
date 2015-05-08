/*
    This file is part of Repetier-Firmware.

    Repetier-Firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Repetier-Firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Repetier-Firmware.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/* Some words on units:

From 0.80 onwards the units used are unified for easier configuration, watch out when transfering from older configs!

Speed is in mm/s
Acceleration in mm/s^2
Temperature is in degrees celsius


##########################################################################################
##                                        IMPORTANT                                     ##
##########################################################################################

For easy configuration, the default settings enable parameter storage in EEPROM.
This means, after the first upload many variables can only be changed using the special
M commands as described in the documentation. Changing these values in the configuration.h
file has no effect. Parameters overriden by EEPROM settings are calibartion values, extruder
values except thermistor tables and some other parameter likely to change during usage
like advance steps or ops mode.
To override EEPROM settings with config settings, set EEPROM_MODE 0

*/


// BASIC SETTINGS: select your board type, thermistor type, axis scaling, and endstop configuration

/** Define the to-be-use micro steps */
#define	RF1000_MICRO_STEPS	32


/** Number of extruders. Maximum 6 extruders. */
#define NUM_EXTRUDER 1

//// The following define selects which electronics board you have. Please choose the one that matches your setup
// Gen3 PLUS for RepRap Motherboard V1.2 = 21
// MEGA/RAMPS up to 1.2       = 3
// RAMPS 1.3/RAMPS 1.4        = 33
// Azteeg X3                  = 34
// Ultimaker Shield 1.5.7     = 37
// Gen6                       = 5
// Gen6 deluxe                = 51
// Sanguinololu up to 1.1     = 6
// Sanguinololu 1.2 and above = 62
// Melzi board                = 63  // Define REPRAPPRO_HUXLEY if you have one for correct HEATER_1_PIN assignment!
// Gen7 1.1 till 1.3.x        = 7
// Gen7 1.4.1 and later       = 71
// Sethi 3D_1                 = 72
// Teensylu (at90usb)         = 8 // requires Teensyduino
// Printrboard (at90usb)      = 9 // requires Teensyduino
// Foltyn 3D Master           = 12
// Conrad RF1000			  = 13
// MegaTronics 1.0            = 70
// Megatronics 2.0            = 701
// RUMBA                      = 80  // Get it from reprapdiscount
// FELIXprinters              = 101
// Rambo                      = 301
// PiBot for Repetier V1.0-1.3= 314
// PiBot for Repetier V1.4    = 315
// Sanguish Beta              = 501
// Unique One rev. A          = 88
// User layout defined in userpins.h = 999

#define MOTHERBOARD		13
#define PROTOTYPE_PCB	 0	// 1 = first PCB's / 0 = Final

#include "pins.h"

// Override pin definions from pins.h
//#define FAN_PIN   4  // Extruder 2 uses the default fan output, so move to an other pin
//#define EXTERNALSERIAL  use Arduino serial library instead of build in. Requires more ram, has only 63 byte input buffer.

// Uncomment the following line if you are using arduino compatible firmware made for Arduino version earlier then 1.0
// If it is incompatible you will get compiler errors about write functions not beeing compatible!
//#define COMPAT_PRE1

/* Define the type of axis movements needed for your printer. The typical case
is a full cartesian system where x, y and z moves are handled by separate motors.

0 = full cartesian system, xyz have seperate motors.
1 = z axis + xy H-gantry (x_motor = x+y, y_motor = x-y)
2 = z axis + xy H-gantry (x_motor = x+y, y_motor = y-x)
3 = Delta printers (Rostock, Kossel, RostockMax, Cerberus, etc)
4 = Tuga printer (Scott-Russell mechanism)
5 = Bipod system (not implemented)
Cases 1 and 2 cover all needed xy H gantry systems. If you get results mirrored etc. you can swap motor connections for x and y.
If a motor turns in the wrong direction change INVERT_X_DIR or INVERT_Y_DIR.
*/
#define DRIVE_SYSTEM 0

// ##########################################################################################
// ##                               Calibration                                            ##
// ##########################################################################################

/** Drive settings for the Delta printers
*/
#if DRIVE_SYSTEM==3
    // ***************************************************
    // *** These parameter are only for Delta printers ***
    // ***************************************************

/** \brief Delta drive type: 0 - belts and pulleys, 1 - filament drive */
#define DELTA_DRIVE_TYPE 0

#if DELTA_DRIVE_TYPE == 0
/** \brief Pitch in mm of drive belt. GT2 = 2mm */
#define BELT_PITCH 2
/** \brief Number of teeth on X, Y and Z tower pulleys */
#define PULLEY_TEETH 20
#define PULLEY_CIRCUMFERENCE (BELT_PITCH * PULLEY_TEETH)
#elif DELTA_DRIVE_TYPE == 1
/** \brief Filament pulley diameter in milimeters */
#define PULLEY_DIAMETER 10
#define PULLEY_CIRCUMFERENCE (PULLEY_DIAMETER * 3.1415927)
#endif

/** \brief Steps per rotation of stepper motor */
#define STEPS_PER_ROTATION 200

/** \brief Micro stepping rate of X, Y and Y tower stepper drivers */
#define MICRO_STEPS 16

// Calculations
#define AXIS_STEPS_PER_MM ((float)(MICRO_STEPS * STEPS_PER_ROTATION) / PULLEY_CIRCUMFERENCE)
#define XAXIS_STEPS_PER_MM AXIS_STEPS_PER_MM
#define YAXIS_STEPS_PER_MM AXIS_STEPS_PER_MM
#define ZAXIS_STEPS_PER_MM AXIS_STEPS_PER_MM
#else
// *******************************************************
// *** These parameter are for all other printer types ***
// *******************************************************

/** Drive settings for printers with cartesian drive systems */
/** \brief Number of steps for a 1mm move in x direction.
For xy gantry use 2*belt moved!
Overridden if EEPROM activated. */
#define XAXIS_STEPS_PER_MM	long(4.761875 * (float)RF1000_MICRO_STEPS)
/** \brief Number of steps for a 1mm move in y direction.
For xy gantry use 2*belt moved!
Overridden if EEPROM activated.*/
#define YAXIS_STEPS_PER_MM	long(4.761875 * (float)RF1000_MICRO_STEPS)
/** \brief Number of steps for a 1mm move in z direction  Overridden if EEPROM activated.*/
#define ZAXIS_STEPS_PER_MM	long(80 * (float)RF1000_MICRO_STEPS)
#endif

// ##########################################################################################
// ##                           Extruder configuration                                     ##
// ##########################################################################################

// for each extruder, fan will stay on until extruder temperature is below this value
#define EXTRUDER_FAN_COOL_TEMP 50

#define EXT0_X_OFFSET 0
#define EXT0_Y_OFFSET 0
// for skeinforge 40 and later, steps to pull the plasic 1 mm inside the extruder, not out.  Overridden if EEPROM activated.
#define EXT0_STEPS_PER_MM	(8.75 * RF1000_MICRO_STEPS)

// What type of sensor is used?
// 1 is 100k thermistor (Epcos B57560G0107F000 - RepRap-Fab.org and many other)
// 2 is 200k thermistor
// 3 is mendel-parts thermistor (EPCOS G550)
// 4 is 10k thermistor
// 8 is ATC Semitec 104GT-2
// 5 is userdefined thermistor table 0
// 6 is userdefined thermistor table 1
// 7 is userdefined thermistor table 2
// 50 is userdefined thermistor table 0 for PTC thermistors
// 51 is userdefined thermistor table 0 for PTC thermistors
// 52 is userdefined thermistor table 0 for PTC thermistors
// 60 is AD8494, AD8495, AD8496 or AD8497 (5mV/degC and 1/4 the price of AD595 but only MSOT_08 package)
// 97 Generic thermistor table 1
// 98 Generic thermistor table 2
// 99 Generic thermistor table 3
// 100 is AD595
// 101 is MAX6675
// 102 is MAX31855
#define EXT0_TEMPSENSOR_TYPE 3
// Analog input pin for reading temperatures or pin enabling SS for MAX6675
#define EXT0_TEMPSENSOR_PIN TEMP_0_PIN
// Which pin enables the heater
#define EXT0_HEATER_PIN HEATER_0_PIN
#define EXT0_STEP_PIN E0_STEP_PIN
#define EXT0_DIR_PIN E0_DIR_PIN
// set to false/true for normal / inverse direction
#define EXT0_INVERSE false
#define EXT0_ENABLE_PIN E0_ENABLE_PIN
// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define EXT0_ENABLE_ON true
// The following speed settings are for skeinforge 40+ where e is the
// length of filament pulled inside the heater. For repsnap or older
// skeinforge use higher values.
//  Overridden if EEPROM activated.
#define EXT0_MAX_FEEDRATE 25
// Feedrate from halted extruder in mm/s
//  Overridden if EEPROM activated.
#define EXT0_MAX_START_FEEDRATE 18
// Acceleration in mm/s^2
//  Overridden if EEPROM activated.
#define EXT0_MAX_ACCELERATION 6000
/** Type of heat manager for this extruder.
- 0 = Simply switch on/off if temperature is reached. Works always.
- 1 = PID Temperature control. Is better but needs good PID values. Defaults are a good start for most extruder.
- 3 = Dead-time control. PID_P becomes dead-time in seconds.
 Overridden if EEPROM activated.
*/
#define EXT0_HEAT_MANAGER 1
/** Wait x seconds, after reaching target temperature. Only used for M109.  Overridden if EEPROM activated. */
#define EXT0_WATCHPERIOD 20

/** \brief The maximum value, I-gain can contribute to the output.

A good value is slightly higher then the output needed for your temperature.
Values for starts:
130 => PLA for temperatures from 170-180 deg C
180 => ABS for temperatures around 240 deg C

The precise values may differ for different nozzle/resistor combination.
 Overridden if EEPROM activated.
*/
#define EXT0_PID_INTEGRAL_DRIVE_MAX 180
/** \brief lower value for integral part

The I state should converge to the exact heater output needed for the target temperature.
To prevent a long deviation from the target zone, this value limits the lower value.
A good start is 30 lower then the optimal value. You need to leave room for cooling.
 Overridden if EEPROM activated.
*/
#define EXT0_PID_INTEGRAL_DRIVE_MIN 40
/** P-gain.  Overridden if EEPROM activated. */
#define EXT0_PID_P   20
/** I-gain. Overridden if EEPROM activated.
*/
#define EXT0_PID_I   5
/** Dgain.  Overridden if EEPROM activated.*/
#define EXT0_PID_D 13
// maximum time the heater is can be switched on. Max = 255.  Overridden if EEPROM activated.
#define EXT0_PID_MAX 255
/** \brief Faktor for the advance algorithm. 0 disables the algorithm.  Overridden if EEPROM activated.
K is the factor for the quadratic term, which is normally disabled in newer versions. If you want to use
the quadratic factor make sure ENABLE_QUADRATIC_ADVANCE is defined.
L is the linear factor and seems to be working better then the quadratic dependency.
*/
#define EXT0_ADVANCE_K 0.0f
#define EXT0_ADVANCE_L 0.0f
/* Motor steps to remove backlash for advance alorithm. These are the steps
needed to move the motor cog in reverse direction until it hits the driving
cog. Direct drive extruder need 0. */
#define EXT0_ADVANCE_BACKLASH_STEPS 0
/** \brief Temperature to retract filament when extruder is heating up. Overridden if EEPROM activated.
*/
#define EXT0_WAIT_RETRACT_TEMP 		150
/** \brief Units (mm/inches) to retract filament when extruder is heating up. Overridden if EEPROM activated. Set
to 0 to disable.
*/
#define EXT0_WAIT_RETRACT_UNITS 	0

/** You can run any gcode command on extruder deselect/select. Seperate multiple commands with a new line \n.
That way you can execute some mechanical components needed for extruder selection or retract filament or whatever you need.
The codes are only executed for multiple extruder when changing the extruder. */
#define EXT0_SELECT_COMMANDS "M117 Extruder 1"
#define EXT0_DESELECT_COMMANDS ""
/** The extruder cooler is a fan to cool the extruder when it is heating. If you turn the etxruder on, the fan goes on. */
#define EXT0_EXTRUDER_COOLER_PIN -1
/** PWM speed for the cooler fan. 0=off 255=full speed */
#define EXT0_EXTRUDER_COOLER_SPEED 255


// =========================== Configuration for second extruder ========================
#define EXT1_X_OFFSET 10
#define EXT1_Y_OFFSET 0
// for skeinforge 40 and later, steps to pull the plasic 1 mm inside the extruder, not out.  Overridden if EEPROM activated.
#define EXT1_STEPS_PER_MM 51
// What type of sensor is used?
// 1 is 100k thermistor (Epcos B57560G0107F000 - RepRap-Fab.org and many other)
// 2 is 200k thermistor
// 3 is mendel-parts thermistor (EPCOS G550)
// 4 is 10k thermistor
// 5 is userdefined thermistor table 0
// 6 is userdefined thermistor table 1
// 7 is userdefined thermistor table 2
// 8 is ATC Semitec 104GT-2
// 50 is userdefined thermistor table 0 for PTC thermistors
// 51 is userdefined thermistor table 0 for PTC thermistors
// 52 is userdefined thermistor table 0 for PTC thermistors
// 60 is AD8494, AD8495, AD8496 or AD8497 (5mV/degC and 1/4 the price of AD595 but only MSOT_08 package)
// 97 Generic thermistor table 1
// 98 Generic thermistor table 2
// 99 Generic thermistor table 3
// 100 is AD595
// 101 is MAX6675
#define EXT1_TEMPSENSOR_TYPE 3
// Analog input pin for reading temperatures or pin enabling SS for MAX6675
#define EXT1_TEMPSENSOR_PIN TEMP_2_PIN
// Which pin enables the heater
#define EXT1_HEATER_PIN HEATER_2_PIN
#define EXT1_STEP_PIN E1_STEP_PIN
#define EXT1_DIR_PIN E1_DIR_PIN
// set to 0/1 for normal / inverse direction
#define EXT1_INVERSE true
#define EXT1_ENABLE_PIN E1_ENABLE_PIN
// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define EXT1_ENABLE_ON true
// The following speed settings are for skeinforge 40+ where e is the
// length of filament pulled inside the heater. For repsnap or older
// skeinforge use heigher values.
//  Overridden if EEPROM activated.
#define EXT1_MAX_FEEDRATE 25
// Feedrate from halted extruder in mm/s
//  Overridden if EEPROM activated.
#define EXT1_MAX_START_FEEDRATE 18
// Acceleration in mm/s^2
//  Overridden if EEPROM activated.
#define EXT1_MAX_ACCELERATION 6000
/** Type of heat manager for this extruder.
- 0 = Simply switch on/off if temperature is reached. Works always.
- 1 = PID Temperature control. Is better but needs good PID values. Defaults are a good start for most extruder.
 Overridden if EEPROM activated.
*/
#define EXT1_HEAT_MANAGER 1
/** Wait x seconds, after reaching target temperature. Only used for M109.  Overridden if EEPROM activated. */
#define EXT1_WATCHPERIOD 20

/** \brief The maximum value, I-gain can contribute to the output.

A good value is slightly higher then the output needed for your temperature.
Values for starts:
130 => PLA for temperatures from 170-180 deg C
180 => ABS for temperatures around 240 deg C

The precise values may differ for different nozzle/resistor combination.
 Overridden if EEPROM activated.
*/
#define EXT1_PID_INTEGRAL_DRIVE_MAX 130
/** \brief lower value for integral part

The I state should converge to the exact heater output needed for the target temperature.
To prevent a long deviation from the target zone, this value limits the lower value.
A good start is 30 lower then the optimal value. You need to leave room for cooling.
 Overridden if EEPROM activated.
*/
#define EXT1_PID_INTEGRAL_DRIVE_MIN 50
/** P-gain.  Overridden if EEPROM activated. */
#define EXT1_PID_P   500
/** I-gain.  Overridden if EEPROM activated.
*/
#define EXT1_PID_I   1
/** D-gain.  Overridden if EEPROM activated.*/
#define EXT1_PID_D 3000
// maximum time the heater is can be switched on. Max = 255.  Overridden if EEPROM activated.
#define EXT1_PID_MAX 255
/** \brief Faktor for the advance algorithm. 0 disables the algorithm.  Overridden if EEPROM activated.
K is the factor for the quadratic term, which is normally disabled in newer versions. If you want to use
the quadratic factor make sure ENABLE_QUADRATIC_ADVANCE is defined.
L is the linear factor and seems to be working better then the quadratic dependency.
*/
#define EXT1_ADVANCE_K 0.0f
#define EXT1_ADVANCE_L 0.0f
/* Motor steps to remove backlash for advance alorithm. These are the steps
needed to move the motor cog in reverse direction until it hits the driving
cog. Direct drive extruder need 0. */
#define EXT1_ADVANCE_BACKLASH_STEPS 0

#define EXT1_WAIT_RETRACT_TEMP 	150
#define EXT1_WAIT_RETRACT_UNITS	0
#define EXT1_SELECT_COMMANDS "M117 Extruder 2"
#define EXT1_DESELECT_COMMANDS ""
/** The extruder cooler is a fan to cool the extruder when it is heating. If you turn the etxruder on, the fan goes on. */
#define EXT1_EXTRUDER_COOLER_PIN -1
/** PWM speed for the cooler fan. 0=off 255=full speed */
#define EXT1_EXTRUDER_COOLER_SPEED 255

/** If enabled you can select the distance your filament gets retracted during a
M140 command, after a given temperature is reached. */
#define RETRACT_DURING_HEATUP true

/** PID control only works target temperature +/- PID_CONTROL_RANGE.
If you get much overshoot at the first temperature set, because the heater is going full power too long, you
need to increase this value. For one 6.8 Ohm heater 10 is ok. With two 6.8 Ohm heater use 15.
*/
#define PID_CONTROL_RANGE 20

/** Prevent extrusions longer then x mm for one command. This is especially important if you abort a print. Then the
extrusion position might be at any value like 23344. If you then have an G1 E-2 it will roll back 23 meter! */
#define EXTRUDE_MAXLENGTH 100
/** Skip wait, if the extruder temperature is already within x degrees. Only fixed numbers, 0 = off */
#define SKIP_M109_IF_WITHIN 2

/** \brief Set PID scaling

PID values assume a usable range from 0-255. This can be further limited to EXT0_PID_MAX by to methods.
Set the value to 0: Normal computation, just clip output to EXT0_PID_MAX if computed value is too high.
Set value to 1: Scale PID by EXT0_PID_MAX/256 and then clip to EXT0_PID_MAX.
If your EXT0_PID_MAX is low, you should prefer the second method.
*/
#define SCALE_PID_TO_MAX 0


#define HEATER_PWM_SPEED 1 // How fast ist pwm signal 0 = 15.25Hz, 1 = 30.51Hz, 2 = 61.03Hz, 3 = 122.06Hz

/** Temperature range for target temperature to hold in M109 command. 5 means +/-5 degC

Uncomment define to force the temperature into the range for given watchperiod.
*/
//#define TEMP_HYSTERESIS 5

/** Userdefined thermistor table

There are many different thermistors, which can be combined with different resistors. This result
in unpredictable number of tables. As a resolution, the user can define one table here, that can
be used as type 5 for thermister type in extruder/heated bed definition. Make sure, the number of entries
matches the value in NUM_TEMPS_USERTHERMISTOR0. If you span definition over multiple lines, make sure to end
each line, except the last, with a backslash. The table format is {{adc1,temp1},{adc2,temp2}...} with
increasing adc values. For more informations, read
http://hydraraptor.blogspot.com/2007/10/measuring-temperature-easy-way.html

If you have a sprinter temperature table, you have to multiply the first value with 4 and the second with 8.
This firmware works with increased precision, so the value reads go from 0 to 4095 and the temperature is
temperature*8.

If you have a PTC thermistor instead of a NTC thermistor, keep the adc values increasing and use themistor types 50-52 instead of 5-7!
*/
/** Number of entries in the user thermistor table 0. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR0 28
#define USER_THERMISTORTABLE0  {\
  {1*4,864*8},{21*4,280*8},{25*4,270*8},{29*4,260*8},{33*4,250*8},{39*4,240*8},{46*4,230*8},{54*4,220*8},{64*4,210*8},{75*4,200*8},\
  {90*4,190*8},{107*4,180*8},{128*4,170*8},{154*4,160*8},{184*4,150*8},{221*4,140*8},{265*4,130*8},{316*4,120*8},{375*4,110*8},\
  {441*4,105*8},{513*4,100*8},{588*4,100*8},{734*4,80*8},{856*4,60*8},{938*4,40*8},{986*4,20*8},{1008*4,0*8},{1018*4,-20*8}	}

/** Number of entries in the user thermistor table 1. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR1 0
#define USER_THERMISTORTABLE1  {}
/** Number of entries in the user thermistor table 2. Set to 0 to disable it. */
#define NUM_TEMPS_USERTHERMISTOR2 0
#define USER_THERMISTORTABLE2  {}

/** If defined, creates a thermistor table at startup.

If you don't feel like computing the table on your own, you can use this generic method. It is
a simple approximation which may be not as accurate as a good table computed from the reference
values in the datasheet. You can increase precision if you use a temperature/resistance for
R0/T0, which is near your operating temperature. This will reduce precision for lower temperatures,
which are not realy important. The resistors must fit the following schematic:
@code
VREF ---- R2 ---+--- Termistor ---+-- GND
                |                 |
                +------ R1 -------+
                |                 |
                +---- Capacitor --+
                |
                V measured
@endcode

If you don't have R1, set it to 0.
The capacitor is for reducing noise from long thermistor cable. If you don't have one, it's OK.

If you need the generic table, uncomment the following define.
*/
//#define USE_GENERIC_THERMISTORTABLE_1

/* Some examples for different thermistors:

EPCOS B57560G104+ : R0 = 100000  T0 = 25  Beta = 4036
EPCOS 100K Thermistor (B57560G1104F) :  R0 = 100000  T0 = 25  Beta = 4092
ATC Semitec 104GT-2 : R0 = 100000  T0 = 25  Beta = 4267
Honeywell 100K Thermistor (135-104LAG-J01)  : R0 = 100000  T0 = 25  Beta = 3974

*/

/** Reference Temperature */
#define GENERIC_THERM1_T0 25
/** Resistance at reference temperature */
#define GENERIC_THERM1_R0 200000
/** Beta value of thermistor

You can use the beta from the datasheet or compute it yourself.
See http://reprap.org/wiki/MeasuringThermistorBeta for more details.
*/
#define GENERIC_THERM1_BETA 8304
/** Start temperature for generated thermistor table */
#define GENERIC_THERM1_MIN_TEMP -20
/** End Temperature for generated thermistor table */
#define GENERIC_THERM1_MAX_TEMP 300
#define GENERIC_THERM1_R1 0
#define GENERIC_THERM1_R2 4700

// The same for table 2 and 3 if needed

//#define USE_GENERIC_THERMISTORTABLE_2
#define GENERIC_THERM2_T0 170
#define GENERIC_THERM2_R0 1042.7
#define GENERIC_THERM2_BETA 4036
#define GENERIC_THERM2_MIN_TEMP -20
#define GENERIC_THERM2_MAX_TEMP 300
#define GENERIC_THERM2_R1 0
#define GENERIC_THERM2_R2 4700

//#define USE_GENERIC_THERMISTORTABLE_3
#define GENERIC_THERM3_T0 170
#define GENERIC_THERM3_R0 1042.7
#define GENERIC_THERM3_BETA 4036
#define GENERIC_THERM3_MIN_TEMP -20
#define GENERIC_THERM3_MAX_TEMP 300
#define GENERIC_THERM3_R1 0
#define GENERIC_THERM3_R2 4700

/** Supply voltage to ADC, can be changed by setting ANALOG_REF below to different value. */
#define GENERIC_THERM_VREF 5
/** Number of entries in generated table. One entry takes 4 bytes. Higher number of entries increase computation time too.
Value is used for all generic tables created. */
#define GENERIC_THERM_NUM_ENTRIES 33

// uncomment the following line for MAX6675 support.
//#define SUPPORT_MAX6675
// uncomment the following line for MAX31855 support.
//#define SUPPORT_MAX31855

// ############# Heated bed configuration ########################

/** \brief Set true if you have a heated bed conected to your board, false if not */
#define HAVE_HEATED_BED true

#define HEATED_BED_MAX_TEMP 180
/** Skip M190 wait, if heated bed is already within x degrees. Fixed numbers only, 0 = off. */
#define SKIP_M190_IF_WITHIN 3

// Select type of your heated bed. It's the same as for EXT0_TEMPSENSOR_TYPE
// set to 0 if you don't have a heated bed
#define HEATED_BED_SENSOR_TYPE 3
/** Analog pin of analog sensor to read temperature of heated bed.  */
#define HEATED_BED_SENSOR_PIN TEMP_2_PIN
/** \brief Pin to enable heater for bed. */
#define HEATED_BED_HEATER_PIN HEATER_2_PIN
// How often the temperature of the heated bed is set (msec)
#define HEATED_BED_SET_INTERVAL 5000

/**
Heat manager for heated bed:
0 = Bang Bang, fast update
1 = PID controlled
2 = Bang Bang, limited check every HEATED_BED_SET_INTERVAL. Use this with relay-driven beds to save life time
3 = dead time control
*/
#define HEATED_BED_HEAT_MANAGER 1
/** \brief The maximum value, I-gain can contribute to the output.
The precise values may differ for different nozzle/resistor combination.
 Overridden if EEPROM activated.
*/
#define HEATED_BED_PID_INTEGRAL_DRIVE_MAX 255
/** \brief lower value for integral part

The I state should converge to the exact heater output needed for the target temperature.
To prevent a long deviation from the target zone, this value limits the lower value.
A good start is 30 lower then the optimal value. You need to leave room for cooling.
 Overridden if EEPROM activated.
*/
#define HEATED_BED_PID_INTEGRAL_DRIVE_MIN 80
/** P-gain.  Overridden if EEPROM activated. */
#define HEATED_BED_PID_PGAIN   53.74
/** I-gain  Overridden if EEPROM activated.*/
#define HEATED_BED_PID_IGAIN   7.48
/** Dgain.  Overridden if EEPROM activated.*/
#define HEATED_BED_PID_DGAIN 96.52
// maximum time the heater can be switched on. Max = 255.  Overridden if EEPROM activated.
#define HEATED_BED_PID_MAX 255

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
#define MAXTEMP 275

/** Extreme values to detect defect thermistors. */
#define MIN_DEFECT_TEMPERATURE -10
#define MAX_DEFECT_TEMPERATURE 300


// ##########################################################################################
// ##                            Endstop configuration                                     ##
// ##########################################################################################

/* By default all endstops are pulled up to HIGH. You need a pullup if you
use a mechanical endstop connected with GND. Set value to false for no pullup
on this endstop.
*/
#define ENDSTOP_PULLUP_X_MIN true
#define ENDSTOP_PULLUP_Y_MIN true
#define ENDSTOP_PULLUP_Z_MIN true
#define ENDSTOP_PULLUP_X_MAX true
#define ENDSTOP_PULLUP_Y_MAX true
#define ENDSTOP_PULLUP_Z_MAX true

//set to true to invert the logic of the endstops
#define ENDSTOP_X_MIN_INVERTING true
#define ENDSTOP_Y_MIN_INVERTING true
#define ENDSTOP_Z_MIN_INVERTING true
#define ENDSTOP_X_MAX_INVERTING true
#define ENDSTOP_Y_MAX_INVERTING true
#define ENDSTOP_Z_MAX_INVERTING true

// Set the values true where you have a hardware endstop. The Pin number is taken from pins.h.

#define MIN_HARDWARE_ENDSTOP_X true
#define MIN_HARDWARE_ENDSTOP_Y true
#define MIN_HARDWARE_ENDSTOP_Z true
#define MAX_HARDWARE_ENDSTOP_X false
#define MAX_HARDWARE_ENDSTOP_Y false
#define MAX_HARDWARE_ENDSTOP_Z false

//If your axes are only moving in one direction, make sure the endstops are connected properly.
//If your axes move in one direction ONLY when the endstops are triggered, set ENDSTOPS_INVERTING to true here



//// ADVANCED SETTINGS - to tweak parameters

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 1
#define Y_ENABLE_ON 1
#define Z_ENABLE_ON 1

// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false

// Inverting axis direction
#define INVERT_X_DIR true
#define INVERT_Y_DIR false
#define INVERT_Z_DIR false

//// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

// Delta robot radius endstop
#define max_software_endstop_r true

//If true, axis won't move to coordinates less than zero.
#define min_software_endstop_x false
#define min_software_endstop_y false
#define min_software_endstop_z false

//If true, axis won't move to coordinates greater than the defined lengths below.
#define max_software_endstop_x true
#define max_software_endstop_y true
#define max_software_endstop_z true

// If during homing the endstop is reached, how many mm should the printer move back for the second try
#define ENDSTOP_X_BACK_MOVE 5
#define ENDSTOP_Y_BACK_MOVE 5
#define ENDSTOP_Z_BACK_MOVE 0.5

// For higher precision you can reduce the speed for the second test on the endstop
// during homing operation. The homing speed is divided by the value. 1 = same speed, 2 = half speed
#define ENDSTOP_X_RETEST_REDUCTION_FACTOR 20
#define ENDSTOP_Y_RETEST_REDUCTION_FACTOR 20
#define ENDSTOP_Z_RETEST_REDUCTION_FACTOR 20

// When you have several endstops in one circuit you need to disable it after homing by moving a
// small amount back. This is also the case with H-belt systems.
#define ENDSTOP_X_BACK_ON_HOME 0
#define ENDSTOP_Y_BACK_ON_HOME 0
#define ENDSTOP_Z_BACK_ON_HOME 0

// You can disable endstop checking for print moves. This is needed, if you get sometimes
// false signals from your endstops. If your endstops don't give false signals, you
// can set it on for safety.
#define ALWAYS_CHECK_ENDSTOPS true

// maximum positions in mm - only fixed numbers!
// For delta robot Z_MAX_LENGTH is the maximum travel of the towers and should be set to the distance between the hotend
// and the platform when the printer is at its home position.
// If EEPROM is enabled these values will be overidden with the values in the EEPROM
#define X_MAX_LENGTH	(long)245
#define Y_MAX_LENGTH	(long)245
#define Z_MAX_LENGTH	(long)200

// Coordinates for the minimum axis. Can also be negative if you want to have the bed start at 0 and the printer can go to the left side
// of the bed. Maximum coordinate is given by adding the above X_MAX_LENGTH values.
#define X_MIN_POS 0
#define Y_MIN_POS 0
#define Z_MIN_POS 0

// ##########################################################################################
// ##                           Movement settings                                          ##
// ##########################################################################################

// Microstep setting (Only functional when stepper driver microstep pins are connected to MCU. Currently only works for RAMBO boards
#define MICROSTEP_MODES {8,8,8,8,8} // [1,2,4,8,16]

// Motor Current setting (Only functional when motor driver current ref pins are connected to a digital trimpot on supported boards)
#if MOTHERBOARD==301
#define MOTOR_CURRENT {135,135,135,135,135} // Values 0-255 (RAMBO 135 = ~0.75A, 185 = ~1A)
#elif MOTHERBOARD==12
#define MOTOR_CURRENT {53570,65535,53570,65535,53570} // Values 0-65535 (53570 = ~1.5A)
#elif MOTHERBOARD==13
#define MOTOR_CURRENT {150,150,126,126,126} // Values 0-255 (126 = ~2A), order: driver 1 (x), driver 2 (y), driver 3 (z), driver 4 (extruder 1), driver 5 (reserved)
#endif

/** \brief Number of segments to generate for delta conversions per second of move
*/
#define DELTA_SEGMENTS_PER_SECOND_PRINT 180 // Move accurate setting for print moves
#define DELTA_SEGMENTS_PER_SECOND_MOVE 70 // Less accurate setting for other moves

// Delta settings
#if DRIVE_SYSTEM==3
/** \brief Delta rod length
*/
#define DELTA_DIAGONAL_ROD 345 // mm


/*  =========== Parameter essential for delta calibration ===================

            C, Y-Axis
            |                        |___| CARRIAGE_HORIZONTAL_OFFSET
            |                        |   \
            |_________ X-axis        |    \
           / \                       |     \  DELTA_DIAGONAL_ROD
          /   \                             \
         /     \                             \    Carriage is at printer center!
         A      B                             \_____/
                                              |--| END_EFFECTOR_HORIZONTAL_OFFSET
                                         |----| DELTA_RADIUS
                                     |-----------| PRINTER_RADIUS

    Column angles are measured from X-axis counterclockwise
    "Standard" positions: alpha_A = 210, alpha_B = 330, alpha_C = 90
*/

/** \brief column positions - change only to correct build imperfections! */
#define DELTA_ALPHA_A 210
#define DELTA_ALPHA_B 330
#define DELTA_ALPHA_C 90

/** Correct radius by this value for each column. Perfect builds have 0 everywhere. */
#define DELTA_RADIUS_CORRECTION_A 0
#define DELTA_RADIUS_CORRECTION_B 0
#define DELTA_RADIUS_CORRECTION_C 0

/** Correction of the default diagonal size. Value gets added.*/
#define DELTA_DIAGONAL_CORRECTION_A 0
#define DELTA_DIAGONAL_CORRECTION_B 0
#define DELTA_DIAGONAL_CORRECTION_C 0

/** Max. radius the printer should be able to reach. */
#define DELTA_MAX_RADIUS 200

/** \brief Horizontal offset of the universal joints on the end effector (moving platform).
*/
#define END_EFFECTOR_HORIZONTAL_OFFSET 33

/** \brief Horizontal offset of the universal joints on the vertical carriages.
*/
#define CARRIAGE_HORIZONTAL_OFFSET 18

/** \brief Printer radius in mm, measured from the center of the print area to the vertical smooth rod.
*/
#define PRINTER_RADIUS 175

/**  \brief Horizontal distance bridged by the diagonal push rod when the end effector is in the center. It is pretty close to 50% of the push rod length (250 mm).
*/
#define DELTA_RADIUS (PRINTER_RADIUS-END_EFFECTOR_HORIZONTAL_OFFSET-CARRIAGE_HORIZONTAL_OFFSET)
/* ========== END Delta calibation data ==============*/

/** When true the delta will home to z max when reset/powered over cord. That way you start with well defined coordinates.
If you don't do it, make sure to home first before your first move.
*/
#define DELTA_HOME_ON_POWER false

/** To allow software correction of misaligned endstops, you can set the correction in steps here. If you have EEPROM enabled
you can also change the values online and autoleveling will store the results here. */
#define DELTA_X_ENDSTOP_OFFSET_STEPS 0
#define DELTA_Y_ENDSTOP_OFFSET_STEPS 0
#define DELTA_Z_ENDSTOP_OFFSET_STEPS 0


/** \brief Experimental calibration utility for delta printers
*/
#define SOFTWARE_LEVELING

#endif
#if DRIVE_SYSTEM == 4 // ========== Tuga special settings =============
/* Radius of the long arm in mm. */
#define DELTA_DIAGONAL_ROD 240
#endif

/** \brief Number of delta moves in each line. Moves that exceed this figure will be split into multiple lines.
Increasing this figure can use a lot of memory since 7 bytes * size of line buffer * MAX_SELTA_SEGMENTS_PER_LINE
will be allocated for the delta buffer. With defaults 7 * 16 * 22 = 2464 bytes. This leaves ~1K free RAM on an Arduino
Mega. Used only for nonlinear systems like delta or tuga. */
#define MAX_DELTA_SEGMENTS_PER_LINE 22

/** After x seconds of inactivity, the stepper motors are disabled.
    Set to 0 to leave them enabled.
    This helps cooling the Stepper motors between two print jobs.
    Overridden if EEPROM activated.
*/
#define STEPPER_INACTIVE_TIME 600
/** After x seconds of inactivity, the system will go down as far it can.
    It will at least disable all stepper motors and heaters. If the board has
    a power pin, it will be disabled, too.
    Set value to 0 for disabled.
    Overridden if EEPROM activated.
*/
#define MAX_INACTIVE_TIME 0L
/** Maximum feedrate, the system allows. Higher feedrates are reduced to these values.
    The axis order in all axis related arrays is X, Y, Z
     Overridden if EEPROM activated.
    */
#define MAX_FEEDRATE_X 500
#define MAX_FEEDRATE_Y 500
#define MAX_FEEDRATE_Z  50

/** Home position speed in mm/s. Overridden if EEPROM activated. */
#define HOMING_FEEDRATE_X 165
#define HOMING_FEEDRATE_Y 165
#define HOMING_FEEDRATE_Z 10

/** Set order of axis homing. Use HOME_ORDER_XYZ and replace XYZ with your order. */
#define HOMING_ORDER HOME_ORDER_XYZ

/* If you have a backlash in both z-directions, you can use this. For most printer, the bed will be pushed down by it's
own weight, so this is nearly never needed. */
#define ENABLE_BACKLASH_COMPENSATION false
#define Z_BACKLASH 0
#define X_BACKLASH 0
#define Y_BACKLASH 0

/** Comment this to disable ramp acceleration */
#define RAMP_ACCELERATION 1

/** If your stepper needs a longer high signal then given, you can add a delay here.
The delay is realized as a simple loop wasting time, which is not available for other
computations. So make it as low as possible. For the most common drivers no delay is needed, as the
included delay is already enough.
*/
#define STEPPER_HIGH_DELAY 0

/** The firmware can only handle 16000Hz interrupt frequency cleanly. If you need higher speeds
a faster solution is needed, and this is to double/quadruple the steps in one interrupt call.
This is like reducing your 1/16th microstepping to 1/8 or 1/4. It is much cheaper then 1 or 3
additional stepper interrupts with all it's overhead. As a result you can go as high as
40000Hz.
*/
#define STEP_DOUBLER_FREQUENCY 12000
/** If you need frequencies off more then 30000 you definitely need to enable this. If you have only 1/8 stepping
enabling this may cause to stall your moves when 20000Hz is reached.
*/
#define ALLOW_QUADSTEPPING true
/** If you reach STEP_DOUBLER_FREQUENCY the firmware will do 2 or 4 steps with nearly no delay. That can be too fast
for some printers causing an early stall.

*/
#define DOUBLE_STEP_DELAY 1 // time in microseconds

/** The firmware supports trajectory smoothing. To achieve this, it divides the stepsize by 2, resulting in
the double computation cost. For slow movements this is not an issue, but for really fast moves this is
too much. The value specified here is the number of clock cycles between a step on the driving axis.
If the interval at full speed is below this value, smoothing is disabled for that line.*/
#define MAX_HALFSTEP_INTERVAL 1999

//// Acceleration settings

/** \brief X, Y, Z max acceleration in mm/s^2 for printing moves or retracts. Make sure your printer can go that high!
 Overridden if EEPROM activated.
*/
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_X 1000
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Y 1000
#define MAX_ACCELERATION_UNITS_PER_SQ_SECOND_Z 100

/** \brief X, Y, Z max acceleration in mm/s^2 for travel moves.  Overridden if EEPROM activated.*/
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_X 1000
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Y 1000
#define MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND_Z 100

/** \brief Maximum allowable jerk.

Caution: This is no real jerk in a physical meaning.

The jerk determines your start speed and the maximum speed at the join of two segments.
Its unit is mm/s. If the printer is standing still, the start speed is jerk/2. At the
join of two segments, the speed difference is limited to the jerk value.

Examples:
For all examples jerk is assumed as 40.

Segment 1: vx = 50, vy = 0
Segment 2: vx = 0, vy = 50
v_diff = sqrt((50-0)^2+(0-50)^2) = 70.71
v_diff > jerk => vx_1 = vy_2 = jerk/v_diff*vx_1 = 40/70.71*50 = 28.3 mm/s at the join

Segment 1: vx = 50, vy = 0
Segment 2: vx = 35.36, vy = 35.36
v_diff = sqrt((50-35.36)^2+(0-35.36)^2) = 38.27 < jerk
Corner can be printed with full speed of 50 mm/s

Overridden if EEPROM activated.
*/
#define MAX_JERK  10  // 0.5
#define MAX_ZJERK 0.1  // 0.3

/** \brief Number of moves we can cache in advance.

This number of moves can be cached in advance. If you wan't to cache more, increase this. Especially on
many very short moves the cache may go empty. The minimum value is 5.
*/
#define MOVE_CACHE_SIZE 16

/** \brief Low filled cache size.

If the cache contains less then MOVE_CACHE_LOW segments, the time per segment is limited to LOW_TICKS_PER_MOVE clock cycles.
If a move would be shorter, the feedrate will be reduced. This should prevent buffer underflows. Set this to 0 if you
don't care about empty buffers during print.
*/
#define MOVE_CACHE_LOW 10
/** \brief Cycles per move, if move cache is low.

This value must be high enough, that the buffer has time to fill up. The problem only occurs at the beginning of a print or
if you are printing many very short segments at high speed. Higher delays here allow higher values in PATH_PLANNER_CHECK_SEGMENTS.
*/
#define LOW_TICKS_PER_MOVE 400000

// ##########################################################################################
// ##                           Extruder control                                           ##
// ##########################################################################################

/* \brief Minimum temperature for extruder operation

This is a saftey value. If your extruder temperature is below this temperature, no
extruder steps are executed. This is to prevent your extruder to move unless the fiament
is at least molten. After havong some complains that the extruder does not work, I leave
it 0 as default.
*/

#define MIN_EXTRUDER_TEMP 160

/** \brief Enable advance algorithm.

Without a correct adjusted advance algorithm, you get blobs at points, where acceleration changes. The
effect increases with speed and acceleration difference. Using the advance method decreases this effect.
For more informations, read the wiki.
*/
#define USE_ADVANCE

/** \brief enables quadratic component.

Uncomment to allow a quadratic advance dependency. Linear is the dominant value, so no real need
to activate the quadratic term. Only adds lots of computations and storage usage. */
#define ENABLE_QUADRATIC_ADVANCE


// ##########################################################################################
// ##                           Communication configuration                                ##
// ##########################################################################################

//// AD595 THERMOCOUPLE SUPPORT UNTESTED... USE WITH CAUTION!!!!

/** \brief Communication speed.

- 250000 : Fastes with errorrate of 0% with 16 or 32 MHz - update wiring_serial.c in your board files. See boards/readme.txt
- 115200 : Fast, but may produce communication errors on quite regular basis, Error rate -3,5%
- 76800 : Best setting for Arduino with 16 MHz, Error rate 0,2% page 198 AVR1284 Manual. Result: Faster communication then 115200
- 57600 : Should produce nearly no errors, on my gen 6 it's faster than 115200 because there are no errors slowing down the connection
- 38600

 Overridden if EEPROM activated.
*/
//#define BAUDRATE 76800
//#define BAUDRATE 115200
#define BAUDRATE 250000

/**
Some boards like Gen7 have a power on pin, to enable the atx power supply. If this is defined,
the power will be turned on without the need to call M80 if initially started.
*/
#define ENABLE_POWER_ON_STARTUP

/**
If you use an ATX power supply you need the power pin to work non inverting. For some special
boards you might need to make it inverting.
*/
#define POWER_INVERTING false

/** What shall the printer do, when it receives an M112 emergency stop signal?
 0 = Disable heaters/motors, wait forever until someone presses reset.
 1 = restart by resetting the AVR controller. The USB connection will not reset if managed by a different chip!
*/
#define KILL_METHOD 1

/** \brief Cache size for incoming commands.

There should be no reason to increase this cache. Commands are nearly immediately sent to
execution.
*/
#define GCODE_BUFFER_SIZE 2
/** Appends the linenumber after every ok send, to acknowledge the received command. Uncomment for plain ok ACK if your host has problems with this */
#define ACK_WITH_LINENUMBER
/** Communication errors can swollow part of the ok, which tells the host software to send
the next command. Not receiving it will cause your printer to stop. Sending this string every
second, if our queue is empty should prevent this. Comment it, if you don't wan't this feature. */
#define WAITING_IDENTIFIER "wait"

/** \brief Sets time for echo debug

You can set M111 1 which enables ECHO of commands sent. This define specifies the position,
when it will be executed. In the original FiveD software, echo is done after receiving the
command. With checksum you know, how it looks from the sending string. With this define
uncommented, you will see the last command executed. To be more specific: It is written after
execution. This helps tracking errors, because there may be 8 or more commands in the queue
and it is elsewise difficult to know, what your reprap is currently doing.
*/
#define ECHO_ON_EXECUTE 1

/** \brief EEPROM storage mode

Set the EEPROM_MODE to 0 if you always want to use the settings in this configuration file. If not,
set it to a value not stored in the first EEPROM-byte used. If you later want to overwrite your current
EEPROM settings with configuration defaults, just select an other value. On the first call to epr_init()
it will detect a mismatch of the first byte and copy default values into EEPROM. If the first byte
matches, the stored values are used to overwrite the settings.

IMPORTANT: With mode <>0 some changes in Configuration.h are not set any more, as they are
           taken from the EEPROM.
*/
#define EEPROM_MODE 1


/**************** duplicate motor driver ***************

If you have an unused extruder stepper free, you could use it to drive the second z motor
instead of driving both with a single stepper. The same works for the other axis if needed.
*/

#define FEATURE_TWO_XSTEPPER false
#define X2_STEP_PIN   E1_STEP_PIN
#define X2_DIR_PIN    E1_DIR_PIN
#define X2_ENABLE_PIN E1_ENABLE_PIN

#define FEATURE_TWO_YSTEPPER false
#define Y2_STEP_PIN   E1_STEP_PIN
#define Y2_DIR_PIN    E1_DIR_PIN
#define Y2_ENABLE_PIN E1_ENABLE_PIN

#define FEATURE_TWO_ZSTEPPER false
#define Z2_STEP_PIN   E1_STEP_PIN
#define Z2_DIR_PIN    E1_DIR_PIN
#define Z2_ENABLE_PIN E1_ENABLE_PIN

/* Ditto printing allows 2 extruders to do the same action. This effectively allows
to print an object two times at the speed of one. Works only with dual extruder setup.
*/
#define FEATURE_DITTO_PRINTING false

/* Servos

If you need to control servos, enable this feature. You can control up to 4 servos.
Control the servos with
M340 P<servoId> S<pulseInUS>
servoID = 0..3
Servos are controlled by a pulse width normally between 500 and 2500 with 1500ms in center position. 0 turns servo off.

WARNING: Servos can draw a considerable amount of current. Make sure your system can handle this or you may risk your hardware!
*/

#define FEATURE_SERVO false
// Servo pins on a RAMPS board are 11,6,5,4
#define SERVO0_PIN 11
#define SERVO1_PIN 6
#define SERVO2_PIN 5
#define SERVO3_PIN 4

/* A watchdog resets the printer, if a signal is not send within predifined time limits. That way we can be sure that the board
is always running and is not hung up for some unknown reason. */
#define FEATURE_WATCHDOG true

/* Z-Probing */

#define FEATURE_Z_PROBE false
#define Z_PROBE_PIN 63
#define Z_PROBE_PULLUP true
#define Z_PROBE_ON_HIGH true
#define Z_PROBE_X_OFFSET 0
#define Z_PROBE_Y_OFFSET 0
#define Z_PROBE_BED_DISTANCE 5.0 // Higher than max bed level distance error in mm

// Waits for a signal to start. Valid signals are probe hit and ok button.
// This is needful if you have the probe trigger by hand.
#define Z_PROBE_WAIT_BEFORE_TEST true
/** Speed of z-axis in mm/s when probing */
#define Z_PROBE_SPEED 2
#define Z_PROBE_XY_SPEED 150
#define Z_PROBE_SWITCHING_DISTANCE 1.5 // Distance to safely switch off probe
#define Z_PROBE_REPETITIONS 5 // Repetitions for probing at one point.
/** The height is the difference between activated probe position and nozzle height. */
#define Z_PROBE_HEIGHT 39.91
/** These scripts are run before resp. after the z-probe is done. Add here code to activate/deactivate probe if needed. */
#define Z_PROBE_START_SCRIPT ""
#define Z_PROBE_FINISHED_SCRIPT ""

/* Autoleveling allows it to z-probe 3 points to compute the inclination and compensates the error for the print.
   This feature requires a working z-probe and you should have z-endstop at the top not at the bottom.
   The same 3 points are used for the G29 command.
*/
#define FEATURE_AUTOLEVEL false
#define Z_PROBE_X1 100
#define Z_PROBE_Y1 20
#define Z_PROBE_X2 160
#define Z_PROBE_Y2 170
#define Z_PROBE_X3 20
#define Z_PROBE_Y3 170

/* Babystepping allows to change z height during print without changing official z height */
#define FEATURE_BABYSTEPPING 0

/* If you have a threaded rod, you want a higher multiplicator to see an effect. Limit value to 50 or you get easily overflows.*/
#define BABYSTEP_MULTIPLICATOR 1

/* Define a pin to turn the case light on/off */
#define CASE_LIGHTS_PIN				25	// PINA.3, 75, OUT1
#define CASE_LIGHTS_DEFAULT_ON		 0

/* Define a pin to turn the case fan on/off */
#define	CASE_FAN_PIN				 9	// PINH.6, 18, HZ2
#define	CASE_FAN_ON_TEMPERATURE		50	// �C
#define	CASE_FAN_OFF_DELAY		 60000	// [ms]

/* Enable the following define for applications where the case fan shall always be on */
//#define	CASE_FAN_ALWAYS_ON

/** Set to false to disable SD support: */
#ifndef SDSUPPORT  // Some boards have sd support on board. These define the values already in pins.h
#define SDSUPPORT true
// Uncomment to enable or change card detection pin. With card detection the card is mounted on insertion.
#define SDCARDDETECT -1
// Change to true if you get a inserted message on removal.
#define SDCARDDETECTINVERTED false
#endif
/** Show extended directory including file length. Don't use this with Pronterface! */
#define SD_EXTENDED_DIR true
// If you want support for G2/G3 arc commands set to true, otherwise false.
#define ARC_SUPPORT true

/** You can store the current position with M401 and go back to it with M402.
   This works only if feature is set to true. */
#define FEATURE_MEMORY_POSITION true

/** If a checksum is sent, all future comamnds must also contain a checksum. Increases reliability especially for binary protocol. */
#define FEATURE_CHECKSUM_FORCED false

/** Should support for fan control be compiled in. If you enable this make sure
the FAN pin is not the same as for your second extruder. RAMPS e.g. has FAN_PIN in 9 which
is also used for the heater if you have 2 extruders connected. */
#define FEATURE_FAN_CONTROL true

/** For displays and keys there are too many permutations to handle them all in once.
For the most common available combinations you can set the controller type here, so
you don't need to configure uicong.h at all. Controller settings > 1 disable usage
of uiconfig.h

0 = no display
1 = Manual definition of display and keys parameter in uiconfig.h

The following settings override uiconfig.h!
2 = Smartcontroller from reprapdiscount on a RAMPS or RUMBA board
3 = Adafruit RGB controller
4 = Foltyn 3D Master with display attached
5 = ViKi LCD - Check pin configuration in ui.h for feature controller 5!!! sd card disabled by default!
6 = ReprapWorld Keypad / LCD, predefined pins for Megatronics v2.0 and RAMPS 1.4. Please check if you have used the defined pin layout in ui.h.
7 = RADDS Extension Port
8 = PiBot Display/Controller extension with 20x4 character display
9 = PiBot Display/Controller extension with 16x2 character display
10 = Gadgets3D shield on RAMPS 1.4, see http://reprap.org/wiki/RAMPS_1.3/1.4_GADGETS3D_Shield_with_Panel
11 = RepRapDiscount Full Graphic Smart Controller
12 = FELIXPrinters Controller
13 = SeeMeCNC Display on Rambo (ORION)
14 = OpenHardware.co.za LCD2004 V2014
15 = Sanguinololu + Panelolu2
33 = RF1000
*/
#define FEATURE_CONTROLLER 33

/**
Select the language to use.
0 = English
1 = German
2 = Dutch
3 = Brazilian portuguese
4 = Italian
5 = Spanish
6 = Swedish
*/
#define UI_LANGUAGE 0

/** Animate switches between menus etc. */
#define UI_ANIMATION false

/** How many ms should a single page be shown, until it is switched to the next one.*/
#define UI_PAGES_DURATION 4000

/** Delay of start screen in milliseconds */
#define UI_START_SCREEN_DELAY 1000
/** Uncomment if you don't want automatic page switching. You can still switch the
info pages with next/previous button/click-encoder */
#define UI_DISABLE_AUTO_PAGESWITCH true

/** Time to return to info menu if x millisconds no key was pressed. Set to 0 to disable it. */
#define UI_AUTORETURN_TO_MENU_AFTER 30000

#define FEATURE_UI_KEYS 0

/* Normally cou want a next/previous actions with every click of your encoder.
Unfotunately, the encoder have a different count of phase changes between clicks.
Select an encoder speed from 0 = fastest to 2 = slowest that results in one menu move per click.
*/
#define UI_ENCODER_SPEED 1

/* There are 2 ways to change positions. You can move by increments of 1/0.1 mm resulting in more menu entries
and requiring many turns on your encode. The alternative is to enable speed dependent positioning. It will change
the move distance depending on the speed you turn the encoder. That way you can move very fast and very slow in the
same setting.

*/
#define UI_SPEEDDEPENDENT_POSITIONING true

/** \brief bounce time of keys in milliseconds */
#define UI_KEY_BOUNCETIME 10

/** \brief First time in ms until repeat of action. */
#define UI_KEY_FIRST_REPEAT 500
/** \brief Reduction of repeat time until next execution. */
#define UI_KEY_REDUCE_REPEAT 50
/** \brief Lowest repeat time. */
#define UI_KEY_MIN_REPEAT 50

#define FEATURE_BEEPER	true
#define BEEPER_MODE		1	// 1 = on, 0 = off
/**
Beeper sound definitions for short beeps during key actions
and longer beeps for important actions.
Parameter is delay in microseconds and the second is the number of repetitions.
Values must be in range 1..255
*/
#define BEEPER_SHORT_SEQUENCE				2,2
#define BEEPER_LONG_SEQUENCE				8,8
#define BEEPER_START_PRINTING_SEQUENCE		100,2
#define BEEPER_ABORT_PRINTING_SEQUENCE		250,5
#define BEEPER_STOP_PRINTING_SEQUENCE		100,3
#define BEEPER_PAUSE_SEQUENCE				50,3
#define BEEPER_CONTINUE_SEQUENCE			50,2
#define BEEPER_START_HEAT_BED_SCAN_SEQUENCE	100,2
#define BEEPER_ABORT_HEAT_BED_SCAN_SEQUENCE	250,5
#define BEEPER_STOP_HEAT_BED_SCAN_SEQUENCE	100,3

// ###############################################################################
// ##                         Values for menu settings                          ##
// ###############################################################################

// Values used for preheat
#define UI_SET_PRESET_HEATED_BED_TEMP_PLA 60
#define UI_SET_PRESET_EXTRUDER_TEMP_PLA   180
#define UI_SET_PRESET_HEATED_BED_TEMP_ABS 110
#define UI_SET_PRESET_EXTRUDER_TEMP_ABS   240

// Extreme values
#define UI_SET_MIN_HEATED_BED_TEMP  55
#define UI_SET_MAX_HEATED_BED_TEMP 180
#define UI_SET_MIN_EXTRUDER_TEMP   160
#define UI_SET_MAX_EXTRUDER_TEMP   270
#define UI_SET_EXTRUDER_FEEDRATE 2 // mm/sec
#define UI_SET_EXTRUDER_RETRACT_DISTANCE 3 // mm

#define	SHOW_DEBUGGING_MENU					0													// 1 = show, 0 = hide
#define	SHOW_EXTRUDER_MENU					0													// 1 = show, 0 = hide

/** \brief Allows to use 6 additional hardware buttons
*/
#define FEATURE_EXTENDED_BUTTONS			1													// 1 = on, 0 = off
#define EXTENDED_BUTTONS_COUNTER_NORMAL		4													// 39 ~ run 100 times per second, 4 ~ run 1000 times per second
#define EXTENDED_BUTTONS_COUNTER_FAST		4													// 39 ~ run 100 times per second, 4 ~ run 1000 times per second
#define	EXTENDED_BUTTONS_STEPPER_DELAY		1													// [�s]

#define	EXTENDED_BUTTONS_Z_MIN				-(ZAXIS_STEPS_PER_MM *2)							// [steps]
#define	EXTENDED_BUTTONS_Z_MAX				long(ZAXIS_STEPS_PER_MM * (Z_MAX_LENGTH -2))		// [steps]

/** \brief Enables safety checks for the manual moving into z-direction via the additional hardware buttons
*/
#define FEATURE_ENABLE_MANUAL_Z_SAFETY		1													// 1 = checks enabled, 0 = checks disabled
#define	MANUAL_Z_OVERRIDE_MAX				ZAXIS_STEPS_PER_MM

/** \brief Enables automatic compensation in z direction

Printers which are able to scan the surface of the heating bed and to remember the 
offsets in z-direction can use this feature in order to perform an automatic compensation
in z direction during the printing of the first layers.
*/
#define FEATURE_Z_COMPENSATION				1													// 1 = on, 0 = off
#define Z_COMPENSATION_COUNTER				39													// 39 ~ run 100 times per second, 4 ~ run 1000 times per second

/** \brief Enables debug outputs from the compensation in z direction
*/
#define DEBUG_Z_COMPENSATION				0													// 1 = on, 0 = off

/** \brief Allows to pause the processing of G-Codes
*/
#define FEATURE_PAUSE_PRINTING				1													// 1 = on, 0 = off

#if FEATURE_PAUSE_PRINTING && !FEATURE_Z_COMPENSATION
	#error FEATURE_PAUSE_PRINTING can not be used without FEATURE_Z_COMPENSATION
#endif // FEATURE_PAUSE_PRINTING && !FEATURE_Z_COMPENSATION

/** \brief Allows to cause an emergency stop via a 3-times push of the pause button
*/
#define FEATURE_EMERGENCY_STOP_VIA_PAUSE	0													// 1 = on, 0 = off

/** \brief Specifies until which height the z compensation must complete

This value should be roughly the double amount of mm which is detected as error of the heat bed.
*/
#define Z_COMPENSATION_MAX_MM				2													// [mm]
#define Z_COMPENSATION_MAX_STEPS			long(Z_COMPENSATION_MAX_MM * ZAXIS_STEPS_PER_MM)	// [steps]

/** \brief Specifies from which height on the z compensation shall be performed

Below this value the z compensation will only change the z axis so that a constant distance to the heat bed is hold (this is good for the first layer).
Above this value the z compensation will distribute the roughness of the surface over the layers until Z_COMPENSATION_MAX_STEPS is reached.
*/
#define	Z_COMPENSATION_NO_MM				float(0.2)												// [mm]
#define Z_COMPENSATION_NO_STEPS				long(Z_COMPENSATION_NO_MM * ZAXIS_STEPS_PER_MM)			// [steps]

/** \brief Enables debug outputs from the heat bed scan
*/
#define DEBUG_HEAT_BED_SCAN					0													// 1 = on, 0 = off

/** \brief Defines the I2C address for the strain gauge
*/
#define	I2C_ADDRESS_STRAIN_GAUGE			0x49

/** \brief Defines which strain gauge is used for the heat bed scan
*/
#define	ACTIVE_STRAIN_GAUGE					0x49
#define UI_TEXT_STRAIN_GAUGE				"F:  %s1 digit"

/** \brief Defines the I2C address for the external EEPROM
*/
#define	I2C_ADDRESS_EXTERNAL_EEPROM			0x50

/** \brief Allows to use this firmware together with the Cura PC application

Without this special handling, the firmware may complain about checksum errors from the Cura PC application and
the Cura PC application may fall over the debug outputs of the firmware.
*/
#define	SUPPORT_CURA						0

/** \brief Enables/disables the set to origin feature
*/
#define FEATURE_SET_TO_ORIGIN				0													// 1 = on, 0 = off

/** \brief Configures the delay between the stop of a print and the clean-up like disabling of heaters, disabling of steppers and the outputting of the object
*/
#define	CLEAN_UP_DELAY_AFTER_STOP_PRINT		100													// [ms]

/** \brief Enables/disables the output of the printed object feature
*/
#define FEATURE_OUTPUT_PRINTED_OBJECT		1													// 1 = on, 0 = off

/** \brief The following script allows to configure the exact behavior of the automatic object output
*/
#define	OUTPUT_OBJECT_SCRIPT				"G21\nG91\nG1 E-10\nG1 Z210 F5000\nG1 X0 Y220 F7500\n"

/** \brief Enables/disables the park feature
*/
#define FEATURE_PARK						1													// 1 = on, 0 = off

/** \brief Specifies the park position, in [mm]
*/
#if FEATURE_PARK
#define PARK_POSITION_X						125
#define	PARK_POSITION_Y						140
#define	PARK_POSITION_Z						200
#endif // FEATURE_PARK

/** \brief Enables/disables the reset via the printer menu
*/
#define	FEATURE_RESET_VIA_MENU				1													// 1 = on, 0 = off

/** \brief Enables/diables the emergency z-stop in case of too high pressure
*/
#define FEATURE_EMERGENCY_Z_STOP			0													// 1 = on, 0 = off

/** \brief Specifies the pressure at which the emergency z-stop shall be performed, in [digits]
*/
#define EMERGENCY_Z_STOP_DIGITS_MIN			-5000
#define EMERGENCY_Z_STOP_DIGITS_MAX			5000

/** \brief Specifies the interval at which the pressure check shall be performed, in [ms]
*/
#define	EMERGENCY_Z_STOP_INTERVAL			10

/** \brief Specifies the number of pressure values which shall be averaged. The emergency z-stop can be detected each EMERGENCY_Z_STOP_INTERVAL * EMERGENCY_Z_STOP_CHECKS [ms]
*/
#define	EMERGENCY_Z_STOP_CHECKS				3

/** \brief Enables/diables the emergency pause in case of too high pressure ... the emergency pause can be turned on only in case the general pause functionality is available
*/
#if FEATURE_PAUSE_PRINTING
#define FEATURE_EMERGENCY_PAUSE				1													// 1 = on, 0 = off
#endif // FEATURE_PAUSE_PRINTING

/** \brief Specifies the pressure at which the emergency pause shall be performed, in [digits]
*/
#define EMERGENCY_PAUSE_DIGITS_MIN			200
#define EMERGENCY_PAUSE_DIGITS_MAX			6000

/** \brief Specifies the interval at which the pressure check shall be performed, in [ms]
*/
#define	EMERGENCY_PAUSE_INTERVAL			100

/** \brief Specifies the number of pressure values which shall be averaged. The emergency pause can be detected each EMERGENCY_PAUSE_INTERVAL * EMERGENCY_PAUSE_CHECKS [ms]
*/
#define	EMERGENCY_PAUSE_CHECKS				10

/** \brief Repeating Pause Beep
*/
#define    EMERGENCY_PAUSE_BEEPS				2000

/** \brief Specifies the time interval after the pausing of the print at which the extruder current is reduced
*/
#define EXTRUDER_CURRENT_PAUSE_DELAY	  2000	// [ms] or 0, in order to disable the lowering of the extruder current

/** \brief Specifies the extruder current which shall be use after pausing of the print and before continuing of the print
*/
#if MOTHERBOARD==12
#define	EXTRUDER_CURRENT_PAUSED			 18204	// ~0.5A
#elif MOTHERBOARD==13
#define	EXTRUDER_CURRENT_PAUSED				32	// ~0.5A
#endif

/** \brief Specifies whether the current print shall be aborted in case a temperature sensor is defect
*/
#define FEATURE_ABORT_PRINT_AFTER_TEMPERATURE_ERROR		1										// 1 = abort, 0 = do not abort

/** \brief Configuration of the external watchdog

The TPS3820 of the RF1000 resets about 25 ms after the last time when it was triggered, the value of WATCHDOG_TIMEOUT should be less than half of this time.
*/
#define WATCHDOG_TIMEOUT					10	// [ms]

/** \brief Specifies whether the x, y and z-positions can be changed manually (e.g. via the "Position X/Y/Z" menus or via the hardware buttons) in case the according axis is unknown.

The position of an axis is unknown until the axis has been homed. The position of an axis becomes unknown in case its stepper is disabled.
Enabling of the following feature can be dangerous because it allows to manually drive the printer above its max x/y/z position.
*/
#define	FEATURE_ALLOW_UNKNOWN_POSITIONS		1													// 1 = allow, 0 = do not allow

/** \brief Specifies whether the firmware shall wait a short time after turning on of the stepper motors - this shall avoid that the first steps are sent to the stepper before it is ready
*/
#define	STEPPER_ON_DELAY					25	// [ms]

#if MOTHERBOARD == 13

/** \brief Configuration of the DRV8711
*/
#define DRV8711_NUM_CHANNELS	5

#if RF1000_MICRO_STEPS == 4
	#define DRV8711_REGISTER_00		0x0E11	// 0000 1110 0001 0001: ENBL = 1, RDIR = 0, RSTEP = 0, MODE = 0010, EXSTALL = 0, ISGAIN = 10, DTIME = 11
#elif RF1000_MICRO_STEPS == 8
	#define DRV8711_REGISTER_00		0x0E19	// 0000 1110 0001 1001: ENBL = 1, RDIR = 0, RSTEP = 0, MODE = 0011, EXSTALL = 0, ISGAIN = 10, DTIME = 11
#elif RF1000_MICRO_STEPS == 16
	#define DRV8711_REGISTER_00		0x0E21	// 0000 1110 0010 0001: ENBL = 1, RDIR = 0, RSTEP = 0, MODE = 0100, EXSTALL = 0, ISGAIN = 10, DTIME = 11
#elif RF1000_MICRO_STEPS == 32
	#define DRV8711_REGISTER_00		0x0E29	// 0000 1110 0010 1001: ENBL = 1, RDIR = 0, RSTEP = 0, MODE = 0101, EXSTALL = 0, ISGAIN = 10, DTIME = 11
#elif RF1000_MICRO_STEPS == 64
	#define DRV8711_REGISTER_00		0x0E31	// 0000 1110 0011 0001: ENBL = 1, RDIR = 0, RSTEP = 0, MODE = 0110, EXSTALL = 0, ISGAIN = 10, DTIME = 11
#else
	#error this number of RF1000 micro steps is not supported
#endif // RF1000_MICRO_STEPS

#define DRV8711_REGISTER_02		0x2097	// 0010 0000 1001 0111: TOFF = 10010111, PWMMODE = 0
#define DRV8711_REGISTER_03		0x31D7  // 0011 0001 1101 0111: TBLANK = 11010111, ABT = 1
#define DRV8711_REGISTER_04		0x4430	// 0100 0100 0011 0000: TDECAY = 00110000, DECMOD = 100
#define DRV8711_REGISTER_05		0x583C	// 0101 1000 0011 1100: SDTHR = 00111100, SDCNT = 00, VDIV = 10
#define DRV8711_REGISTER_06		0x60F0	// 0110 0000 1111 0000: OCPTH = 00, OCPDEG = 00, TDRIVEN = 11, TDRIVEP = 11, IDRIVEN = 00, IDRIVEP = 00
#define DRV8711_REGISTER_07		0x7000	// 0111 0000 0000 0000: OTS = 0, AOCP = 0, BOCP = 0, UVLO = 0, APDF = 0, BPDF = 0, STD = 0, STDLAT = 0

#endif // MOTHERBOARD == 13

/** \brief Configuration of the heat bed scan
*/
#define SCAN_X_START_MM					15																// [mm]
#define SCAN_X_START_STEPS				long(XAXIS_STEPS_PER_MM * SCAN_X_START_MM)						// [steps]
#define SCAN_X_END_MM					5																// [mm]
#define SCAN_X_END_STEPS				long(XAXIS_STEPS_PER_MM * SCAN_X_END_MM)						// [steps]
#define SCAN_X_STEP_SIZE_MM				20																// [mm]
#define SCAN_X_STEP_SIZE_STEPS			long(XAXIS_STEPS_PER_MM * SCAN_X_STEP_SIZE_MM)					// [steps]
#define SCAN_X_MAX_POSITION_STEPS		long(X_MAX_LENGTH * XAXIS_STEPS_PER_MM - SCAN_X_END_STEPS)		// [steps]

#define	SCAN_Y_START_MM					30																// [mm]
#define	SCAN_Y_START_STEPS				long(YAXIS_STEPS_PER_MM * SCAN_Y_START_MM)						// [steps]
#define	SCAN_Y_END_MM					5																// [mm]
#define	SCAN_Y_END_STEPS				long(YAXIS_STEPS_PER_MM * SCAN_Y_END_MM)						// [steps]
#define SCAN_Y_STEP_SIZE_MM				20																// [mm]
#define	SCAN_Y_STEP_SIZE_STEPS			long(YAXIS_STEPS_PER_MM * SCAN_Y_STEP_SIZE_MM)					// [steps]
#define SCAN_Y_MAX_POSITION_STEPS		long(Y_MAX_LENGTH * YAXIS_STEPS_PER_MM - SCAN_Y_END_STEPS)		// [steps]

#define SCAN_HEAT_BED_UP_FAST_STEPS		-20																// [steps]
#define SCAN_HEAT_BED_UP_SLOW_STEPS		-4																// [steps]
#define SCAN_HEAT_BED_DOWN_SLOW_STEPS	10																// [steps]
#define SCAN_HEAT_BED_DOWN_FAST_STEPS	long(ZAXIS_STEPS_PER_MM / 4)									// [steps]
#define	SCAN_FAST_STEP_DELAY_MS			5																// [ms]
#define	SCAN_SLOW_STEP_DELAY_MS			100																// [ms]
#define SCAN_IDLE_DELAY_MS				250																// [ms]

#define SCAN_CONTACT_PRESSURE_DELTA		10																// [digits]
#define SCAN_RETRY_PRESSURE_DELTA		5																// [digits]
#define SCAN_IDLE_PRESSURE_DELTA		0																// [digits]
#define SCAN_IDLE_PRESSURE_MIN			-5000															// [digits]
#define SCAN_IDLE_PRESSURE_MAX			5000															// [digits]

#define SCAN_RETRIES					3																// [-]
#define	SCAN_PRESSURE_READS				15																// [-]
#define SCAN_PRESSURE_TOLERANCE			15																// [digits]
#define SCAN_PRESSURE_READ_DELAY_MS		15																// [ms]

#define REMEMBER_PRESSURE				0

/** \brief Configuration of the manual steps
*/
#define DEFAULT_MANUAL_Z_STEPS			(RF1000_MICRO_STEPS *2)
#define MAXIMAL_MANUAL_Z_STEPS			(ZAXIS_STEPS_PER_MM *10)
#define DEFAULT_MANUAL_EXTRUDER_STEPS	(EXT0_STEPS_PER_MM /5)

/** \brief Configuration of the pause steps
*/
#define	DEFAULT_PAUSE_STEPS_X			(XAXIS_STEPS_PER_MM *50)
#define	DEFAULT_PAUSE_STEPS_Y			(YAXIS_STEPS_PER_MM *50)
#define DEFAULT_PAUSE_STEPS_Z			(ZAXIS_STEPS_PER_MM *2)
#define	DEFAULT_PAUSE_STEPS_EXTRUDER	(EXT0_STEPS_PER_MM *10)

#define	PAUSE_X_MIN						(XAXIS_STEPS_PER_MM *5)
#define	PAUSE_Y_MIN						(YAXIS_STEPS_PER_MM *5)
#define	PAUSE_Z_MIN						(ZAXIS_STEPS_PER_MM *2)
#define	PAUSE_X_MAX						((X_MAX_LENGTH -5) * XAXIS_STEPS_PER_MM)
#define	PAUSE_Y_MAX						((Y_MAX_LENGTH -5) * YAXIS_STEPS_PER_MM)
#define	PAUSE_Z_MAX						((Z_MAX_LENGTH -2) * ZAXIS_STEPS_PER_MM)

/** \brief Automatic filament change, unmounting of the filament - ensure that G1 does not attempt to extrude more than EXTRUDE_MAXLENGTH
*/
#define	UNMOUNT_FILAMENT_SCRIPT				"G21\nG90\nG92 E0\nG1 E50 F100\nG92 E0\nG1 E-90 F500\n"

/** \brief Automatic filament change, mounting of the filament - ensure that G1 does not attempt to extrude more than EXTRUDE_MAXLENGTH
*/
#define	MOUNT_FILAMENT_SCRIPT				"G21\nG90\nG92 E0\nG1 E90 F100"

/** \brief Printer name and firmware version
*/
#define UI_PRINTER_NAME "RF1000"
#define UI_PRINTER_COMPANY "Conrad SE"
#define UI_VERSION_STRING "V " REPETIER_VERSION ".48-WKD"

#endif
