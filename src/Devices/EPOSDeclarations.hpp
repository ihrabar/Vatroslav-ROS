/*!	\file	EPOSDeclarations.h
	\brief	Constants declarations for EPOS servo drive programming

	//==========================================================================================//
	//    DEFINITIONS OF THE MOST USED CONSTANTS FOR COMMANDS IN EPOS MOTOR CONTROL PROGRAM     //

	//                            Author: Kristijan Brkic, 2009.                                //
	//==========================================================================================//

*/

namespace Vatroslav
{

	// ====================================
	// Communication ErrorCode Definitions
	// ====================================

	static const int ERR_NOTHING				= 0x00000000;  // No error
	 
	static const int ERR_OBJ_NOT_EXIST			= 0x06020000;  // Object does not exist in the object dictionary

	static const int ERR_SUB_INDX_NOT_EXIST	= 0x06090011;  // Sub-index does not exist

	static const int ERR_CLIENT_SERVER_COMMAND	= 0x05040001;  // Client/server command specifier not valid or unknown

	static const int ERR_TGL_BIT_NOT_ALTERED	= 0x05030000;  // Toggle bit not alternated

	static const int ERR_SDO_TIMED_OUT			= 0x05040000;  // SDO protocol timed out

	static const int ERR_OUT_OF_MEMORY			= 0x05040005;  // Out of memory

	static const int ERR_UNSUPPORTED_ACCESS	= 0x06010000;  // Unsupported access to an object

	static const int ERR_WRITE_ONLY			= 0x06010001;  // Attempt to read a write only object

	static const int ERR_READ_ONLY				= 0x06010002;  // Attempt to write a read only object

	static const int ERR_OBJ_CANNOT_MAP		= 0x06040041;  // Object cannot be mapped to the PDO

	static const int ERR_HUGE_LENGTH			= 0x06040042;  // The number and length of the objects to be mapped would exceed PDO length

	static const int ERR_INCOMPATIBILITY_PARAM	= 0x06040043;  // General parameter incompatibility reason

	static const int ERR_INCOMPATIBILITY_DEV	= 0x06040047;  // General internal incompatibility in the device

	static const int ERR_ACCES_FAILED			= 0x06060000;  // Access failed due to an hardware error

	static const int ERR_DATA_TYPE_MATCH		= 0x06070010;  // Data type does not match, length of service parameter does not match

	static const int ERR_DATA_TYPE_PARAM_HIGH	= 0x06070012;  // Data type does not match, length of service parameter too high

	static const int ERR_DATA_TYPE_PARAM_LOW	= 0x06070013;  // Data type does not match, length of service parameter too low

	static const int ERR_VALUE_RANGE_EXCEEDED	= 0x06090030;  // Value range of parameter exceeded (only for write access)

	static const int ERR_VALUE_TOO_HIGH		= 0x06090031;  // Value of parameter written too high

	static const int ERR_VALUE_TOO_LOW			= 0x06090032;  // Value of parameter written too low

	static const int ERR_MAX_MIN_VAL			= 0x06090036;  // Maximum value is less than minimum value

	static const int ERR_GENERAL				= 0x08000000;  // General error

	static const int ERR_DATA_APP				= 0x08000020;  // Data cannot be transferred or stored to the application

	static const int ERR_DATA_LOCAL_CONTROL	= 0x08000021;  // Data cannot be transferred or stored to the application because of local control

	static const int ERR_DATA_DEV_STATE		= 0x08000022;  // Data cannot be transferred or stored to the application because of the present device state

	// ===================================================
	// MAXON Specific Communication ErrorCode Definitions
	// ===================================================

	static const int ERR_MAX_WRONG_NMT_STATE		= 0x0F00FFC0;  // The device is in wrong NMT state

	static const int ERR_MAX_ILLEGAL_COMMAND		= 0x0F00FFBF;  // The RS-232 command is illegal

	static const int ERR_MAX_INCORRECT_PASSWORD	= 0x0F00FFBE;  // The password is not correct

	static const int ERR_MAX_DEV_NOT_SERVICE_MODE	= 0x0F00FFBC;  // The device is not in service mode

	static const int ERR_MAX_NODE_ID				= 0x0F00FFB9;  // Error Node-ID
	 

	// =============================
	// Important Command Specifiers
	// =============================

	// For reading SDO frames:
	static const int READ_OBJECT_SEND_DATA_1_BYTE		= 0x40;
	static const int READ_OBJECT_SEND_DATA_2_BYTE		= 0x40;
	static const int READ_OBJECT_SEND_DATA_4_BYTE		= 0x40;

	static const int READ_OBJECT_RECEIVE_DATA_1_BYTE	= 0x4F;
	static const int READ_OBJECT_RECEIVE_DATA_2_BYTE	= 0x4B;
	static const int READ_OBJECT_RECEIVE_DATA_4_BYTE	= 0x43;

	// For writing SDO frames:
	static const int WRITE_OBJECT_SEND_DATA_1_BYTE		= 0x2F;
	static const int WRITE_OBJECT_SEND_DATA_2_BYTE		= 0x2B;
	static const int WRITE_OBJECT_SEND_DATA_4_BYTE		= 0x23;

	static const int WRITE_OBJECT_RECEIVE_DATA_1_BYTE	= 0x60;
	static const int WRITE_OBJECT_RECEIVE_DATA_2_BYTE	= 0x60;
	static const int WRITE_OBJECT_RECEIVE_DATA_4_BYTE	= 0x60;

	// When an error appears
	static const int ERROR_COMMAND_SPEC					= 0x80;

	// ===========================================================================
	// Object Dictionary indexes, sub-indexes and sizes for defining Motor type
	// ===========================================================================

	// This constant describes the device type. The lower word of the device type stands
	// for the supported device profile number. The value 0x0192 (402) mean that device 
	// follows the CiA Draft Standard Proposal 402, Device Profile Drives and Motion Control.
	// The higher word holds information about the drive type. The value 0x0002 means that 
	// the drive is a servo drive.
	static const int DEVICE_TYPE_INDX				= 0x1000;
	static const int DEVICE_TYPE_SUB_INDX			= 0x00;
	static const int DEVICE_TYPE_SIZE				= 4;

	// The bit rate of the CAN interface can be changed with the CAN bitrate parameter.
	// (!!Requires EPOS Restart!!)
	static const int CAN_BITRATE_INDX				= 0x2001;
	static const int CAN_BITRATE_SUB_INDX			= 0x00;
	static const int CAN_BITRATE_SIZE				= 2;

	// Set Baudrate for RS232 Interface (!!Requires EPOS Restart!!)
	static const int RS232_BAUDRATE_INDX			= 0x2002;
	static const int RS232_BAUDRATE_SUB_INDX		= 0x00;
	static const int RS232_BAUDRATE_SIZE			= 2;

	// Choose correct type of Maxon motor
	static const int MOTOR_TYPE_INDX				= 0x6402;
	static const int MOTOR_TYPE_SUB_INDX			= 0x00;
	static const int MOTOR_TYPE_SIZE				= 2;

	// This object represents the maximal permissible continuous current of the motor [mA].
	static const int CURRENT_LIMIT_INDX				= 0x6410;
	static const int CURRENT_LIMIT_SUB_INDX			= 0x01;
	static const int CURRENT_LIMIT_SIZE				= 2;

	// Number of magnetic pole pairs (number of poles / 2) from rotor of a brushless DC motor.
	static const int POLE_PAIRS_INDX				= 0x6410;
	static const int POLE_PAIRS_SUB_INDX			= 0x03;
	static const int POLE_PAIRS_SIZE				= 1;

	// To prevent mechanical destroys in current mode it is possible to limit the velocity [rpm].
	static const int MAX_SPEED_INDX					= 0x6410;
	static const int MAX_SPEED_SUB_INDX				= 0x04;
	static const int MAX_SPEED_SIZE					= 2;

	// The thermal time constant of motor winding is used to calculate the time how long
	// the maximal output current is allowed for the connected motor [100 ms].
	static const int THERMAL_TIME_CONST_INDX		= 0x6410;
	static const int THERMAL_TIME_CONST_SUB_INDX	= 0x05;
	static const int THERMAL_TIME_CONST_SIZE		= 2;

	// The encoder pulse number should be set to number of counts per revolution of the
	// connected incremental encoder.
	// Minimal Value: 16 pulse per turn
	// Maximal Value: 7500 pulse per turn
	// 500 pulse/turn is default
	static const int ENCODER_PULSE_NUMBER_INDX		= 0x2210;
	static const int ENCODER_PULSE_NUMBER_SUB_INDX	= 0x01;
	static const int ENCODER_PULSE_NUMBER_SIZE		= 2;

	// The position sensor type can be changed with this parameter.
	static const int POSITION_SENSOR_TYPE_INDX		= 0x2210;
	static const int POSITION_SENSOR_TYPE_SUB_INDX	= 0x02;
	static const int POSITION_SENSOR_TYPE_SIZE		= 2;


	// ===========================================================================
	// Object Dictionary indexes, sub-indexes and sizes for regulator parameters
	// ===========================================================================

	// This parameter represents the proportional gain of the current controller.
	static const int CURRENT_REG_P_GAIN_INDX		= 0x60F6;
	static const int CURRENT_REG_P_GAIN_SUB_INDX	= 0x01;
	static const int CURRENT_REG_P_GAIN_SIZE		= 2;

	// This parameter represents the integral gain of the current controller.
	static const int CURRENT_REG_I_GAIN_INDX		= 0x60F6;
	static const int CURRENT_REG_I_GAIN_SUB_INDX	= 0x02;
	static const int CURRENT_REG_I_GAIN_SIZE		= 2;

	// This parameter represents the proportional gain of the velocity controller.
	static const int VELOCITY_REG_P_GAIN_INDX		= 0x60F9;
	static const int VELOCITY_REG_P_GAIN_SUB_INDX	= 0x01;
	static const int VELOCITY_REG_P_GAIN_SIZE		= 2;

	// This parameter represents the integral gain of the velocity controller.
	static const int VELOCITY_REG_I_GAIN_INDX		= 0x60F9;
	static const int VELOCITY_REG_I_GAIN_SUB_INDX	= 0x02;
	static const int VELOCITY_REG_I_GAIN_SIZE		= 2;

	// This parameter represents the proportional gain of the position controller.
	static const int POSITION_REG_P_GAIN_INDX		= 0x60FB;
	static const int POSITION_REG_P_GAIN_SUB_INDX	= 0x01;
	static const int POSITION_REG_P_GAIN_SIZE		= 2;

	// This parameter represents the integral gain of the position controller.
	static const int POSITION_REG_I_GAIN_INDX		= 0x60FB;
	static const int POSITION_REG_I_GAIN_SUB_INDX	= 0x02;
	static const int POSITION_REG_I_GAIN_SIZE		= 2;

	// This parameter represents the differential gain of the position controller.
	static const int POSITION_REG_D_GAIN_INDX		= 0x60FB;
	static const int POSITION_REG_D_GAIN_SUB_INDX	= 0x03;
	static const int POSITION_REG_D_GAIN_SIZE		= 2;


	// ===========================================================================
	// Object Dictionary indexes, sub-indexes and sizes for most used SDO commands
	// ===========================================================================

	// Set operation modes: Profile Position Mode, Profile Velocity Mode, ... etc.
	static const int OPERATION_MODE_INDX			= 0x6060;
	static const int OPERATION_MODE_SUB_INDX		= 0x00;
	static const int OPERATION_MODE_SIZE			= 1;

	// Maximal allowed difference of position actual value to position demand value. 
	// If difference of position demand value and position actual value is bigger, 
	// a following error occurs.
	static const int MAX_FOLLOWING_ERROR_INDX			= 0x6065;
	static const int MAX_FOLLOWING_ERROR_SUB_INDX		= 0x00;
	static const int MAX_FOLLOWING_ERROR_SIZE			= 4;

	// In Profile Position Mode the position window defines a symmetrical range of 
	// accepted positions relatively to Target position. If the actual value of the
	// position encoder is within the position window, this target position is regarded as reached.
	static const int POSITION_WINDOW_INDX				= 0x6067;
	static const int POSITION_WINDOW_SUB_INDX			= 0x00;
	static const int POSITION_WINDOW_SIZE				= 4;

	// When the Position actual value actual position is within the position window
	// during the defined Position Window time, which is given in multiples of milliseconds,
	// the corresponding bit 10 target reached in the Statusword will be set to one.
	static const int POSITION_WINDOW_TIME_INDX			= 0x6068;
	static const int POSITION_WINDOW_TIME_SUB_INDX		= 0x00;
	static const int POSITION_WINDOW_TIME_SIZE			= 2;

	// Minimal position limit defines the absolute negative position limit for the
	// position demand value [Position units]. If the desired or the actual position
	// is lower then the negative position limit a software position limit Error will be launched.
	static const int MIN_POSITION_LIMIT_INDX			= 0x607D;
	static const int MIN_POSITION_LIMIT_SUB_INDX		= 0x01;
	static const int MIN_POSITION_LIMIT_SIZE			= 4;

	// Maximal position limit defines the absolute positive position limit for the
	// position demand value [Position units]. If the desired or the actual position
	// is higher then the positive position limit a software position limit Error will be launched.
	static const int MAX_POSITION_LIMIT_INDX			= 0x607D;
	static const int MAX_POSITION_LIMIT_SUB_INDX		= 0x02;
	static const int MAX_POSITION_LIMIT_SIZE			= 4;

	// This value is used as velocity limit in a position (or velocity) profile move.
	static const int MAX_PROFILE_VELOCITY_INDX			= 0x607F;
	static const int MAX_PROFILE_VELOCITY_SUB_INDX		= 0x00;
	static const int MAX_PROFILE_VELOCITY_SIZE			= 4;

	// The profile velocity is the velocity normally attained at the end of the
	// acceleration ramp during a profiled move.
	static const int PROFILE_VELOCITY_INDX				= 0x6081;
	static const int PROFILE_VELOCITY_SUB_INDX			= 0x00;
	static const int PROFILE_VELOCITY_SIZE				= 4;

	// This value is used as acceleration in a position (or velocity) profile move.
	static const int PROFILE_ACCELERATION_INDX			= 0x6083;
	static const int PROFILE_ACCELERATION_SUB_INDX		= 0x00;
	static const int PROFILE_ACCELERATION_SIZE			= 4;

	// This value is used as deceleration in a position (or velocity) profile move.
	static const int PROFILE_DECELERATION_INDX			= 0x6084;
	static const int PROFILE_DECELERATION_SUB_INDX		= 0x00;
	static const int PROFILE_DECELERATION_SIZE			= 4;

	// The Quick stop deceleration is used with quickstop command given with the
	// according Controlword. The quick stop deceleration is also used in fault
	// reaction state when the quick-stop profile is allowed.
	static const int QUICK_STOP_DECELERATION_INDX		= 0x6085;
	static const int QUICK_STOP_DECELERATION_SUB_INDX	= 0x00;
	static const int QUICK_STOP_DECELERATION_SIZE		= 4;

	// This object selects the type of the motion profile for trajectories used
	// in Profile Position Mode, Homing Mode or Profile Velocity Mode.
	static const int MOTION_PROFILE_TYPE_INDX			= 0x6086;
	static const int MOTION_PROFILE_TYPE_SUB_INDX		= 0x00;
	static const int MOTION_PROFILE_TYPE_SIZE			= 2;

	// Used for device control commands and control of operating modes.
	static const int CONTROLWORD_INDX					= 0x6040;
	static const int CONTROLWORD_SUB_INDX				= 0x00;
	static const int CONTROLWORD_SIZE					= 2;

	// The statusword indicates the current state of the drive. These bits are not latched.
	static const int STATUSWORD_INDX					= 0x6041;
	static const int STATUSWORD_SUB_INDX				= 0x00;
	static const int STATUSWORD_SIZE					= 2;

	// The target position is the position that the drive should move to in
	// profile position mode using the current settings of motion control parameters
	// such as velocity, acceleration, and deceleration. The target position will be
	// interpreted as absolute or relative depend on controlword
	static const int SET_TARGET_POSITION_INDX			= 0x607A;
	static const int SET_TARGET_POSITION_SUB_INDX		= 0x00;
	static const int SET_TARGET_POSITION_SIZE			= 4;

	// The target velocity is the input for the trajectory generator.
	static const int SET_TARGET_VELOCITY_INDX			= 0x60FF;
	static const int SET_TARGET_VELOCITY_SUB_INDX		= 0x00;
	static const int SET_TARGET_VELOCITY_SIZE			= 4;

	// The actual position is absolute and referenced to system zero position.
	static const int READ_ACTUAL_POSITION_INDX			= 0x6064;
	static const int READ_ACTUAL_POSITION_SUB_INDX		= 0x00;
	static const int READ_ACTUAL_POSITION_SIZE			= 4;

	// The velocity actual value averaged [Velocity units] represents the
	// velocity actual value [Velocity units] filtered by 1st order digital
	// low-pass filter with a cut-off frequency of 5 Hz.
	static const int READ_ACTUAL_VELOCITY_INDX			= 0x2028;
	static const int READ_ACTUAL_VELOCITY_SUB_INDX		= 0x00;
	static const int READ_ACTUAL_VELOCITY_SIZE			= 4;

	// The actual measured current can be read in this object [mA].
	static const int READ_ACTUAL_CURRENT_INDX			= 0x6078;
	static const int READ_ACTUAL_CURRENT_SUB_INDX		= 0x00;
	static const int READ_ACTUAL_CURRENT_SIZE			= 2;

	// All parameters of device where stored in non volatile memory, if the code
	// “save” is written to this object.
	static const int STORE_PARAMETERS_INDX				= 0x1010;
	static const int STORE_PARAMETERS_SUB_INDX			= 0x01;
	static const int STORE_PARAMETERS_SIZE				= 4;

	// All parameters of device where restored with default values, if the code
	// “load” is written to this object.
	static const int RESTORE_PARAMETERS_INDX			= 0x1011;
	static const int RESTORE_PARAMETERS_SUB_INDX		= 0x01;
	static const int RESTORE_PARAMETERS_SIZE			= 4;


	// ===================================================================
	// Object Dictionary indexes, sub-indexes and sizes for analog inputs
	// ===================================================================
	
	// The voltage measured at analog input 1 [mV]; analog input 1
	static const int ANALOG_INPUT1_INDX					= 0x207C;
	static const int ANALOG_INPUT1_SUB_INDX				= 0x01;
	static const int ANALOG_INPUT1_SIZE					= 2;

	// The voltage measured at analog input 2 [mV]; analog input 2
	static const int ANALOG_INPUT2_INDX					= 0x207C;
	static const int ANALOG_INPUT2_SUB_INDX				= 0x02;
	static const int ANALOG_INPUT2_SIZE					= 2;


	// ==================================================================
	// Object Dictionary indexes, sub-indexes and sizes for PDO commands
	// ==================================================================

	// Communication Object Identifier of synchronization object.
	static const int COB_ID_SYNC_INDX					= 0x1005;
	static const int COB_ID_SYNC_SUB_INDX				= 0x00;
	static const int COB_ID_SYNC_SIZE					= 4;

	// This object multiplied by life time factor gives the life time for
	// the Life Guarding Protocol. The lifetime is scaled in milliseconds. It is 0 if not used.
	static const int GUARD_TIME_INDX					= 0x100C;
	static const int GUARD_TIME_SUB_INDX				= 0x00;
	static const int GUARD_TIME_SIZE					= 2;

	// This object multiplied by guard time gives the life time for 
	// the Life Guarding Protocol. It is 0 if not used.
	static const int LIFE_TIME_FACTOR_INDX				= 0x100D;
	static const int LIFE_TIME_FACTOR_SUB_INDX			= 0x00;
	static const int LIFE_TIME_FACTOR_SIZE				= 1;

	// The producer heartbeat time defines the cycle time of the heartbeat. 
	// The producer heartbeat time is 0 if it not used. The time has to be a multiple of 1 ms.
	static const int PROD_HEARTBEAT_TIME_INDX			= 0x1017;
	static const int PROD_HEARTBEAT_TIME_SUB_INDX		= 0x00;
	static const int PROD_HEARTBEAT_TIME_SIZE			= 2;

	// The consumer heartbeat times define the expected cycle time of the heartbeat.
	static const int CONS1_HEARTBEAT_TIME_INDX			= 0x1016;
	static const int CONS1_HEARTBEAT_TIME_SUB_INDX		= 0x01;
	static const int CONS1_HEARTBEAT_TIME_SIZE			= 4;
	static const int CONS2_HEARTBEAT_TIME_INDX			= 0x1016;
	static const int CONS2_HEARTBEAT_TIME_SUB_INDX		= 0x02;
	static const int CONS2_HEARTBEAT_TIME_SIZE			= 4;

	// Communication Object Identifier of emergency object.
	static const int COB_ID_EMCY_INDX					= 0x1014;
	static const int COB_ID_EMCY_SUB_INDX				= 0x00;
	static const int COB_ID_EMCY_SIZE					= 4;

	// PDO Configuration:

	// Communication Object Identifier of transmit process data object 1.
	static const int COB_ID_TRANSMIT_PDO1_INDX				= 0x1800;
	static const int COB_ID_TRANSMIT_PDO1_SUB_INDX			= 0x01;
	static const int COB_ID_TRANSMIT_PDO1_SIZE				= 4;

	// The transmission type describes how PDO 1 communication works.
	static const int PDO1_TRANSMIT_TYPE_INDX				= 0x1800;
	static const int PDO1_TRANSMIT_TYPE_SUB_INDX			= 0x02;
	static const int PDO1_TRANSMIT_TYPE_SIZE				= 1;

	// This time is the minimum interval for event triggered PDO 1 transmission. 
	// The value is defined as multiple of 100 us.
	static const int PDO1_TRANSMIT_INHIBIT_TIME_INDX		= 0x1800;
	static const int PDO1_TRANSMIT_INHIBIT_TIME_SUB_INDX	= 0x03;
	static const int PDO1_TRANSMIT_INHIBIT_TIME_SIZE		= 2;

	// Communication Object Identifier of transmit process data object 2.
	static const int COB_ID_TRANSMIT_PDO2_INDX				= 0x1801;
	static const int COB_ID_TRANSMIT_PDO2_SUB_INDX			= 0x01;
	static const int COB_ID_TRANSMIT_PDO2_SIZE				= 4;

	// The transmission type describes how PDO 2 communication works.
	static const int PDO2_TRANSMIT_TYPE_INDX				= 0x1801;
	static const int PDO2_TRANSMIT_TYPE_SUB_INDX			= 0x02;
	static const int PDO2_TRANSMIT_TYPE_SIZE				= 1;

	// This time is the minimum interval for event triggered PDO 2 transmission. 
	// The value is defined as multiple of 100 us.
	static const int PDO2_TRANSMIT_INHIBIT_TIME_INDX		= 0x1801;
	static const int PDO2_TRANSMIT_INHIBIT_TIME_SUB_INDX	= 0x03;
	static const int PDO2_TRANSMIT_INHIBIT_TIME_SIZE		= 2;

	// Communication Object Identifier of transmit process data object 3.
	static const int COB_ID_TRANSMIT_PDO3_INDX				= 0x1802;
	static const int COB_ID_TRANSMIT_PDO3_SUB_INDX			= 0x01;
	static const int COB_ID_TRANSMIT_PDO3_SIZE				= 4;

	// The transmission type describes how PDO 3 communication works.
	static const int PDO3_TRANSMIT_TYPE_INDX				= 0x1802;
	static const int PDO3_TRANSMIT_TYPE_SUB_INDX			= 0x02;
	static const int PDO3_TRANSMIT_TYPE_SIZE				= 1;

	// This time is the minimum interval for event triggered PDO 3 transmission. 
	// The value is defined as multiple of 100 us.
	static const int PDO3_TRANSMIT_INHIBIT_TIME_INDX		= 0x1802;
	static const int PDO3_TRANSMIT_INHIBIT_TIME_SUB_INDX	= 0x03;
	static const int PDO3_TRANSMIT_INHIBIT_TIME_SIZE		= 2;

	// Communication Object Identifier of transmit process data object 4.
	static const int COB_ID_TRANSMIT_PDO4_INDX				= 0x1803;
	static const int COB_ID_TRANSMIT_PDO4_SUB_INDX			= 0x01;
	static const int COB_ID_TRANSMIT_PDO4_SIZE				= 4;

	// The transmission type describes how PDO 4 communication works.
	static const int PDO4_TRANSMIT_TYPE_INDX				= 0x1803;
	static const int PDO4_TRANSMIT_TYPE_SUB_INDX			= 0x02;
	static const int PDO4_TRANSMIT_TYPE_SIZE				= 1;

	// This time is the minimum interval for event triggered PDO 4 transmission. 
	// The value is defined as multiple of 100 us.
	static const int PDO4_TRANSMIT_INHIBIT_TIME_INDX		= 0x1803;
	static const int PDO4_TRANSMIT_INHIBIT_TIME_SUB_INDX	= 0x03;
	static const int PDO4_TRANSMIT_INHIBIT_TIME_SIZE		= 2;

	// PDO Mapping:

	// Number of mapped Application Objects in transmit PDO 1.
	static const int OBJECT_NO_IN_TRANSMIT_PDO1_INDX		= 0x1A00;
	static const int OBJECT_NO_IN_TRANSMIT_PDO1_SUB_INDX	= 0x00;
	static const int OBJECT_NO_IN_TRANSMIT_PDO1_SIZE		= 1;

	// Map only 1 object of 8
	static const int TRANSMIT_PDO1_1ST_MAP_INDX			= 0x1A00;
	static const int TRANSMIT_PDO1_1ST_MAP_SUB_INDX		= 0x01;
	static const int TRANSMIT_PDO1_1ST_MAP_SIZE			= 4;

	// Number of mapped Application Objects in transmit PDO 2.
	static const int OBJECT_NO_IN_TRANSMIT_PDO2_INDX		= 0x1A01;
	static const int OBJECT_NO_IN_TRANSMIT_PDO2_SUB_INDX	= 0x00;
	static const int OBJECT_NO_IN_TRANSMIT_PDO2_SIZE		= 1;

	// Map only 1 object of 8
	static const int TRANSMIT_PDO2_1ST_MAP_INDX			= 0x1A01;
	static const int TRANSMIT_PDO2_1ST_MAP_SUB_INDX		= 0x01;
	static const int TRANSMIT_PDO2_1ST_MAP_SIZE			= 4;

	// Number of mapped Application Objects in transmit PDO 3.
	static const int OBJECT_NO_IN_TRANSMIT_PDO3_INDX		= 0x1A02;
	static const int OBJECT_NO_IN_TRANSMIT_PDO3_SUB_INDX	= 0x00;
	static const int OBJECT_NO_IN_TRANSMIT_PDO3_SIZE		= 1;

	// Map only 1 object of 8
	static const int TRANSMIT_PDO3_1ST_MAP_INDX			= 0x1A02;
	static const int TRANSMIT_PDO3_1ST_MAP_SUB_INDX		= 0x01;
	static const int TRANSMIT_PDO3_1ST_MAP_SIZE			= 4;

	// Number of mapped Application Objects in transmit PDO 4.
	static const int OBJECT_NO_IN_TRANSMIT_PDO4_INDX		= 0x1A03;
	static const int OBJECT_NO_IN_TRANSMIT_PDO4_SUB_INDX	= 0x00;
	static const int OBJECT_NO_IN_TRANSMIT_PDO4_SIZE		= 1;

	// Map only 1 object of 8
	static const int TRANSMIT_PDO4_1ST_MAP_INDX			= 0x1A03;
	static const int TRANSMIT_PDO4_1ST_MAP_SUB_INDX		= 0x01;
	static const int TRANSMIT_PDO4_1ST_MAP_SIZE			= 4;


	// Flag for target position/velocity reached
	static const int TARGET_REACHED_FLAG	= 0x400;

	// COB-ID for sending data to CAN
	// Make sure that COB-ID SDO client to server SDO parameter (0x1200 - 0x01) is set to 
	// 0x00000600 + Node ID (read - only object)
	static const int COB_ID_SEND			= 0x600;

	// COB-ID for receiving data from CAN
	// Make sure that COB-ID SDO server to client SDO parameter (0x1200 - 0x02) is set to
	// 0x00000580 + Node ID (read - only object)
	static const int COB_ID_RECEIVE			= 0x580;

	// COB-ID for NMT 
	static const int COB_ID_NMT				= 0x0;

	// Enter Operational State
	static const int OPERATIONAL_STATE		= 0x0100;

	// Enter Pre-Operational State
	static const int PRE_OPERATIONAL_STATE	= 0x8000;


	// ===============================================================
	// Adjust some maximal parameter values for motor and servo drive
	// ===============================================================

	// Maximal gain value of parameters in all controllers
	static const int MAX_GAIN_VALUE			= 32767;

	// Maximal number of pole pairs
	static const int MAX_POLE_PAIRS			= 255;

	// Maximal thermal time constant winding for EPOS 70/10
	static const int MAX_THERMAL_TIME		= 5400;

	// Maximal encoder pulse number
	static const int MAX_ENC_PULSE_NO		= 7500;

	// Maximal profile velocity
	static const int MAX_PROFILE_VELOCITY	= 25000;

	// Maximal speed value
	static const int MAX_SPEED_VALUE		= 65535;

	// Delay between commands in [ms]
	static const int DELAY_TIME_MS			= 2;

	// =================================================
	// Object Dictionary enumeration types with values
	// =================================================

	// Bitrate for CAN communication.
	enum CANBitrate
	{
		BITRATE_1000			= 0,  // default in EPOS
		BITRATE_800				= 1,
		BITRATE_500				= 2,
		BITRATE_250				= 3,
		BITRATE_125				= 4,
		BITRATE_50				= 5,
		BITRATE_20				= 6
	};

	// Baudrate for serial communication.
	enum RS232Baudrate
	{
		BRATE_9p6				= 0,
		BRATE_14p4				= 1,
		BRATE_19p2				= 2,
		BRATE_38p4				= 3,   // default
		BRATE_57p6				= 4,
		BRATE_115p2				= 5
	};

	// The type of the motor driven by this controller has to be selected.
	enum MotorType
	{
		DC_PHASE_MODULATED		= 1,
		SINUSOIDAL_PM_BL		= 10,  // default
		TRAPEZOIDAL_PM_BL		= 11
	};

	// Position sensor types.
	enum PosSensorType
	{
		INC_ENCODER_INDEX		= 1,  // default
		INC_ENCODER_NO_INDEX	= 2,
		HALL_SENSORS			= 3
	};

	// The parameter mode of operation switches the actually chosen operation mode.
	enum OperationModes
	{
		HOMING_MODE				= 6,
		PROFILE_VELOCITY_MODE	= 3,
		PROFILE_POSITION_MODE	= 1,  // default
		POSITION_MODE			= -1,
		VELOCITY_MODE			= -2,
		CURRENT_MODE			= -3,
		DIAGNOSTIC_MODE			= -4,
		MASTERENCODER_MODE		= -5,
		STEP_DIRECTION_MODE		= -6
	};

	// Select between trapezoidal and sinusoidal profiles.
	enum MotionProfileTypes
	{
		LINEAR_RAMP			= 0, // default
		SIN2_RAMP			= 1
	};

	// Change operation modes and types of movement
	enum ControlWordOptions
	{
		FAULT_RESET			= 0x0080,
		SHUT_DOWN			= 0x0006,
		DISABLE_VOLTAGE		= 0x0000,
		SWITCH_ON			= 0x000F, // or start move in other sit.
		HALT_ON				= 0x010F,
		QUICK_STOP			= 0x000B,

		ABSOLUTE_MOVE		= 0x001F,
		ABSOLUTE_IMM_MOVE	= 0x003F,
		RELATIVE_MOVE		= 0x005F,
		RELATIVE_IMM_MOVE	= 0x007F
	};

	enum DriveStates
	{
		N_READY_TO_SWITCH_ON= 0x00,
		READY_TO_SWITCH_ON	= 0x21,
		SWITCHED_ON			= 0x23,
		OPERATION_ENABLE	= 0x37,
		FAULT				= 0x08,
		QUICK_STOP_STATE	= 0x17,
		SWITCH_ON_DISABLE	= 0x40,
		UNKNOWN_STATE
	};

	// PDO communication
	enum PDOCommunicationTypes
	{
		SYNCHRON		= 1,
		SYNCHRON_RTR	= 253,
		SYNCHRON_CHANGE	= 255	// the PDO is transmitted if the data’s change its values
	};

	// Type of operation according to EPOS (write to or read from)
	enum OperationType
	{
		READ_FROM_EPOS	= 0,
		WRITE_TO_EPOS	= 1
	};

};

