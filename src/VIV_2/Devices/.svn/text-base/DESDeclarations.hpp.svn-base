/*!	\file	DESDeclarations.h
	\brief	Constants declarations for DES servo drive programming

	//==========================================================================================//
	//     DEFINITIONS OF THE MOST USED CONSTANTS FOR COMMANDS IN DES MOTOR CONTROL PROGRAM     //

	//                            Author: Kristijan Brkic, 2010.                                //
	//==========================================================================================//

*/

namespace Vatroslav
{

	// ===========================
	// DES error code definitions
	// ===========================

	// No error
	static const int DES_ERROR_NONE						= 0;

	// Hall sensor error
	static const int DES_HALL_ERROR						= 1;

	// Index processing error
	static const int DES_INDX_PROCESSING_ERROR			= 2;

	// Wrong setting of encoder resolution
	static const int DES_WRONG_ENC_RESOLUTION			= 3;

	// Hall sensor 3 not found
	static const int DES_HALL_SENSOR_3_ERROR			= 4;

	// Over current error
	static const int DES_OVER_CURRENT					= 5;

	// Over voltage error
	static const int DES_OVER_VOLTAGE					= 6;

	// Over speed error
	static const int DES_SPEED_ERROR					= 7;

	// Supply voltage too low for operation
	static const int DES_LOW_VOLTAGE					= 8;

	// Angle detection error
	static const int DES_ANGLE_DETECTION				= 9;

	// Over temperature error (only on HW 4003h)
	static const int DES_OVER_TEMPERATURE				= 10;

	// Parameter out of range
	static const int DES_PARAM_OUT_RANGE				= 11;

	// ===============================
	// DES CAN error code definitions
	// ===============================

	// Warning Status
	static const int DES_CAN_WARNING					= 12;

	// Error Passive Status
	static const int DES_CAN_PASSIVE_STATUS_ERROR		= 13;

	// Bus Off Status
	static const int DES_CAN_BUS_OFF					= 14;

	// Acknowledge Error
	static const int DES_CAN_ACKNOWLEDGE_ERROR			= 15;

	// Stuff Error
	static const int DES_CAN_STUFF_ERROR				= 16;

	// CRC Error
	static const int DES_CAN_CRC_ERROR					= 17;

	// Stuck at dominant Error
	static const int DES_CAN_STUCK_ERROR				= 18;

	// Bit Error Flag
	static const int DES_CAN_BIT_FLAG_ERROR				= 19;

	// Form Error Flag
	static const int DES_CAN_FORM_ERROR					= 20;

	// PDO accessing frequency is too high
	static const int DES_CAN_PDO_FREQ_TOO_HIGH			= 21;

	// PDO overflow to lose sending message
	static const int DES_CAN_PDO_OVERFLOW				= 22;

	// TxPDO abort acknowledge in sending a message
	static const int DES_CAN_TXPDO_ABORT_ACK			= 23;

	// TxSDO abort acknowledge in sending a message
	static const int DES_CAN_TXSDO_ABORT_ACK			= 24;

	// RxPDO receive message lost
	static const int DES_CAN_RXPDO_MSG_LOST				= 25;

	// RxSDO receive message lost
	static const int DES_CAN_RXSDO_MSG_LOST				= 26;


	// =============================================================
	// Operation codes and data lenghts for DES "Status Functions"
	// =============================================================
	
	// "ReadSysStatus" - Read the system status of the DES. The system 
	// status is a 16-bit value containing different flags.
	static const int READ_SYS_STATUS_OPCODE				= 0x01;
	static const int READ_SYS_STATUS_LEN				= 1;
	static const int READ_SYS_STATUS_RESPONSE			= 1;
	
	// "ReadError" - Read a 16-bit value of system errors.
	static const int READ_ERROR_OPCODE					= 0x02;
	static const int READ_ERROR_LEN						= 1;
	static const int READ_ERROR_RESPONSE				= 1;

	// "ClearError" - Clear the system error.
	static const int CLEAR_ERROR_OPCODE					= 0x03;
	static const int CLEAR_ERROR_LEN					= 1;
	static const int CLEAR_ERROR_RESPONSE				= 0;

	// "Reset" - Reset the system by restarting the software.
	static const int RESET_OPCODE						= 0x04;
	static const int RESET_LEN							= 1;
	static const int RESET_RESPONSE						= 0;

	// "Enable" - Set the system into the enabled or disabled state.
	// The DES has to be configured for a software setting of Enable.
	// If the hardware Enable is activated this command has no effect.
	static const int ENABLE_OPCODE						= 0x05;
	static const int ENABLE_LEN							= 1;
	static const int ENABLE_RESPONSE					= 0;

	// ======================================================================
	// Operation codes and data lenghts for DES "System parameter functions"
	// ======================================================================

	// "ReadTempParam" - Read the requested temporary system parameter from DES-RAM.
	static const int READ_TEMP_PARAM_OPCODE				= 0x14;
	static const int READ_TEMP_PARAM_LEN				= 2;
	static const int READ_TEMP_PARAM_RESPONSE			= 1;

	// "SetTempParam" - Write a new value to a temporary system parameter. Refer to 
	// the section about system parameters to find the desired system parameter numbers.
	static const int SET_TEMP_PARAM_OPCODE				= 0x15;
	static const int SET_TEMP_PARAM_LEN					= 3;  // only for CAN communication
	static const int SET_TEMP_PARAM_RESPONSE			= 0;

	// "ResetTempParam" - Copy the permanent system parameter contained in the EEPROM
	// memory into the temporary parameter set.
	static const int RESET_TEMP_PARAM_OPCODE			= 0x16;
	static const int RESET_TEMP_PARAM_LEN				= 1;
	static const int RESET_TEMP_PARAM_RESPONSE			= 0;

	// "SaveTempParam" - Save the temporary parameters to the EEPROM (non volatile memory).
	static const int SAVE_TEMP_PARAM_OPCODE				= 0x17;
	static const int SAVE_TEMP_PARAM_LEN				= 1;
	static const int SAVE_TEMP_PARAM_RESPONSE			= 0;

	// "SysParSetDefault (Software Version 0x1050 and higher)" - Set all system parameters
	// to default. The system parameter structure is described in the section ‘Data Structures’.
	static const int SYS_PAR_SET_DEFAULT_OPCODE			= 0x1B;
	static const int SYS_PAR_SET_DEFAULT_LEN			= 1;
	static const int SYS_PAR_SET_DEFAULT_RESPONSE		= 0;

	// =============================================================
	// Operation codes and data lenghts for DES "Setting Functions"
	// =============================================================

	// "SetVelocity" - Set a new velocity of the rotor. This function is only
	// available in speed regulation mode.
	static const int SET_VELOCITY_OPCODE				= 0x21;
	static const int SET_VELOCITY_LEN					= 1;
	static const int SET_VELOCITY_RESPONSE				= 0;

	// "SetCurrent" - Set a new current amplitude. This function is only
	// available in current regulation mode.
	static const int SET_CURRENT_OPCODE					= 0x22;
	static const int SET_CURRENT_LEN					= 1;
	static const int SET_CURRENT_RESPONSE				= 0;

	// "StopMotion" - This command changes the stopping state. If the motor is
	// already stopped it will be released. The digital input STOP has the same behaviour.
	// Only for speed regulation mode!
	static const int STOP_MOTION_OPCODE					= 0x23;
	static const int STOP_MOTION_LEN					= 1;
	static const int STOP_MOTION_RESPONSE				= 0;

	// =============================================================
	// Operation codes and data lenghts for DES "Monitor Functions"
	// =============================================================

	// "ReadVelocityIsMust" - Read the effective and requested velocity of the motor.
	static const int READ_VELOCITY_IS_MUST_OPCODE		= 0x28;
	static const int READ_VELOCITY_IS_MUST_LEN			= 1;
	static const int READ_VELOCITY_IS_MUST_RESPONSE		= 1;

	// "ReadCurrentIsMust" - Read the effective and requested current of the motor.
	static const int READ_CURRENT_IS_MUST_OPCODE		= 0x29;
	static const int READ_CURRENT_IS_MUST_LEN			= 1;
	static const int READ_CURRENT_IS_MUST_RESPONSE		= 1;

	// ===========================================================================
	// Operation codes and data lenghts for DES "CAN Bus Configuration Functions"
	// ===========================================================================

	// "SetModuleID" - Set CAN Module-ID (max. 11bit). The module ID is set by DIP
	// switches at the system power up. During operation it's possible to overwrite
	// temporary the moduleID with the command 'SetModuleID'. The moduleID determines 
	// the IDs for SDO communication (TxSDO ID = 1408 + Module-ID; RxSDO ID = 1536 + Module-ID).
	static const int SET_MODULE_ID_OPCODE				= 0x39;
	static const int SET_MODULE_ID_LEN					= 1;
	static const int SET_MODULE_ID_RESPONSE				= 0;

	// "SetTPDOID" - Set CAN Transmit-PDO ID (11 bit). This is the message ID sent by the DES.
	static const int SET_TPDO_ID_OPCODE					= 0x3B;
	static const int SET_TPDO_ID_LEN					= 1;
	static const int SET_TPDO_ID_RESPONSE				= 0;

	// "SetRPDOID" - Set CAN Receive-PDO ID (11 bit). This is the message ID received by the DES.
	static const int SET_RPDO_ID_OPCODE					= 0x3C;
	static const int SET_RPDO_ID_LEN					= 1;
	static const int SET_RPDO_ID_RESPONSE				= 0;

	// "SendCANmsg" - Send CAN standard frame message command.
	static const int SEND_CAN_MSG_OPCODE				= 0x3D;
	static const int SEND_CAN_MSG_LEN					= 5;
	static const int SEND_CAN_MSG_RESPONSE				= 0;

	// "ReadModuleID" - Read DES CAN Module-ID. The module ID is set by DIP switches at the system
	// power up. It's possible that the system parameter 'moduleID' has another value than the DIP
	// switches. It's possible to overwrite this value temporary.
	static const int READ_MODULE_ID_OPCODE				= 0x3E;
	static const int READ_MODULE_ID_LEN					= 1;
	static const int READ_MODULE_ID_RESPONSE			= 1;

	// "SetCANBitrate" - Set CAN transfer rate to calculated values.
	// See the section Bit Timing for more information about this configuration register.
	static const int SET_CAN_BITRATE_OPCODE				= 0x40;
	static const int SET_CAN_BITRATE_LEN				= 1;
	static const int SET_CAN_BITRATE_RESPONSE			= 0;

	// "ReadCANError" - Read a 16 bit value of the CAN error register.
	static const int READ_CAN_ERROR_OPCODE				= 0x43;
	static const int READ_CAN_ERROR_LEN					= 1;
	static const int READ_CAN_ERROR_RESPONSE			= 1;

	// "ConfigPDO" - Switch on and off the PDO communication. The state of the PDO
	// communication can be read with the system parameter 'CAN Config' (SysParam 41, Bit14).
	static const int CONFIG_PDO_OPCODE					= 0x45;
	static const int CONFIG_PDO_LEN						= 1;
	static const int CONFIG_PDO_RESPONSE				= 0;

	// "ResetCANError (Software Version 0x1040 and higher)" - Reset the CAN Error.
	static const int RESET_CAN_ERROR_OPCODE				= 0x06;
	static const int RESET_CAN_ERROR_LEN				= 1;
	static const int RESET_CAN_ERROR_RESPONSE			= 0;

	// "ResetCAN (Software Version 0x1040 and higher)" - Reset the CAN communication.
	static const int RESET_CAN_OPCODE					= 0x07;
	static const int RESET_CAN_LEN						= 1;
	static const int RESET_CAN_RESPONSE					= 0;

	// //////////////////////////////////////////////////////////////////////////////////////////// //
	//                                DES System Parameters constants                               //
	// //////////////////////////////////////////////////////////////////////////////////////////// //

	// System configuration parameter
	static const int PAR_SYS_CONFIG_NB					= 1;

	// ==========================================================
	// Parameter numbers and limit values for regulator parameters
	// ==========================================================

	// Current regulation P-gain parameter constants
	static const int PAR_CURRENT_REG_P_GAIN_NB			= 2;
	static const int PAR_CURRENT_REG_P_GAIN_DWN			= 0;
	static const int PAR_CURRENT_REG_P_GAIN_UP			= 32767;

	// Current regulation I-gain parameter constants
	static const int PAR_CURRENT_REG_I_GAIN_NB			= 3;
	static const int PAR_CURRENT_REG_I_GAIN_DWN			= 0;
	static const int PAR_CURRENT_REG_I_GAIN_UP			= 32767;

	// Speed regulator P-gain parameter constants
	static const int PAR_VELOCITY_REG_P_GAIN_NB			= 5;
	static const int PAR_VELOCITY_REG_P_GAIN_DWN		= 0;
	static const int PAR_VELOCITY_REG_P_GAIN_UP			= 32767;

	// Speed regulator I-gain parameter constants
	static const int PAR_VELOCITY_REG_I_GAIN_NB			= 6;
	static const int PAR_VELOCITY_REG_I_GAIN_DWN		= 0;
	static const int PAR_VELOCITY_REG_I_GAIN_UP			= 32767;

	// =============================================================
	// Parameter numbers and limit values for motor and sensor data
	// =============================================================

	// Motor peak current 
	static const int PAR_PEAK_CURRENT_NB				= 14;
	static const int PAR_PEAK_CURRENT_DWN				= 1;
	static const int PAR_PEAK_CURRENT_UP				= 15000;

	// Motor Max continuous current
	static const int PAR_MAX_CONT_CURR_NB				= 15;
	static const int PAR_MAX_CONT_CURR_DWN				= 1;
	static const int PAR_MAX_CONT_CURR_UP				= 5000;

	// Motor thermal constant
	static const int PAR_THERMAL_CONST_NB				= 16;
	static const int PAR_THERMAL_CONST_DWN				= 0;
	static const int PAR_THERMAL_CONST_UP				= 32767;

	// Motor encoder resolution
	static const int PAR_ENC_RESOLUTION_NB				= 20;
	static const int PAR_ENC_RESOLUTION_DWN				= 0;
	static const int PAR_ENC_RESOLUTION_UP				= 32767;

	// Motor pole pair-numbers
	static const int PAR_POLE_PAIRS_NB					= 21;
	static const int PAR_POLE_PAIRS_DWN					= 1;
	static const int PAR_POLE_PAIRS_UP					= 64;

	// =================================================================
	// Parameter numbers and limit values for motion settings functions
	// =================================================================

	// Motor Max speed
	static const int PAR_MAX_SPEED_NB					= 17;
	static const int PAR_MAX_SPEED_DWN					= 0;
	static const int PAR_MAX_SPEED_UP					= 25000;

	// Motor acceleration
	static const int PAR_ACCELERATION_NB				= 18;
	static const int PAR_ACCELERATION_DWN				= 0;
	static const int PAR_ACCELERATION_UP				= 32767;

	// Motor MaxSpeed in Current Regulation Mode
	static const int PAR_MAX_SPEED_CURR_MODE_NB			= 44;
	static const int PAR_MAX_SPEED_CURR_MODE_DWN		= 0;
	static const int PAR_MAX_SPEED_CURR_MODE_UP			= 32767;

	// ErrorProc
	static const int PAR_ERROR_PROC_NB					= 43;
	static const int PAR_ERROR_PROC_DWN					= 0;  // Disable
	static const int PAR_ERROR_PROC_UP					= 1;  // Stop

	// ================================================
	// Parameter numbers for CAN functions (READ ONLY)
	// ================================================

	// CAN module ID
	static const int PAR_CAN_MODULE_ID_NB				= 30;
	static const int PAR_CAN_MODULE_ID_DWN				= 1;
	static const int PAR_CAN_MODULE_ID_UP				= 127;

	// CAN RxPDO ID
	static const int PAR_CAN_RXPDO_ID_NB				= 32;
	static const int PAR_CAN_RXPDO_ID_DWN				= 385;
	static const int PAR_CAN_RXPDO_ID_UP				= 1407;

	// CAN TxPDO ID
	static const int PAR_CAN_TXPDO_ID_NB				= 33;
	static const int PAR_CAN_TXPDO_ID_DWN				= 385;
	static const int PAR_CAN_TXPDO_ID_UP				= 1407;

	// CAN RxSDO ID
	static const int PAR_CAN_RXSDO_ID_NB				= 37;
	static const int PAR_CAN_RXSDO_ID_DWN				= 1537;
	static const int PAR_CAN_RXSDO_ID_UP				= 1663;

	// CAN TxSDO ID
	static const int PAR_CAN_TXSDO_ID_NB				= 38;
	static const int PAR_CAN_TXSDO_ID_DWN				= 1409;
	static const int PAR_CAN_TXSDO_ID_UP				= 1535;

	// //////////////////////////////////////////////////////////////////////////////////////////// //
	//                                 DES Status Variables constants                               //
	// //////////////////////////////////////////////////////////////////////////////////////////// //


	// System operating status
	static const int PAR_SYS_OPER_STATUS_NB				= 128;

	// Actual mean current value in d-axis (app. 0)
	static const int PAR_MEAN_CURR_D_AXIS_NB			= 129;

	// Actual mean current value in q-axis (=> Torque)
	static const int PAR_MEAN_CURR_Q_AXIS_NB			= 130;

	// Current setting value
	static const int PAR_CURR_SETTING_VALUE_NB			= 131;

	// Speed setting value
	static const int PAR_SPEED_SETTING_VALUE_NB			= 133;

	// Actual mean speed value
	static const int PAR_MEAN_SPEED_VALUE_NB			= 134;

	// Absolute rotor position
	static const int PAR_ABSOLUTE_ROTOR_POS_NB			= 136;

	// Standard Error
	static const int PAR_STANDARD_ERROR_NB				= 137;

	// CAN Error
	static const int PAR_CAN_ERROR_NB					= 138;

	// Actual current value in q-axis (=> Torque) (not averaged)
	static const int PAR_ACT_CURR_VALUE_NB				= 139;

	// Actual speed value (not averaged)
	static const int PAR_ACT_SPEED_VALUE_NB				= 140;

	// Encoder Counter
	static const int PAR_ENC_COUNTER_NB					= 143;

	// ////////////////////////////////////////////////// //
	//              Other DES constants                   //
	// ////////////////////////////////////////////////// //

	// Dummy constant
	static const int DUMMY								= 0x0000;

	// Predefined CAN ID for calculation of RxSDO ID
	static const int CAN_RX_ID							= 1536;

	// Predefined CAN ID for calculation of TxSDO ID
	static const int CAN_TX_ID							= 1408;

	// Delay between commands in [ms]
	static const int DELAY_TIME_MS_DES					= 2;

	// Delay time when all parameters are set
	// (by using "Connect" method) in [ms]
	static const int DELAY_TIME_CONNECT_DES				= 700;

	// Delay in code for other, smaller changes in [ms]
	static const int DELAY_TIME_OTHER_DES				= 2;

	// Delay time; time needed for receiving command response
	// from DES, in [ms]
	static const int DELAY_TIME_RESPONSE				= 2;

	// Tolerance for velocity in calculation if target is reached
	static const int REACHED_TARGET_VEL_TOL				= 5;

	// Tolerance for current in calculation if target is reached
	static const int REACHED_TARGET_CURR_TOL			= 6;


	// //////////////////////////////////////////////////////////////////// //
	//                  DES enumeration types with values                   //
	// //////////////////////////////////////////////////////////////////// //

	// DES system states
	enum DESSystemStates
	{
		DISABLE_STATE			= 0,
		ENABLE_STATE			= 1
	};

	// The parameter mode of operation switches the actually chosen operation mode.
	enum DESOperationModes
	{
		DES_VELOCITY_MODE			= 0,
		DES_CURRENT_MODE			= 1
	};

	// Monitor speed or torque signal
	enum MonitorSignalType
	{
		SPEED_SIGNAL			= 0,
		TORQUE_SIGNAL			= 1
	};

	// Data format of the variable
	enum DataFormat
	{
		WORD_DATA				= 0,
		LWORD_DATA				= 1
	};

	// Monitor value types
	enum MonitorValueTypes
	{
		MEAN_VALUES				= 0x0000,
		REAL_TIME_VALUES		= 0x0001
	};

	// Bitrate for CAN communication.
	enum DESCANBitrate
	{
		DES_BITRATE_1000			= 0,  
		DES_BITRATE_800				= 1,
		DES_BITRATE_500				= 2,
		DES_BITRATE_250				= 3,
		DES_BITRATE_125				= 4,
		DES_BITRATE_50				= 5,
		DES_BITRATE_20				= 6,
		DES_BITRATE_10				= 7
	};

	// PDO configuration settings
	enum PDOConfiguration
	{
		PDO_SWITCH_OFF				= 0,
		PDO_SWITCH_ON				= 1
	};

	// ErrorProc settings
	enum ErrorProc
	{
		DISABLE_EP				= 0,
		STOP_EP					= 1
	};

}