/*!	\file	MotorDES.hpp
	\brief	MotorDES class definition.
	//==========================================================================================//
	//                           DEFINITIONS OF THE MOTOR CLASSES                               //

	//                            Author: Kristijan Brkic, 2010.                                //
	//==========================================================================================//

	(Note: If any class method is declared with include-word "Update" or with uppercase characters, 
	       its using causes communication with servo-drive!)

     !!!WARNING!!!: USE THIS CLASS ONLY ON CAN COMMUNICATION INTERFACE WITH SERVO-DRIVE. IT WILL 
	                NOT WORK ON SERIAL COMMUNICATION INTERFACE.
 */

#ifndef VATROSLAV_MOTOR_DES_HPP
#define VATROSLAV_MOTOR_DES_HPP

#include <memory>
#include <cassert>
#include <string>

#include "Communication/Communication.hpp"
#include "DESDeclarations.hpp"
#include "Motor.hpp"

namespace Vatroslav
{

//=============================================================================

//! Regulator parameters class (RegParDES)
/*!
	This class defines gain parameters of current and velocity PI controllers.

	@param currRegGainP - proportional gain of current controller
	@param currRegGainI - integral gain of current controller
	
	@param velRegGainP  - proportional gain of velocity controller
	@param velRegGainI  - integral gain of velocity controller
*/
class RegParDES
{
	public:
		// Data type appropriate for regulator params.
		typedef unsigned short ushort;
		//! Constructor for regulator params (also default constructor)
		/*!
			Defines all default gain parameters for normal robot operation
		*/
		RegParDES( ushort currRegGainP = 3057, ushort currRegGainI = 994, 
				ushort velRegGainP = 575, ushort velRegGainI = 92 );

		//! Get current controller gain P
		ushort GetCurrRegGainP(void) const;

		//! Set current controller gain P
		void SetCurrRegGainP(ushort currRegGainP);

		//! Get current controller gain I
		ushort GetCurrRegGainI(void) const;

		//! Set current controller gain I
		void SetCurrRegGainI(ushort currRegGainI);

		/////////////////////////////////////////////////////////////////////

		//! Get velocity controller gain P
		ushort GetVelRegGainP(void) const;

		//! Set velocity controller gain P
		void SetVelRegGainP(ushort velRegGainP);

		//! Get velocity controller gain I
		ushort GetVelRegGainI(void) const;

		//! Set velocity controller gain I
		void SetVelRegGainI(ushort velRegGainI);

	private:
		// Current PI controller parameters
		ushort currRegGainP_;		// Proportional gain
		ushort currRegGainI_;		// Integral gain

		// Velocity PI controller parameters
		ushort velRegGainP_;		// Proportional gain
		ushort velRegGainI_;		// Integral gain

};

//=============================================================================

//! Parameters for motor motion (OperationParDES)
/*!
    This class defines parameters for motor motion:

	@param motorOperationMode	-	defines operation mode (current or velocity mode)
	@param currSpeedSettingByAnalogueInput	-	defines whether current or speed setting value is set by software or extern user input
	@param accelerationEnabled	-	defines whether acceleration is enabled or disabled
	@param stopMotorBySoftware	-	defines whether motor can be stopped with software or extern user input
	@param setMaxSpeedBySoftware	-	defines whether max speed value is set by software or extern user input
	@param setOffsetBySoftware	-	defines whether offset value is set by software or extern user input
	@param setMaxCurrBySoftware		-	defines whether max current is set by software or extern user input
	@param setRegGainsBySoftware	-	defines whether regulator gains are set by software or extern user input
	@param enableSystemBySoftware	-	defines whether system is enabled/disabled by software or extern user input
	@param regulationModeByBits		-	defines whether regulation mode is set by software or extern user input
	@param monitorSignal	-	defines the type of the monitoring signal (speed or torque)
	@param monitoringSignalByBits	-	defines whether monitoring signal is set by software or extern user input

*/

class OperationParDES
{
	public:
		typedef unsigned long ulong;
		typedef unsigned short ushort;

		//! Constructor for motor motion parameters
		/*!

		*/
		OperationParDES(DESOperationModes motorOperationMode, 
				bool currSpeedSettingByAnalogueInput = false,
				bool accelerationEnabled = true,
				bool stopMotorBySoftware = true,
				bool setMaxSpeedBySoftware = true,
				bool setOffsetBySoftware = true,
				bool setMaxCurrBySoftware = true,
				bool setRegGainsBySoftware = true,
				bool enableSystemBySoftware = true,
				bool regulationModeByBits = true,
				MonitorSignalType monitorSignal = SPEED_SIGNAL,
				bool monitoringSignalByBits = true);
		
		//! Get DES operation mode (current or velocity mode)
		DESOperationModes GetOperationMode(void) const;

		//! Set DES operation mode (current or velocity mode)
		void SetOperationMode(DESOperationModes motorOperationMode);

		//! Get the data if the current/speed value is set by analogue input
		bool GetCurrSpeedSettingByAnalogueInput(void) const;

		//! Set the data if the current/speed value is set by analogue input
		void SetCurrSpeedSettingByAnalogueInput(bool currSpeedSettingByAnalogueInput);

		//! Get the data if acceleration is enabled
		bool GetAccelerationEnabled(void) const;

		//! Set the data if acceleration is enabled
		void SetAccelerationEnabled(bool accelerationEnabled);

		//! Get the data if motor can be stopped by software
		bool GetStopMotorBySoftware(void) const;

		//! Set the data if motor can be stopped by software
		void SetStopMotorBySoftware(bool stopMotorBySoftware);

		//! Get the data if the max speed can be defined with software
		bool GetMaxSpeedBySoftware(void) const;

		//! Set the data if the max speed can be defined with software
		void SetMaxSpeedBySoftware(bool setMaxSpeedBySoftware);

		//! Get the data if offset can be set by software
		bool GetOffsetBySoftware(void) const;

		//! Set the data if offset can be set by software
		void SetOffsetBySoftware(bool setOffsetBySoftware);

		//! Get the data if max current can be set by software
		bool GetMaxCurrentBySoftware(void) const;

		//! Set the data if max current can be set by software
		void SetMaxCurrentBySoftware(bool setMaxCurrBySoftware);

		//! Get the data if regulator gains can be set by software
		bool GetRegGainsBySoftware(void) const;

		//! Set the data if regulator gains can be set by software
		void SetRegGainsBySoftware(bool setRegGainsBySoftware);

		//! Get the data if system can be enabled by software
		bool GetEnableSystemBySoftware(void) const;

		//! Set the data if system can be enabled by software
		void SetEnableSystemBySoftware(bool enableSystemBySoftware);

		//! Get the data if regulation mode can be set by bits, i.e. by software
		bool GetRegulationModeBySoftware(void) const;

		//! Set the data if regulation mode can be set by bits, i.e. by software
		void SetRegulationModeBySoftware(bool regulationModeByBits);

		//! Get the type of the monitoring signal
		MonitorSignalType GetMonitoringSignalType(void) const;

		//! Set the type of the monitoring signal
		void SetMonitoringSignalType(MonitorSignalType monitorSignal);

		//! Get the data if monitoring signal is defined by software
		bool GetMonitoringSignalBySoftware(void) const;

		//! Set the data if monitoring signal is defined by software
		void SetMonitoringSignalBySoftware(bool monitoringSignalByBits);

	private:
		//! Motor operation mode
		DESOperationModes motorOperationMode_;

		//! Current/speed setting by analogue input
		bool currSpeedSettingByAnalogueInput_;

		//! Acceleration is enabled
		bool accelerationEnabled_;

		//! Stop motor by software
		bool stopMotorBySoftware_;

		//! Motor Max speed by software
		bool setMaxSpeedBySoftware_;

		//! Offset by sofrware
		bool setOffsetBySoftware_;

		//! Max current by software
		bool setMaxCurrBySoftware_;

		//! Set regulator gains by software
		bool setRegGainsBySoftware_;

		//! Enable system by Software
		bool enableSystemBySoftware_;

		//! Regulation mode by software/bits
		bool regulationModeByBits_;

		//! Type of the monitoring signal
		MonitorSignalType monitorSignal_;

		//! Monitoring signal defined with bits/software
		bool monitoringSignalByBits_;

};

//=============================================================================

//! Motor parameters class (MotorParDES)
/*!
	@param ID				-	motor identification number
	@param regPar			-	motor regulator parameters
	@param operationPar		-	defines how to operate with DES

	@param noPolePairs		-	number of motor pole pairs
	@param maxVelocity		-	maximal possible motor velocity in [rpm]
	@param acceleration		-	motor acceleration value in [rpm/128ms]
	@param peakCurrent		-	value of the motor peak current in [mA]
	@param maxContCurrent	-	value of the maximal continous current in [mA]
	@param encResolution	-	motor encoder resolution in [pulse per turn]
*/

class MotorParDES
{
	public:

		// Less typing :-)
		typedef unsigned short ushort;
		typedef unsigned long ulong;

		//! Constructor for motor data.
		/*!
			Constructs motor parameters for Vatroslav motors.
		*/
		MotorParDES( ushort ID, 
					 RegParDES regPar,
					 OperationParDES operationPar,
					 ushort noPolePairs = 1,
					 ushort maxVelocity = 12000,
					 ushort acceleration = 1500,
					 ushort peakCurrent = 5000,
					 ushort maxContCurrent = 4720,
					 ushort encResolution = 500,
					 ushort maxVelocityCurrMode = 1000);

		//! Get motor ID
		ushort GetMotorID(void) const;

		//! Get controller parameters
		RegParDES& RegParams(void);

		//! Get operation parameters
		OperationParDES& OperationParams(void);

		//! Get number of pole pairs
		ushort GetNumberPolePairs(void) const;

		//! Set number of pole pairs
		void SetNumberPolePairs(ushort noPolePairs);

		//! Get maximal permissible speed
		ushort GetMaximalSpeed(void) const;

		//! Set maximal permissible speed
		void SetMaximalSpeed(ushort maxPermissSpeed);

		//! Get motor acceleration value
		ushort GetMotorAcceleration(void) const;

		//! Set motor acceleration value
		void SetMotorAcceleration(ushort acceleration);

		//! Get motor peak current
		ushort GetMotorPeakCurrent(void) const;

		//! Set motor peak current
		void SetMotorPeakCurrent(ushort peakCurrent);

		//! Get maximal continous current
		ulong GetMaximalCurrent(void) const;

		//! Set maximal continous current
		void SetMaximalCurrent(ulong nominalCurrent);

		//! Get encoder resolution 
		ushort GetEncoderResolution(void) const;

		//! Set encoder resolution
		void SetEncoderResolution(ushort encResolution);

		//! Get maximal velocity in current regulation mode
		ushort GetMaxVelocityCurrentMode(void) const;

		//! Set maximal velocity in current regulation mode
		void SetMaxVelocityCurrentMode(ushort maxVelocityCurrMode);

	private:

		// Regulator parameters
		RegParDES			regPar_;

		// Motion parameters
		OperationParDES		operationPar_;

		// Motor identification number on network
		ushort			ID_;

		// Some motor technical data
		ushort			noPolePairs_;
		ushort			maxPermissSpeed_;
		ushort			acceleration_;
		ushort			peakCurrent_;
		ushort			nominalCurrent_;
		ushort			encResolution_;
		ushort			maxVelocityCurrMode_;

};

//=============================================================================
//! The Motor class.
/*!

 */
class MotorDES : public Motor
{
	public:

		typedef unsigned short ushort;
		typedef unsigned char uchar;
		typedef unsigned long ulong;

	
		//! Constructor.
		/*!

		 */
		MotorDES( MotorParDES motpar, CommPtr pComm );

		//! Destructor.
		/*!

		 */
		~MotorDES();
		
		//! Get motor parameters
		/*!

		 */
		MotorParDES& Params( void ) { return motPar_; }

		///// Read actual current, velocity or position /////
		//! Read actual current
		virtual long Current(void) const;

		//! Read actual position
		virtual long Position(void) const;

		//! Read actual velocity
		virtual long Velocity(void) const;

		///// Set reference current or velocity /////
		//! Set motor current reference
		void CurrentRef(long Current);

		//! Set motor velocity reference
		virtual void VelocityRef(long Velocity);

		virtual void PositionRef( long pos_ref ) { positionRef_ = (short) pos_ref; }

		///// Get reference current or velocity /////
		//! Get motor current reference
		long CurrentRef(void) const;

		//! Get motor velocity reference
		virtual long VelocityRef(void) const;

		virtual long PositionRef( void ) const { return positionRef_; }

		//! Get error code (error if nonzero)
		long GetErrorCode(void) const;

		//! Get error description
		std::string GetErrorDescription(void);
	

		////////////////////////////////// UPDATE PART ///////////////////////////////////
	
		///// Update regulator parameters by reading from and writing values to DES /////
		//! Read values
		bool UpdateRegParRead( void );

		//! Write values
		bool UpdateRegParWrite( void );

		///// Update motor operation parameters by reading from and writing values to DES /////
		//! Read values
		bool UpdateOperationParRead( void );

		//! Write values
		bool UpdateOperationParWrite( void );

		///// Update motor and sensor parameters by reading from and writing values to DES /////
		//! Read values
		bool UpdateMotorParRead( void );

		//! Write values
		bool UpdateMotorParWrite( void );


		//! Update actual motor current, speed and velocity, read all data
		bool UpdateReadMeasuredData( void );

		//! Set references for motor speed or current
		bool UpdateSetReferences( void );


		//! Connect to Device.
		/*!
			"Connect" method updates all parameters to DES device that
			was set in construction of "RegParDES", "OperationParDES" and "MotorParDES"
			objects. 
		 */
		virtual bool Connect( void );

		//! Disconnect from Device.
		/*!
			
		 */
		virtual bool Disconnect( void );

		//! Read motor sensors.
		/*!
			Used to read actual current, velocity and position
			from DES device.
		 */
		virtual bool UpdateRead( void );

		//! Write setpoints to motor.
		/*! Prepares DES states for motor running and
			updates velocity and current setpoints before motor starts. Depending on
			Current Mode/Velocity Mode, approriate setpoints will be used.

		 */
		virtual bool UpdateWrite( void );

		//! Stop motor immediately with high deceleration
		/*!
			This method stops motor movement very fast. This command can be used only when
			DES is in Velocity Operation Mode.
		*/
		virtual bool QUICKSTOP(void);

		//! Halts motor movement
		/*!
			This method halts slowly motor movement. This command can be used only when
			DES is in Velocity Operation Mode.
		*/
		virtual bool HALT(void);

		//! Determine if predefined target is reached
		/*!
			Determines whether current or speed reference value is reavhed.
		*/
		virtual bool IS_TARGET_REACHED(void);
		
		//! Disable voltage on DES controller
		/*!
			Disable voltage on DES and disable also any possible movements; leave device
			in a safe state whatever the power is turned on.
		*/
		bool DISABLE_DES(void);

		//! Get  motor status.
		/*! 
			Return current motor status, ENABLED, DISABLED or ERROR.
		 */
		virtual MotorStatus Status( void );


	private:

		 //! Actual current variable
		 short actualCurrent_;

		 //! Actual position variable
		 short actualPosition_;

		 //! Actual velocity variable
		 short actualVelocity_;

		 //! Current reference variable
		 short currentRef_;
		 
		 //! Position reference variable
		 short positionRef_;
		 
		 //! Velocity reference variable
		 short velocityRef_;

		 //! Motor parameters
		 MotorParDES motPar_;
		 
		 //! Determine if motor is connected or not
		 bool motorConnected_;

		 //! Communication parameters
		 CommPtr pComm_; 

		 //! SDO Frame ID (Write: 1536 + Module-ID; Read: 1408 + Module-ID)
		 unsigned _SDO_ID_;

		 //! Data variable for sending SDO Frame
		 char _dataSendSDO_[8];

		 //! Data variable for receiving SDO Frame
		 char _dataReceiveSDO_[8];
	
		 //! opCode of the written or readed SDO frame
		 uchar opCode_;

		 //! Data length of the actual function data in words (16-bits)
		 ushort dataLength_;

		 //! Response of the actual function from DES (expect or not)
		 ushort functionResponse_;
		
		 //! Error code from DES
		 unsigned _ErrorCode_;

		 //! Last status of the motor
		 MotorStatus _status_;

		 //! Delay; halt operation for given number of mimiseconds, [ms]
		 /*
			 This function is used in some code parts to halt/stop temporarily 
			 program execution to enable DES to execute his inward commands/modify parameters.
		 */
		 void delay_( ushort milisec );

		 //! Format SDO Frame for sending to DES
		 /*!
			 Formats SDO Frame to send to DES when it reads/writes data
			 for any command. To format SDO Frame, these
			 informations about specific command are required:
		 
			 Operation Code (opCode_) - defines specific function for reading/writing to DES
			 Data Length (dataLength_) - defines input data length in words (16-bits)
			 Function Response (functionResponse_) - defines whether function has response from DES or not
			 Param1, Param2, Param3 - input parameters for specific function
		*/
		 void formatSDOSendFrame_(ushort Param1, ushort Param2, ushort Param3);

		 //! Execute functions for writing to and reading from DES
		 /*!
			 Call this function for any command from DES Command Reference
		 */
		 bool UpdateCmdToDES_(void);

		 //! Check if correct SDO Frame format was received
		 /*!
			 Checks whether received Frame is SDO (Service Data Object) or 
			 PDO (Process Data Object)
		 */
		 bool checkReceivedSDOFrameFormat(void);

		 //================================================================================================================//
		 //                         Definitions of DES command functions and related parameters                            //
		 //================================================================================================================//

		 //+++++++++++++++++++++++++++++++++++++++ "Status Functions" members +++++++++++++++++++++++++++++++++++++++++++++//

		 //! Read the system status of the DES. The system status is a 16-bit value containing different flags.
		 bool readSysStatusUpdate_(ushort Dummy, ushort &Status);

		 //! Read a 16-bit value of system errors.
		 bool readErrorUpdate_(ushort Dummy, ushort &Error);

		 //! Clear the system error.
		 bool clearErrorUpdate_(ushort Dummy);

		 //! Reset the system by restarting the software.
		 bool resetUpdate_(ushort Dummy);

		 //! Set the system into the enabled or disabled state. The DES has to be configured for a software setting of Enable.
		 //! If the hardware Enable is activated this command has no effect.
		 bool enableUpdate_(DESSystemStates newState);

		 // +++++++++++++++++++++++++++++++++++ "System Parameter Functions" members ++++++++++++++++++++++++++++++++++++++//

		 //! Read the requested temporary system parameter from DES-RAM.
		 bool readTempParamUpdate_(ushort paramNb, DataFormat dataFormat, ulong &value);

		 //! Write a new value to a temporary system parameter. Refer to the section about system parameters to find
		 //! the desired system parameter numbers. !!!Only for CAN!!!
		 bool setTempParamUpdate_(ushort paramNb, DataFormat dataFormat, ushort value);

		 //! Copy the permanent system parameter contained in the EEPROM memory into the temporary parameter set.
		 bool resetTempParamUpdate_(ushort Dummy);

		 //! Save the temporary parameters to the EEPROM (non volatile memory).
		 bool saveTempParamUpdate_(ushort Dummy);

		 //! Set all system parameters to default. The system parameter structure is described in the section ‘Data Structures’.
		 bool sysParSetDefaultUpdate_(ushort Dummy);

		 // ++++++++++++++++++++++++++++++++++++++++ "Setting Functions" members ++++++++++++++++++++++++++++++++++++++++++//

		 //! Set a new velocity of the rotor. This function is only available in speed regulation mode.
		 bool setVelocityUpdate_(short newVelocity);

		 //! Set a new current amplitude. This function is only available in current regulation mode.
		 bool setCurrentUpdate_(short newCurrent);

		 //! This command changes the stopping state. If the motor is already stopped it will be released. The digital
		 //! input STOP has the same behaviour. Only for speed regulation mode!
		 bool stopMotionUpdate_(ushort Dummy);

		 // ++++++++++++++++++++++++++++++++++++++++ "Monitor Functions" members ++++++++++++++++++++++++++++++++++++++++++//

		 //! Read the effective and requested velocity of the motor.
		 bool readVelocityIsMustUpdate_(MonitorValueTypes Type, short &isVelocity, short &mustVelocity);

		 //! Read the effective and requested current components of the motor.
		 bool readCurrentIsMustUpdate_(MonitorValueTypes Type, short &isCurrent, short &mustCurrent);

		 // +++++++++++++++++++++++++++++++++ "CAN Bus Configuration Functions" members +++++++++++++++++++++++++++++++++++//

		 //! Set CAN Module-ID (max. 11bit). The module ID is set by DIP switches at the system power up. During operation
		 //! it's possible to overwrite temporary the moduleID with the command 'SetModuleID'. The moduleID determines the
		 //! IDs for SDO communication (TxSDO ID = 1408 + Module-ID; RxSDO ID = 1536 + Module-ID).
		 bool setModuleIDUpdate_(ushort moduleID);

		 //! Set CAN Transmit-PDO ID (11 bit). This is the message ID sent by the DES.
		 bool setTPDOIDUpdate_(ushort tpdoID);

		 //! Set CAN Receive-PDO ID (11 bit). This is the message ID received by the DES.
		 bool setRPDOIDUpdate_(ushort rpdoID);

		 //! Read DES CAN Module-ID. The module ID is set by DIP switches at the system power up. It's possible that the 
		 //! system parameter 'moduleID' has another value than the DIP switches. It's possible to overwrite this value temporary.
		 bool readModuleIDUpdate_(ushort Dummy, ushort &moduleID);

		 //! Set CAN transfer rate to calculated values. See the section Bit Timing for more information about this configuration register.
		 bool setCANBitrateUpdate_(DESCANBitrate bitrate);

		 //! Read a 16 bit value of the CAN error register.
		 bool readCANErrorUpdate_(ushort Dummy, ushort &ErrorCode);

		 //! Switch on and off the PDO communication. The state of the PDO communication can be read with the system
		 //! parameter 'CAN Config' (SysParam 41, Bit14).
		 bool configPDOUpdate_(PDOConfiguration action);

		 //! Reset the CAN Error.
		 bool resetCANErrorUpdate_(ushort Dummy);

		 //! Reset the CAN communication.
		 bool resetCANUpdate_(ushort Dummy);

		 //================================================================================================================//
		 //                      End of definitions of DES command functions and related parameters                        //
		 //================================================================================================================//





		 //================================================================================================================//
		 //                                    DES System Parameters definitions                                           //
		 //================================================================================================================//

		 //+++++++++++++++++++++++++++++++++++ Functions for modifying regulator parameters +++++++++++++++++++++++++++++++//

		 //! Set current controller P-gain
		 bool setCurrentRegPGainUpdate_(ushort Gain);

		 //! Get current controller P-gain
		 bool getCurrentRegPGainUpdate_(ushort &Gain);

		 //! Set current controller I-gain
		 bool setCurrentRegIGainUpdate_(ushort Gain);

		 //! Get current controller I-gain
		 bool getCurrentRegIGainUpdate_(ushort &Gain);

		 //! Set velocity controller P-gain
		 bool setVelocityRegPGainUpdate_(ushort Gain);

		 //! Get velocity controller P-gain
		 bool getVelocityRegPGainUpdate_(ushort &Gain);

		 //! Set velocity controller I-gain
		 bool setVelocityRegIGainUpdate_(ushort Gain);

		 //! Get velocity controller I-gain
		 bool getVelocityRegIGainUpdate_(ushort &Gain);

		 //+++++++++++++++++++++++++++ Functions for modifying motor and sensor data parameters ++++++++++++++++++++++++++//

		 //! Set peak-current value
		 bool setPeakCurrentUpdate_(ushort Current);

		 //! Get peak-current value
		 bool getPeakCurrentUpdate_(ushort &Current);

		 //! Set nominal current value
		 bool setNominalCurrentUpdate_(ushort Current);

		 //! Get nominal current value
		 bool getNominalCurrentUpdate_(ushort &Current);

		 //! Set encoder resolution
		 bool setEncoderResolutionUpdate_(ushort Resolution);

		 //! Get encoder resolution
		 bool getEncoderResolutionUpdate_(ushort &Resolution);

		 //! Set pole pairs, no.
		 bool setPolePairsUpdate_(ushort PolePairs);

		 //! Get pole pairs, no.
		 bool getPolePairsUpdate_(ushort &PolePairs);

		 //+++++++++++++++++++++++++++++++++++ Functions for modifying motion parameters ++++++++++++++++++++++++++++++++++//

		 //! Set max possible speed
		 bool setMaximalSpeedUpdate_(ushort Speed);

		 //! Get max possible speed
		 bool getMaximalSpeedUpdate_(ushort &Speed);

		 //! Set motor acceleration
		 bool setMotorAccelerationUpdate_(ushort Acceleration);

		 //! Get motor acceleration
		 bool getMotorAccelerationUpdate_(ushort &Acceleration);

		 //! Set max speed in current regulation mode
		 bool setMaxSpeedCurrentModeUpdate_(ushort Speed);

		 //! Get max speed in current regulation mode
		 bool getMaxSpeedCurrentModeUpdate_(ushort &Speed);

		 //! Set ErrorProc parameter
		 bool setErrorProcUpdate_(ErrorProc Processing);

		 //! Get ErrorProc parameter
		 bool getErrorProcUpdate_(ErrorProc &Processing);

		 //+++++++++++++++++++++++++++++++ Functions for reading some CAN communication parameters ++++++++++++++++++++++++//

		 //! Get CAN module ID
		 bool getCANmoduleIDUpdate_(ushort &ID);

		 //! Get CAN RxPDO ID
		 bool getCANRxPDOIDUpdate_(ushort &RxPDO_ID);

		 //! Get CAN TxPDO ID
		 bool getCANTxPDOIDUpdate_(ushort &TxPDO_ID);

		 //! Get CAN RxSDO ID
		 bool getCANRxSDOIDUpdate_(ushort &RxSDO_ID);

		 //! Get CAN TxSDO ID
		 bool getCANTxSDOIDUpdate_(ushort &TxSDO_ID);

		 //================================================================================================================//
		 //                                 End of DES System Parameters definitions                                       //
		 //================================================================================================================//





		 //================================================================================================================//
		 //                                    DES Status Variables definitions                                            //
		 //================================================================================================================//

		 //! Get system operating status
		 bool getSystemOperatingStatusUpdate_(ushort &SystemStatus);

		 //! Get actual mean current value in d-axis
		 bool getActualMeanCurrD_AxisUpdate_(short &MeanCurrent);

		 //! Get actual mean current value in q-axis
		 bool getActualMeanCurrQ_AxisUpdate_(short &MeanCurrent);

		 //! Get current setting value
		 bool getCurrentSettingValueUpdate_(short &Current);

		 //! Get speed setting value
		 bool getSpeedSettingValueUpdate_(short &Speed);

		 //! Get actual mean speed value
		 bool getSpeedActualMeanValueUpdate_(short &Speed);

		 //! Get absolute rotor position
		 bool getAbsoluteRotorPositionUpdate_(long &Position);

		 //! Get standard error
		 bool getStandardErrorUpdate_(short &Error);

		 //! Get actual current value in q-axis (=> Torque) (not averaged)
		 bool getActualCurrentQAxisUpdate_(short &Current);

		 //! Get actual speed value (not averaged)
		 bool getActualSpeedValueUpdate_(short &Speed);

		 //! Get encoder counter
		 bool getEncoderCounterUpdate_(short &Counter);

		 //================================================================================================================//
		 //                                 End of DES Status Variables definitions                                        //
		 //================================================================================================================//

		 //! Copy constructor.
		 /*!
			 Forbid copying motor objects.
		  */
		 MotorDES( const MotorDES& motor );

		 //! Assignment operator.
		 /*! 
			 Forbid assigning motor objects.
		  */
		 MotorDES& operator=( const MotorDES& motor );

};

} // Vatroslav

#endif