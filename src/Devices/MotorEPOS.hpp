/*!	\file	MotorEPOS.hpp
	\brief	MotorEPOS class definition.
	//==========================================================================================//
	//                           DEFINITIONS OF THE MOTOR CLASSES                               //

	//                            Author: Kristijan Brkic, 2009.                                //
	//==========================================================================================//

	(Note: If any class method is declared with include-word "Update" or with uppercase characters, 
	       its using causes communication with servodrive!)
 */

#ifndef VATROSLAV_MOTOR_EPOS_HPP
#define VATROSLAV_MOTOR_EPOS_HPP

#include <memory>
#include <cassert>
#include <string>

#include "../Communication/Communication.hpp"
#include "EPOSDeclarations.hpp"
#include "Motor.hpp"

namespace Vatroslav
{

//=============================================================================

//! Regulator parameters class (RegParEPOS)
/*!
	This class defines gain parameters of current and velocity PI controllers 
	and	gain parameters of position PID controller.

	@param currRegGainP - proportional gain of current controller
	@param currRegGainI - integral gain of current controller
	
	@param velRegGainP  - proportional gain of velocity controller
	@param velRegGainI  - integral gain of velocity controller

	@param posRegGainP  - proportional gain of position controller
	@param posRegGainI  - integral gain of position controller
	@param posRegGainD  - derivative gain of position controller
*/
class RegParEPOS
{
	public:
		// Data type appropriate for regulator params.
		typedef unsigned short ushort;

		//! Constructor for regulator params (also default constructor)
		/*!
			Defines all default gain parameters for normal robot operation
		*/
		RegParEPOS( ushort currRegGainP = 1125, ushort currRegGainI = 656, 
				ushort velRegGainP = 850, ushort velRegGainI = 150,
				ushort posRegGainP = 550, ushort posRegGainI = 30, ushort posRegGainD = 150);
		

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
		
		//////////////////////////////////////////////////////////////////////

		//! Get position controller gain P
		ushort GetPosRegGainP(void) const;

		//! Set position controller gain P
		void SetPosRegGainP(ushort posRegGainP);

		//! Get position controller gain I
		ushort GetPosRegGainI(void) const;

		//! Set position controller gain I
		void SetPosRegGainI(ushort posRegGainI);

		//! Get position controller gain D
		ushort GetPosRegGainD(void) const;

		//! Set position controller gain D
		void SetPosRegGainD(ushort posRegGainD);

	private:
		// Current PI controller parameters
		ushort currRegGainP_;		// Proportional gain
		ushort currRegGainI_;		// Integral gain

		// Velocity PI controller parameters
		ushort velRegGainP_;		// Proportional gain
		ushort velRegGainI_;		// Integral gain
		
		// Position PID controller parameters
		ushort posRegGainP_;		// Proportional gain
		ushort posRegGainI_;		// Integral gain
		ushort posRegGainD_;		// Derivative gain
};

//=============================================================================

//! Parameters for motor motion (MotionParEPOS)
/*!
    This class defines parameters for motor motion:

	@param maxFollowingErr		- Maximal allowed difference of position actual value to position demand value
	@param positionWindowSize	- Tolerance (+/- depending on demand final position state) to determine the target is reached
	@param positionWindowTime	- When actual position is within Window Size, the target is reached if this time has elapsed
	@param minPositionLimit	- Minimal possible motor position
	@param maxPositionLimit	- Maximal possible motor position
	@param maxProfileVelocity	- Maximal possible motor profile velocity in profile position operation mode
	@param profileAcceleration	- Profile acceleration in profile velocity/position operation mode
	@param profileDeceleration	- Profile deceleration in profile velocity/position operation mode
	@param quickStopDeceleration-Quick deceleration when quickstop command is called
	@param profileVelocity		- Profile velocity in position operation mode
	@param motionProfileType	- Define the type of the motion profile (linear or sin2) in Profile Position Mode, Homing Mode or Profile Velocity Mode
	@param motorOperationMode	- Define motor operation mode: profile position, profile velocity, ... etc.

*/

class MotionParEPOS
{
	public:
		typedef unsigned long ulong;
		typedef unsigned short ushort;

		//! Constructor for motor motion parameters
		/*!
			
		*/
		MotionParEPOS( OperationModes motorOperationMode, 
				   ushort maxProfileVelocity = 10000, 
				   ulong profileAcceleration = 1500,
				   ulong profileDeceleration = 1800, 
				   ulong quickStopDeceleration = 10000, 
				   ushort profileVelocity = 1000, 
				   MotionProfileTypes motionProfileType = LINEAR_RAMP, 
				   ulong maxFollowingErr = 2000,
				   ulong positionWindowSize = 4294967295, 
				   ushort positionWindowTime = 0, 
				   long minPositionLimit = -2147483647,
				   long maxPositionLimit = 2147483647 );

		//! Get maximal allowed following error
		ulong GetMaxFollowingErr(void) const;

		//! Set maximal allowed following error
		void SetMaxFollowingErr(ulong maxFollowingErr);

		//! Get position window size
		ulong GetPositionWindowSize(void) const;

		//! Set position window size
		void SetPositionWindowSize(ulong positionWindowSize);

		//! Get position window time
		ushort GetPositionWindowTime(void) const;

		//! Set position window time
		void SetPositionWindowTime(ushort positionWindowTime);

		//! Get minimal software position limit
		long GetMinimalPositionLimit(void) const;

		//! Set minimal software position limit
		void SetMinimalPositionLimit(long minPositionLimit);

		//! Get maximal software position limit
		long GetMaximalPositionLimit(void) const;

		//! Set maximal software position limit
		void SetMaximalPositionLimit(long maxPositionLimit);

		//! Get maximal profile velocity
		ushort GetMaximalProfileVelocity(void) const;

		//! Set maximal profile velocity
		void SetMaximalProfileVelocity(ushort maxProfileVelocity);

		//! Get profile acceleration
		ulong GetProfileAcceleration(void) const;

		//! Set profile acceleration
		void SetProfileAcceleration(ulong profileAcceleration);

		//! Get profile deacceleration
		ulong GetProfileDeceleration(void) const;

		//! Set profile deacceleration
		void SetProfileDeceleration(ulong profileDeceleration);

		//! Get quick stop deceleration
		ulong GetQuickStopDeceleration(void) const;

		//! Set quick stop deceleration
		void SetQuickStopDeceleration(ulong quickStopDeceleration);

		//! Get profile velocity
		ushort GetProfileVelocity(void) const;

		//! Set profile velocity
		void SetProfileVelocity(ushort profileVelocity);

		//! Get motion profile type
		MotionProfileTypes GetMotionProfileType(void) const;

		//! Set motion profile type
		void SetMotionProfileType(MotionProfileTypes motionProfileType);

		//! Get motor operation mode
		OperationModes GetMotorOperationMode(void) const;

		//! Set motor operation mode
		void SetMotorOperationMode(OperationModes motorOperationMode);


	private:
		// Maximal allowed difference of position actual value to position demand value
		ulong		maxFollowingErr_;

		// Position window size
		ulong		positionWindowSize_;

		// Position window time
		ushort		positionWindowTime_;

		// Minimal software position limit
		long		minPositionLimit_;

		// Maximal software position limit
		long		maxPositionLimit_;

		// Maximal profile velocity
		ushort		maxProfileVelocity_;

		// Profile acceleration
		ulong		profileAcceleration_;

		// Profile deacceleration
		ulong		profileDeceleration_;

		// Quick stop deceleration
		ulong		quickStopDeceleration_;

		// Profile velocity
		ushort		profileVelocity_;

		// Motion profile type
		MotionProfileTypes motionProfileType_;

		// Motor operation modes
		OperationModes motorOperationMode_;

};

//=============================================================================

//! Motor parameters class (MotorParEPOS)
/*!
	This class defines motor type with these parameters:

	@param ID				- motor identification number
	@param regPar			- motor regulator parameters
	@param motionPar		- motor motion parameters

	@param noPolePairs		- number of motor Pole Pairs
	@param maxPermissSpeed - maximal permissible speed
	@param nominalCurrent	- nominal (max. continous) current
	@param thermalTimeConst - thermal time constant winding
	@param encResolution	- encoder resolution

	@param sensorType	    - position sensor type
	@param motorType		- type of motor

*/

class MotorParEPOS
{
	public:

		// Less typing :-)
		typedef unsigned short ushort;
		typedef unsigned long ulong;

		//! Constructor for motor data.
		/*!
			Constructs motor parameters for Vatroslav motors.
		*/
		MotorParEPOS( ushort ID, 
				  RegParEPOS regPar, 
				  MotionParEPOS motionPar, 
				  ushort noPolePairs = 1, 
			      ushort maxPermissSpeed = 12000, 
				  ulong nominalCurrent = 4720, 
				  ushort thermalTimeConst = 308, 
				  ushort encResolution = 500, 
				  PosSensorType sensorType = INC_ENCODER_INDEX, 
				  MotorType motorType = SINUSOIDAL_PM_BL);

		//! Get motor ID
		ushort GetMotorID(void) const;

		//! Get controller parameters
		RegParEPOS& RegParams(void);

		//! Get motion parameters
		MotionParEPOS& MotionParams(void);

		//! Get number of pole pairs
		ushort GetNumberPolePairs(void) const;

		//! Set number of pole pairs
		void SetNumberPolePairs(ushort noPolePairs);

		//! Get maximal permissible speed
		ushort GetMaximalSpeed(void) const;

		//! Set maximal permissible speed
		void SetMaximalSpeed(ushort maxPermissSpeed);

		//! Get maximal continous current
		ulong GetMaximalCurrent(void) const;

		//! Set maximal continous current (maximal value depends on hardware)
		void SetMaximalCurrent(ulong nominalCurrent);

		//! Get thermal time constant winding
		ushort GetThermalTimeConst(void) const;

		//! Set thermal time constant winding
		void SetThermalTimeConst(ushort thermalTimeConst);

		//! Get encoder resolution 
		ushort GetEncoderResolution(void) const;

		//! Set encoder resolution
		void SetEncoderResolution(ushort encResolution);

		//! Get position sensor type
		PosSensorType GetSensorType(void) const;

		//! Set position sensor type
		void SetSensorType(PosSensorType sensorType);

		//! Get motor type
		MotorType GetMotorType(void) const;

		//! Set motor type
		void SetMotorType(MotorType motorType);


	private:

		// Regulator parameters
		RegParEPOS			regPar_;

		// Motion parameters
		MotionParEPOS		motionPar_;

		// Motor identification number on network
		ushort			ID_;

		// Some motor technical data
		ushort			noPolePairs_;
		ushort			maxPermissSpeed_;
		ulong			nominalCurrent_;
		ushort			thermalTimeConst_;
		ushort			encResolution_;

		// Position sensor type 
		PosSensorType	sensorType_;
		
		// Type of the motor
		MotorType		motorType_;

};

//=============================================================================
//! The Motor class.
/*!

 */
class MotorEPOS : public Motor
{
 public:
	
	typedef unsigned char uchar;

	//! Constructor.
	/*!

	 */
	MotorEPOS( MotorParEPOS motpar, CommPtr pComm );

	//! Destructor.
	/*!

	 */
	~MotorEPOS();
	
	//! Get motor parameters
	/*!

	 */
	MotorParEPOS& Params( void ) { return motPar_; }

	///// Read actual current, velocity or position /////
	//! Read actual current
	virtual long Current(void) const;

	//! Read actual position
	virtual long Position(void) const;

	//! Read actual velocity
	virtual long Velocity(void) const;
	
	///// Set reference position or velocity /////
	//! Set motor position reference (only Profile Position Mode)
	virtual void PositionRef(long Position);

	//! Set motor velocity reference (only Profile Velocity Mode)
	virtual void VelocityRef(long Velocity);

	///// Get reference position or velocity /////
	//! Get motor position reference (only Profile Position Mode)
	virtual long PositionRef(void) const;

	//! Get motor velocity reference (only Profile Velocity Mode)
	virtual long VelocityRef(void) const;

	///// Set absolute/relative movement and immediately run settings /////
	//! Set absolute (true) or relative (false) motor movement
	void SetAbsoluteMovement(bool Absolute);

	//! Set if motor is updated immediately on reference change or not
	void SetImmediateRef(bool Immediate);

	///// Get absolute/relative movement and immediately run settings /////
	//! Get absolute (true) or relative (false) motor movement
	bool GetAbsoluteMovement(void) const;

	//! Get if motor is updated immediately on reference change or not
	bool GetImmediateRef(void) const;

	//! Get last measured analog input1 value
	long AnalogInput1(void) const;

	//! Get last measured analog input2 value
	long AnalogInput2(void) const;

	//! Get communication error code (error if nonzero)
	long GetCommErrorCode(void) const;

	//! Get communication error description
	std::string GetErrorDescription(void);

	////////////////////////////////// UPDATE PART ///////////////////////////////////

	///// Update regulator parameters by reading from and writing values to EPOS /////
	//! Read values
	bool UpdateRegParRead( void );

	//! Write values
	bool UpdateRegParWrite( void );


	///// Update motor motion parameters by reading from and writing values to EPOS /////
	//! Read values
	bool UpdateMotionParRead( void );

	//! Write values
	bool UpdateMotionParWrite( void );


	///// Update motor parameters by reading from and writing values to EPOS /////
	//! Read values
	bool UpdateMotorParRead( void );

	//! Write values
	bool UpdateMotorParWrite( void );


	//! Update actual motor current, speed and velocity, read all data
	bool UpdateReadMeasuredData( void );


	//! Set references for motor speed or position
	bool UpdateSetReferences( void );

	//! Read all analog inputs on EPOS
	/*!
		Reads analog inputs from input1 and input2
	*/
	bool UpdateAnalogInputs( void );

	//! Connect to Device.
	/*!
	    "Connect" method updates all parameters to EPOS device that
		was set in construction of "RegParEPOS", "MotionParEPOS" and "MotorParEPOS"
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
		from EPOS device.
	 */
	virtual bool UpdateRead( void );

	//! Write setpoints to motor.
	/*! Prepares EPOS states for motor running and
		updates velocity and position setpoints before motor starts. Depending on
		Profile Velocity/Position Mode, approriate setpoints will be used.

	 */
	virtual bool UpdateWrite( void );

	//! Stop motor immediately with high deceleration
	/*!
	    This method stops motor movement very fast. The used deceleration can be
		taken by "GetQuickStopDeceleration" method from "MotionParEPOS" object.
	*/
	virtual bool QUICKSTOP(void);

	//! Halts motor movement
	/*!
	    This method halts slowly motor movement. The used deceleration can be
		taken by "GetProfileDeceleration" method from "MotionParEPOS" object. In 
		any profile operation mode, this parameter is used!
	*/
	virtual bool HALT(void);

	//! Determine if predefined target is reached
	/*!
	    Method checks if settled reference is equal to actual value. The type
		of control variable depends on operation mode (Profile Velocity Mode -
		velocity or Profile Position Mode - position). Of course, the variable
		value must be inside position window to conclude that target is achieved.
	*/
	virtual bool IS_TARGET_REACHED(void);

	//! Gets state of the EPOS drive and motor
	/*!
		Method gives actual state or mode of operation of the controller. The
		device can be in 7 states, and user can carry out some actions according
		to result from this method. This method also gives the state of the motor.
	*/
	DriveStates GET_EPOS_STATE(void);
	
	//! Disable voltage on EPOS controller
	/*!
		Disable voltage on EPOS and disable also any possible movements; leave device
		in a safe state whatever the power is turned on.
	*/
	bool DISABLE_EPOS(void);

	//! Get  motor status.
	/*! 
		Return current motor status, ENABLED, DISABLED or ERROR.
	 */
	virtual MotorStatus Status( void );


 private:

	 //! Actual current variable
	 long actualCurrent_;

	 //! Actual position variable
	 long actualPosition_;

	 //! Actual velocity variable
	 long actualVelocity_;

	 //! Position reference variable
	 long positionRef_;

	 //! Velocity reference variable
	 long velocityRef_;

	 //! Move motor absolute if true, or relative if false
	 bool absoluteRun_;

	 //! Change setpoint immediately if true, or when the target is reached if false
	 bool immediatelyRun_;

	 //! Analog input1 value
	 long anainput1_;

	 //! Analog input2 value
	 long anainput2_;

	 //! Motor parameters
	 MotorParEPOS motPar_;
	 
	 //! Determine if motor is connected or not
	 bool motorConnected_;

	 //! Communication parameters
	 CommPtr pComm_; 
	 
	 //! SDO Frame COB-ID (Write: 0x600 + Node-ID; Read: 0x580 + Node-ID)
	 unsigned _COB_ID_;

	 //! Data variable for sending SDO Frame
	 char _dataSendSDO_[8];

	 //! Data frame for sending in serial communication
	 char _dataSendSerial_[9];

	 //! Data variable for receiving SDO Frame
	 char _dataReceiveSDO_[8];
 
	 //! Data frame received in serial communication
	 char _dataReceiveSerial_[9];

	 //! Index from written or readed SDO Frame
	 unsigned _Index_;
	 
	 //! SubIndex from written or readed SDO Frame
	 uchar _SubIndex_;

	 //! Command specifier (depend on size of data) by writing or reading
	 uchar _cmdSpec_;
	 
	 //! Data that was readed from or data to write to EPOS
	 long _Data_;

	 //! Error code from CANopen communication
	 unsigned _ErrorCode_;

	 //! Last status of the motor
	 MotorStatus _status_;

	 //! Operation types, reading or writing to EPOS
	 OperationType _TypeOfOperation_;

	 //! Update command for writing to and reading from EPOS
	 /*!
	     Call this function for any command in Object Dictionary
	 */
	 bool UpdateCmdToEPOS_(void);

	 //! Check if correct SDO Frame format was received
	 /*!
	     Checks whether received Frame is SDO (Service Data Object) or 
		 PDO (Process Data Object)
	 */
	 bool checkReceivedSDOFrameFormat(void);


	 //! Format SDO Frame for sending to EPOS
	 /*!
	     Formats SDO Frame to send to EPOS when it reads data 
		 for any command. To format SDO Frame, these
		 informations about specific command are required:
		 
		 Command specifier (_cmdSpec_) - depends on data size to write or read
		 Index (_Index_)               - defines command in Object Dictionary
		 Sub-Index (_SubIndex_)        - additionally defines command in Object Dictionary
		 Data (_Data_)                 - Data to send to EPOS (only if WRITE_TO_EPOS OperationType is selected)
	 */
	 void formatSDOSendFrame_(void);

	 //! Format frame for sending to EPOS using serial communication
	 /*!
		 Formats data frame to send to EPOS in serial communication.
	 */
	 void formatSerialCommSendFrame_(void);

	 //! Set data values received from EPOS
	 /*!
	     Set data values received from EPOS for any command
		 in reading or writing process. When SDO Frame is received,
		 the following data is set:

		 Command specifier (_cmdSpec_) - depends on data size to write or read
		 Index (_Index_)               - defines command in Object Dictionary
		 Sub-Index (_SubIndex_)        - additionally defines command in Object Dictionary
		 Data (_Data_)                 - Data to received from EPOS (only if READ_FROM_EPOS OperationType is selected)
	 */
	 void formatReceivedDataSDO_(void);

	 //! Set data value received from EPOS using serial communication
	 /*!
		 Set data value readed from EPOS device using serial communication.
	 */
	 void formatReceivedDataSerial_(void);

	 //! Run motor movement
	 /*!
	     This class method enables motor movement; in Profile Position Mode
		 it runs absolute or relative according to home position, and it can
		 change its setpoints immediately or when the target is reached.
	 */
	 bool RUN(void);


     //================================================================================================================//
	 //                    DEFINITIONS OF PRIVATE FUNCTIONS FOR ALL EPOS OBJECTS IN OBJECT DICTIONARY                  //
	 //================================================================================================================//

	 //! Set CAN Bitrate (!!requires EPOS restart!!)
	 bool setCANBitrateUpdate_(CANBitrate Bitrate);

	 //! Get CAN Bitrate
	 bool getCanBitrateUpdate_(CANBitrate &Bitrate);

	 //! Set RS232 Baudrate (!!requires EPOS restart!!)
	 bool setRS232BaudrateUpdate_(RS232Baudrate Baudrate);

	 //! Get RS232 Baudrate
	 bool getRS232BaudrateUpdate_(RS232Baudrate &Baudrate);

	 // ++++++++++++++++++++++++++++++++++ Functions for members in "MotorParEPOS" ++++++++++++++++++++++++++++++++++++++++//

	 //! Set type of Maxon motor
	 bool setMotorTypeUpdate_(MotorType Type);

	 //! Get type of Maxon motor
	 bool getMotorTypeUpdate_(MotorType &Type);

	 //! Set current limit in [mA]
	 bool setCurrentLimitUpdate_(long Current);

	 //! Get current limit in [mA]
	 bool getCurrentLimitUpdate_(long &Current);

	 //! Set pole pairs, no.
	 bool setPolePairsUpdate_(long PolePairs);

	 //! Get pole pairs, no.
	 bool getPolePairsUpdate_(long &PolePairs);

	 //! Set thermal time constant of motor winding
	 bool setThermalTimeConstUpdate_(long ThermalTime);

	 //! Get thermal time constant of motor winding
	 bool getThermalTimeConstUpdate_(long &ThermalTime);

	 //! Set maximal motor speed in [rpm]
	 bool setMaximalSpeedUpdate_(long MaxSpeed);

	 //! Get maximal motor speed in [rpm]
	 bool getMaximalSpeedUpdate_(long &MaxSpeed);

	 //! Set encoder resolution
	 bool setEncoderResolutionUpdate_(long Resolution);

	 //! Get encoder resolution
	 bool getEncoderResolutionUpdate_(long &Resolution);

	 //! Set position sensor type
	 bool setPositionSensorTypeUpdate_(PosSensorType Type);

	 //! Get position sensor type
	 bool getPositionSensorTypeUpdate_(PosSensorType &Type);

	 // ++++++++++++++++++++++++++++++++++ Functions for members in "RegParEPOS" ++++++++++++++++++++++++++++++++++++++++//

	 //! Set current controller P-gain
	 bool setCurrentRegPGainUpdate_(long Gain);

	 //! Get current controller P-gain
	 bool getCurrentRegPGainUpdate_(long &Gain);

	 //! Set current controller I-gain
	 bool setCurrentRegIGainUpdate_(long Gain);

	 //! Get current controller I-gain
	 bool getCurrentRegIGainUpdate_(long &Gain);

	 //! Set velocity controller P-gain
	 bool setVelocityRegPGainUpdate_(long Gain);

	 //! Get velocity controller P-gain
	 bool getVelocityRegPGainUpdate_(long &Gain);

	 //! Set velocity controller I-gain
	 bool setVelocityRegIGainUpdate_(long Gain);

	 //! Get velocity controller I-gain
	 bool getVelocityRegIGainUpdate_(long &Gain);

	 //! Set position controller P-gain
	 bool setPositionRegPGainUpdate_(long Gain);

	 //! Get position controller P-gain
	 bool getPositionRegPGainUpdate_(long &Gain);

	 //! Set position controller I-gain
	 bool setPositionRegIGainUpdate_(long Gain);

	 //! Get position controller I-gain
	 bool getPositionRegIGainUpdate_(long &Gain);

	 //! Set position controller D-gain
	 bool setPositionRegDGainUpdate_(long Gain);

	 //! Get position controller D-gain
	 bool getPositionRegDGainUpdate_(long &Gain);

	 // ++++++++++++++++++++++++++++++++++ Functions for members in "MotionParEPOS" ++++++++++++++++++++++++++++++++++++++++//

	 //! Set motor operation mode
	 bool setOperationModeUpdate_(OperationModes OpMode);

	 //! Get motor operation mode
	 bool getOperationModeUpdate_(OperationModes &OpMode);

	 //! Set maximal profile velocity
	 bool setMaxProfileVelocityUpdate_(long MaxVelocity);

	 //! Get maximal profile velocity
	 bool getMaxProfileVelocityUpdate_(long &MaxVelocity);

	 //! Set profile acceleration
	 bool setProfileAccelerationUpdate_(long Acceleration);

	 //! Get profile acceleration
	 bool getProfileAccelerationUpdate_(long &Acceleration);

	 //! Set profile deceleration
	 bool setProfileDecelerationUpdate_(long Deceleration);

	 //! Get profile deceleration
	 bool getProfileDecelerationUpdate_(long &Deceleration);

	 //! Set quickstop deceleration
	 bool setQuickstopDecelerationUpdate_(long QuickDeceleration);

	 //! Get quickstop deceleration
	 bool getQuickstopDecelerationUpdate_(long &QuickDeceleration);

	 //! Set profile velocity
	 bool setProfileVelocityUpdate_(long Velocity);

	 //! Get profile velocity
	 bool getProfileVelocityUpdate_(long &Velocity);

	 //! Set motion profile type
	 bool setMotionProfileTypeUpdate_(MotionProfileTypes MotionType);

	 //! Get motion profile type
	 bool getMotionProfileTypeUpdate_(MotionProfileTypes &MotionType);

	 //! Set maximal following error
	 bool setMaxFollowingErrorUpdate_(long MaxError);

	 //! Get maximal following error
	 bool getMaxFollowingErrorUpdate_(long &MaxError);

	 //! Set position window size
	 bool setPositionWindowSizeUpdate_(long Size);

	 //! Get position window size
	 bool getPositionWindowSizeUpdate_(long &Size);

	 //! Set position window time
	 bool setPositionWindowTimeUpdate_(long Time);

	 //! Get position window time
	 bool getPositionWindowTimeUpdate_(long &Time);

	 //! Set minimal position limit
	 bool setMinPositionLimitUpdate_(long PositionLimit);

	 //! Get minimal position limit
	 bool getMinPositionLimitUpdate_(long &PositionLimit);

	 //! Set maximal position limit
	 bool setMaxPositionLimitUpdate_(long PositionLimit);

	 //! Get maximal position limit
	 bool getMaxPositionLimitUpdate_(long &PositionLimit);

	 // ++++++++++++++++++++++++++++++++++ Functions for reading current, velocity and position +++++++++++++++++++++++//

	 //! Get actual motor current
	 bool getActualCurrentUpdate_(long &Current);

	 //! Get actual motor velocity
	 bool getActualVelocityUpdate_(long &Velocity);

	 //! Get actual motor position
	 bool getActualPositionUpdate_(long &Position);


	 // ++++++++++++++++++++++++++++++++++ Setting reference for motor velocity and position +++++++++++++++++++++++//

	 //! Set position reference value
	 bool setPositionRefUpdate_(long Position);

	 //! Set velocity reference value
	 bool setVelocityRefUpdate_(long Velocity);

	 // ++++++++++++++++++++++++++++++++++ Store or load parameters from device volatile memory +++++++++++++++++++++++//

	 //! Save parameters
	 bool saveParametersUpdate_(void);

	 //! Load parameters
	 bool loadParametersUpdate_(void);


	 // +++++++++++++++++++++++++++++++++++++ Functions for reading EPOS analog inputs +++++++++++++++++++++++++++++++++//

	 //! Get analog input 1 measured value
	 bool getAnalogInput1Update_(long &Input1);

	 //! Get analog input 2 measured value
	 bool getAnalogInput2Update_(long &Input2);

	 // ++++++++++++++++++++++++++++++++++ Functions for changing EPOS operating states +++++++++++++++++++++++++++++++//

	 //! Shut down device
	 bool shutDownUpdate_(void);

	 //! Switch on device
	 bool switchOnUpdate_(void);

	 //! Disable voltage
	 bool disableVoltageUpdate_(void);

	 //! Halt device
	 bool haltUpdate_(void);

	 //! Quick stop device
	 bool quickStopUpdate_(void);

	 //! Fault reset on device
	 bool faultResetUpdate_(void);

	 // ++++++++++++++++++++++++++++++ Function for changing CONTROLWORD SDO and run motor ++++++++++++++++++++++++++++//

	 //! Modify CONTROLWORD and run motor
	 bool runMotorUpdate_(int WriteData);

	 // +++++++++++++++++++++++ Function for reading STATUSWORD SDO to determine target reached +++++++++++++++++++++++//

	 //! Read STATUSWORD SDO
	 bool readSTATUSWORDUpdate_(int &Statusword);

	 //================================================================================================================//
	 //                  END OF DEFINITIONS OF PRIVATE FUNCTIONS FOR ALL EPOS OBJECTS IN OBJECT DICTIONARY             //
	 //================================================================================================================//


	 //! Copy constructor.
	 /*!
		 Forbid copying motor objects.
	  */
	 MotorEPOS( const MotorEPOS& motor );

	 //! Assignment operator.
	 /*! 
		 Forbid assigning motor objects.
	  */
	 MotorEPOS& operator=( const MotorEPOS& motor );

};


} // Vatroslav

#endif
