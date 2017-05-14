/*! \file	MotorEPOS.cpp
	\brief	MotorEPOS member declaration.
	//==========================================================================================//
	//                           DECLARATIONS OF THE MOTOR CLASSES                              //

	//                            Author: Kristijan Brkic, 2009.                                //
	//==========================================================================================//
 */

#include <iostream>
#include <memory.h>
#include <sys/timeb.h>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/local_time/local_time.hpp"

#include "../Communication/CommPrint.hpp"
#include "MotorEPOS.hpp"


namespace Vatroslav
{

namespace pt = boost::posix_time;

// Specific data types used in this file.
typedef unsigned short ushort;
typedef unsigned long ulong;
typedef unsigned char uchar;

//=============================================================================
// Object:						RegParEPOS
//=============================================================================

RegParEPOS::RegParEPOS( ushort currRegGainP, ushort currRegGainI, 
			    ushort velRegGainP, ushort velRegGainI,
	            ushort posRegGainP, ushort posRegGainI, ushort posRegGainD)
{
	// Make sure that gain params are in correct range.

	if (abs(currRegGainP) > MAX_GAIN_VALUE)
		currRegGainP_ = MAX_GAIN_VALUE;
	else
		currRegGainP_ = abs(currRegGainP);

	if (abs(currRegGainI) > MAX_GAIN_VALUE)
		currRegGainI_ = MAX_GAIN_VALUE;
	else
		currRegGainI_ = abs(currRegGainI);
	//////////////////////////////////////////
	if (abs(velRegGainP) > MAX_GAIN_VALUE)
		velRegGainP_ = MAX_GAIN_VALUE;
	else
		velRegGainP_ = abs(velRegGainP);

	if (abs(velRegGainI) > MAX_GAIN_VALUE)
		velRegGainI_ = MAX_GAIN_VALUE;
	else
		velRegGainI_ = abs(velRegGainI);
	//////////////////////////////////////////
	if (abs(posRegGainP) > MAX_GAIN_VALUE)
		posRegGainP_ = MAX_GAIN_VALUE;
	else
		posRegGainP_ = abs(posRegGainP);

	if (abs(posRegGainI) > MAX_GAIN_VALUE)
		posRegGainI_ = MAX_GAIN_VALUE;
	else
		posRegGainI_ = abs(posRegGainI);

	if (abs(posRegGainD) > MAX_GAIN_VALUE)
		posRegGainD_ = MAX_GAIN_VALUE;
	else
		posRegGainD_ = abs(posRegGainD);
}

//=============================================================================

ushort RegParEPOS::GetCurrRegGainP(void) const 
{ 
	return currRegGainP_; 
}

//=============================================================================

void RegParEPOS::SetCurrRegGainP(ushort currRegGainP) 
{
	if (abs(currRegGainP) > MAX_GAIN_VALUE)
		currRegGainP_ = MAX_GAIN_VALUE;
	else
		currRegGainP_ = abs(currRegGainP);
}

//=============================================================================

ushort RegParEPOS::GetCurrRegGainI(void) const 
{ 
	return currRegGainI_; 
}

//=============================================================================

void RegParEPOS::SetCurrRegGainI(ushort currRegGainI)
{
	if (abs(currRegGainI) > MAX_GAIN_VALUE)
		currRegGainI_ = MAX_GAIN_VALUE;
	else
		currRegGainI_ = abs(currRegGainI);
}

//=============================================================================

ushort RegParEPOS::GetVelRegGainP(void) const 
{ 
	return velRegGainP_; 
}

//=============================================================================

void RegParEPOS::SetVelRegGainP(ushort velRegGainP)
{
	if (abs(velRegGainP) > MAX_GAIN_VALUE)
		velRegGainP_ = MAX_GAIN_VALUE;
	else
		velRegGainP_ = abs(velRegGainP);
}

//=============================================================================

ushort RegParEPOS::GetVelRegGainI(void) const 
{ 
	return velRegGainI_; 
}

//=============================================================================

void RegParEPOS::SetVelRegGainI(ushort velRegGainI)
{
	if (abs(velRegGainI) > MAX_GAIN_VALUE)
		velRegGainI_ = MAX_GAIN_VALUE;
	else
		velRegGainI_ = abs(velRegGainI);
}

//=============================================================================

ushort RegParEPOS::GetPosRegGainP(void) const 
{ 
	return posRegGainP_; 
}

//=============================================================================

void RegParEPOS::SetPosRegGainP(ushort posRegGainP)
{
	if (abs(posRegGainP) > MAX_GAIN_VALUE)
		posRegGainP_ = MAX_GAIN_VALUE;
	else
		posRegGainP_ = abs(posRegGainP);
}

//=============================================================================

ushort RegParEPOS::GetPosRegGainI(void) const 
{ 
	return posRegGainI_; 
}

//=============================================================================

void RegParEPOS::SetPosRegGainI(ushort posRegGainI)
{
	if (abs(posRegGainI) > MAX_GAIN_VALUE)
		posRegGainI_ = MAX_GAIN_VALUE;
	else
		posRegGainI_ = abs(posRegGainI);
}

//=============================================================================

ushort RegParEPOS::GetPosRegGainD(void) const 
{ 
	return posRegGainD_; 
}

//=============================================================================

void RegParEPOS::SetPosRegGainD(ushort posRegGainD)
{
	if (abs(posRegGainD) > MAX_GAIN_VALUE)
		posRegGainD_ = MAX_GAIN_VALUE;
	else
		posRegGainD_ = abs(posRegGainD);
}

//=============================================================================
// Object:						MotorParEPOS
//=============================================================================

MotorParEPOS::MotorParEPOS(ushort ID, RegParEPOS regPar, MotionParEPOS motionPar, 
				   ushort noPolePairs, ushort maxPermissSpeed, ulong nominalCurrent,
				   ushort thermalTimeConst, ushort encResolution, PosSensorType sensorType,
				   MotorType motorType): 

                                                          ID_(ID),
														  regPar_(regPar),
														  motionPar_(motionPar),
														  noPolePairs_(noPolePairs),
														  maxPermissSpeed_(maxPermissSpeed),
														  nominalCurrent_(nominalCurrent),
														  thermalTimeConst_(thermalTimeConst),
														  encResolution_(encResolution),
														  sensorType_(sensorType),
														  motorType_(motorType)
		{
			if (abs(noPolePairs) > MAX_POLE_PAIRS)
				noPolePairs_ = MAX_POLE_PAIRS;
			else
				noPolePairs_ = abs(noPolePairs);

			if (abs(thermalTimeConst) > MAX_THERMAL_TIME)
				thermalTimeConst_ = MAX_THERMAL_TIME;
			else
				thermalTimeConst_ = abs(thermalTimeConst);

			if (abs(encResolution) > MAX_ENC_PULSE_NO)
				encResolution_ = MAX_ENC_PULSE_NO;
			else
				encResolution_ = abs(encResolution);
		}

//=============================================================================

ushort MotorParEPOS::GetMotorID(void) const 
{ 
	return ID_; 
}

//=============================================================================

RegParEPOS& MotorParEPOS::RegParams(void)
{
	return regPar_;
}

//=============================================================================

MotionParEPOS& MotorParEPOS::MotionParams(void)
{
	return motionPar_;
}

//=============================================================================

ushort MotorParEPOS::GetNumberPolePairs(void) const 
{ 
	return noPolePairs_; 
}

//=============================================================================

void MotorParEPOS::SetNumberPolePairs(ushort noPolePairs) 
{
	if (abs(noPolePairs) > MAX_POLE_PAIRS)
		noPolePairs_ = MAX_POLE_PAIRS;
	else
		noPolePairs_ = abs(noPolePairs);
}

//=============================================================================

ushort MotorParEPOS::GetMaximalSpeed(void) const 
{ 
	return maxPermissSpeed_; 
}

//=============================================================================

void MotorParEPOS::SetMaximalSpeed(ushort maxPermissSpeed) 
{ 
	maxPermissSpeed_ = maxPermissSpeed; 
}

//=============================================================================

ulong MotorParEPOS::GetMaximalCurrent(void) const 
{ 
	return nominalCurrent_; 
}

//=============================================================================

void MotorParEPOS::SetMaximalCurrent(ulong nominalCurrent) 
{
	nominalCurrent_ = nominalCurrent; 
}

//=============================================================================

ushort MotorParEPOS::GetThermalTimeConst(void) const 
{ 
	return thermalTimeConst_; 
}

//=============================================================================

void MotorParEPOS::SetThermalTimeConst(ushort thermalTimeConst) 
{
	if (abs(thermalTimeConst) > MAX_THERMAL_TIME)
		thermalTimeConst_ = MAX_THERMAL_TIME;
	else
		thermalTimeConst_ = abs(thermalTimeConst);
}

//=============================================================================

ushort MotorParEPOS::GetEncoderResolution(void) const 
{ 
	return encResolution_; 
}

//=============================================================================

void MotorParEPOS::SetEncoderResolution(ushort encResolution) 
{
	if (abs(encResolution) > MAX_ENC_PULSE_NO)
		encResolution_ = MAX_ENC_PULSE_NO;
	else
		encResolution_ = abs(encResolution);
}

//=============================================================================

PosSensorType MotorParEPOS::GetSensorType(void) const 
{ 
	return sensorType_; 
}

//=============================================================================

void MotorParEPOS::SetSensorType(PosSensorType sensorType) 
{
	sensorType_ = sensorType; 
}

//=============================================================================

MotorType MotorParEPOS::GetMotorType(void) const 
{ 
	return motorType_; 
}

//=============================================================================

void MotorParEPOS::SetMotorType(MotorType motorType) 
{
	motorType_ = motorType; 
}

//=============================================================================
// Object:						MotionParEPOS
//=============================================================================

MotionParEPOS::MotionParEPOS(OperationModes motorOperationMode, ushort maxProfileVelocity, ulong profileAcceleration,
				     ulong profileDeceleration, ulong quickStopDeceleration, ushort profileVelocity, 
				     MotionProfileTypes motionProfileType, ulong maxFollowingErr, ulong positionWindowSize,
					 ushort positionWindowTime, long minPositionLimit, long maxPositionLimit):

										motorOperationMode_(motorOperationMode),
										maxFollowingErr_(maxFollowingErr),
										positionWindowSize_(positionWindowSize),
										positionWindowTime_(positionWindowTime),
										minPositionLimit_(minPositionLimit),
										maxPositionLimit_(maxPositionLimit),
										maxProfileVelocity_(maxProfileVelocity),
										profileAcceleration_(profileAcceleration),
										profileDeceleration_(profileDeceleration),
										quickStopDeceleration_(quickStopDeceleration),
										profileVelocity_(profileVelocity),
										motionProfileType_(motionProfileType)
			{
				if (abs(maxProfileVelocity) > MAX_PROFILE_VELOCITY)
					maxProfileVelocity_ = MAX_PROFILE_VELOCITY;
				else
					maxProfileVelocity_ = abs(maxProfileVelocity);

				if (abs(profileVelocity) > MAX_PROFILE_VELOCITY)
					profileVelocity_ = MAX_PROFILE_VELOCITY;
				else
					profileVelocity_ = abs(profileVelocity);
			}

//=============================================================================

ulong MotionParEPOS::GetMaxFollowingErr(void) const
{
	return maxFollowingErr_;
}

//=============================================================================

void MotionParEPOS::SetMaxFollowingErr(ulong maxFollowingErr)
{
	maxFollowingErr_ = maxFollowingErr;
}

//=============================================================================

ulong MotionParEPOS::GetPositionWindowSize(void) const
{
	return positionWindowSize_;
}

//=============================================================================

void MotionParEPOS::SetPositionWindowSize(ulong positionWindowSize)
{
	positionWindowSize_ = positionWindowSize;
}

//=============================================================================

ushort MotionParEPOS::GetPositionWindowTime(void) const
{
	return positionWindowTime_;
}

//=============================================================================

void MotionParEPOS::SetPositionWindowTime(ushort positionWindowTime)
{
	positionWindowTime_ = positionWindowTime;
}

//=============================================================================

long MotionParEPOS::GetMinimalPositionLimit(void) const
{
	return minPositionLimit_;
}

//=============================================================================

void MotionParEPOS::SetMinimalPositionLimit(long minPositionLimit)
{
	minPositionLimit_ = minPositionLimit;
}

//=============================================================================

long MotionParEPOS::GetMaximalPositionLimit(void) const
{
	return maxPositionLimit_;
}

//=============================================================================

void MotionParEPOS::SetMaximalPositionLimit(long maxPositionLimit)
{
	maxPositionLimit_ = maxPositionLimit;
}

//=============================================================================

ushort MotionParEPOS::GetMaximalProfileVelocity(void) const
{
	return maxProfileVelocity_;
}

//=============================================================================

void MotionParEPOS::SetMaximalProfileVelocity(ushort maxProfileVelocity)
{
	if (abs(maxProfileVelocity) > MAX_PROFILE_VELOCITY)
		maxProfileVelocity_ = MAX_PROFILE_VELOCITY;
	else
		maxProfileVelocity_ = abs(maxProfileVelocity);
}

//=============================================================================

ulong MotionParEPOS::GetProfileAcceleration(void) const
{
	return profileAcceleration_;
}

//=============================================================================

void MotionParEPOS::SetProfileAcceleration(ulong profileAcceleration)
{
	profileAcceleration_ = profileAcceleration;
}

//=============================================================================

ulong MotionParEPOS::GetProfileDeceleration(void) const
{
	return profileDeceleration_;
}

//=============================================================================

void MotionParEPOS::SetProfileDeceleration(ulong profileDeceleration)
{
	profileDeceleration_ = profileDeceleration;
}

//=============================================================================

ulong MotionParEPOS::GetQuickStopDeceleration(void) const
{
	return quickStopDeceleration_;
}

//=============================================================================

void MotionParEPOS::SetQuickStopDeceleration(ulong quickStopDeceleration)
{
	quickStopDeceleration_ = quickStopDeceleration;
}

//=============================================================================

ushort MotionParEPOS::GetProfileVelocity(void) const
{
	return profileVelocity_;
}

//=============================================================================

void MotionParEPOS::SetProfileVelocity(ushort profileVelocity)
{
	if (abs(profileVelocity) > MAX_PROFILE_VELOCITY)
		profileVelocity_ = MAX_PROFILE_VELOCITY;
	else
		profileVelocity_ = abs(profileVelocity);
}

//=============================================================================

MotionProfileTypes MotionParEPOS::GetMotionProfileType(void) const
{
	return motionProfileType_;
}

//=============================================================================

void MotionParEPOS::SetMotionProfileType(MotionProfileTypes motionProfileType)
{
	motionProfileType_ = motionProfileType;
}

//=============================================================================

OperationModes MotionParEPOS::GetMotorOperationMode(void) const
{
	return motorOperationMode_;
}

//=============================================================================

void MotionParEPOS::SetMotorOperationMode(OperationModes motorOperationMode)
{
	motorOperationMode_ = motorOperationMode;
}

//=============================================================================
// Object:						  MotorEPOS
//=============================================================================

MotorEPOS::MotorEPOS( MotorParEPOS motpar, CommPtr pComm ) : pComm_( pComm ),
                                                motPar_( motpar )
{
	//! Clear data buffers for sending and receiving SDO frames
	memset(_dataSendSDO_, 0, 8);
	memset(_dataReceiveSDO_, 0, 8);

	//! Clear data buffers for sending and receiving frames in serial communication
	memset(_dataSendSerial_, 0, 9);
	memset(_dataReceiveSerial_, 0, 9);

	//! Clear all object variables
	motorConnected_ = false;

	actualCurrent_ = 0;
	actualPosition_ = 0;
	actualVelocity_ = 0;

	positionRef_ = 0;
	velocityRef_ = 0;

	anainput1_ = 0;
	anainput2_ = 0;

	//! Clear errors at motor object construction
	_ErrorCode_ = ERR_NOTHING;
	_status_.Message("Indeterminated status!");
	_status_.Id(MotorStatus::DISABLED);

	//! Move motor to relative position and immediately
	absoluteRun_ = false;
	immediatelyRun_ = false;

	//! Set all remaining variables for reading device type of EPOS
	_COB_ID_ = COB_ID_SEND + motPar_.GetMotorID();
	_Index_ = DEVICE_TYPE_INDX;
	_SubIndex_ = DEVICE_TYPE_SUB_INDX;
	_cmdSpec_ = DEVICE_TYPE_SIZE;
	_Data_ = 0;
	_TypeOfOperation_ = READ_FROM_EPOS;
}

//=============================================================================

MotorEPOS::~MotorEPOS()
{
	motorConnected_ = false;
}

//=============================================================================

/* virtual */
long MotorEPOS::Current(void) const
{
	return actualCurrent_;
}

//=============================================================================

/* virtual */
long MotorEPOS::Position(void) const
{
	return actualPosition_;
}

//=============================================================================

/* virtual */
long MotorEPOS::Velocity(void) const
{
	return actualVelocity_;
}

//=============================================================================

/* virtual */
void MotorEPOS::PositionRef(long Position)
{
	positionRef_ = Position;
}

//=============================================================================

/* virtual */
void MotorEPOS::VelocityRef(long Velocity)
{
	velocityRef_ = Velocity;
}

//=============================================================================

/* virtual */
long MotorEPOS::PositionRef(void) const
{
	return positionRef_;
}

//=============================================================================

/* virtual */
long MotorEPOS::VelocityRef(void) const
{
	return velocityRef_;
}

//=============================================================================

void MotorEPOS::SetAbsoluteMovement(bool Absolute)
{
	absoluteRun_ = Absolute;
}

//=============================================================================

void MotorEPOS::SetImmediateRef(bool Immediate)
{
	immediatelyRun_ = Immediate;
}

//=============================================================================

bool MotorEPOS::GetAbsoluteMovement(void) const
{
	return absoluteRun_;
}

//=============================================================================

bool MotorEPOS::GetImmediateRef(void) const
{
	return immediatelyRun_;
}

//=============================================================================

long MotorEPOS::AnalogInput1(void) const
{
	return anainput1_;
}

//=============================================================================
	
long MotorEPOS::AnalogInput2(void) const
{
	return anainput2_;
}

//=============================================================================

long MotorEPOS::GetCommErrorCode(void) const
{
	return _ErrorCode_;
}

//=============================================================================

std::string MotorEPOS::GetErrorDescription(void)
{
	switch (_ErrorCode_)
	{
		case ERR_NOTHING : 
			return "No error!";

		case ERR_OBJ_NOT_EXIST:
			return "Object does not exist in the object dictionary!";

		case ERR_SUB_INDX_NOT_EXIST:
			return "Sub-index does not exist!";

		case ERR_CLIENT_SERVER_COMMAND:
			return "Client/server command specifier not valid or unknown!";

		case ERR_TGL_BIT_NOT_ALTERED:
			return "Toggle bit not alternated!";

		case ERR_SDO_TIMED_OUT:
			return "SDO protocol timed out!";

		case ERR_OUT_OF_MEMORY:
			return "Out of memory!";

		case ERR_UNSUPPORTED_ACCESS:
			return "Unsupported access to an object!";

		case ERR_WRITE_ONLY:
			return "Attempt to read a write only object!";

		case ERR_READ_ONLY:
			return "Attempt to write a read only object!";

		case ERR_OBJ_CANNOT_MAP:
			return "Object cannot be mapped to the PDO!";

		case ERR_HUGE_LENGTH:
			return "The number and length of the objects to be mapped would exceed PDO length!";

		case ERR_INCOMPATIBILITY_PARAM:
			return "General parameter incompatibility reason!";

		case ERR_INCOMPATIBILITY_DEV:
			return "General internal incompatibility in the device!";

		case ERR_ACCES_FAILED:
			return "Access failed due to an hardware error!";

		case ERR_DATA_TYPE_MATCH:
			return "Data type does not match, length of service parameter does not match!";

		case ERR_DATA_TYPE_PARAM_HIGH:
			return "Data type does not match, length of service parameter too high!";

		case ERR_DATA_TYPE_PARAM_LOW:
			return "Data type does not match, length of service parameter too low!";

		case ERR_VALUE_RANGE_EXCEEDED:
			return "Value range of parameter exceeded (only for write access)!";

		case ERR_VALUE_TOO_HIGH:
			return "Value of parameter written too high!";

		case ERR_VALUE_TOO_LOW:
			return "Value of parameter written too low!";

		case ERR_MAX_MIN_VAL:
			return "Maximum value is less than minimum value!";

		case ERR_GENERAL:
			return "General error!";

		case ERR_DATA_APP:
			return "Data cannot be transferred or stored to the application!";

		case ERR_DATA_LOCAL_CONTROL:
			return "Data cannot be transferred or stored to the application because of local control!";

		case ERR_DATA_DEV_STATE:
			return "Data cannot be transferred or stored to the application because of the present device state!";

		case ERR_MAX_WRONG_NMT_STATE:
			return "The device is in wrong NMT state!";

		case ERR_MAX_ILLEGAL_COMMAND:
			return "The RS-232 command is illegal!";

		case ERR_MAX_INCORRECT_PASSWORD:
			return "The password is not correct!";

		case ERR_MAX_DEV_NOT_SERVICE_MODE:
			return "The device is not in service mode!";

		case ERR_MAX_NODE_ID:
			return "Error Node-ID!";

		default: 
			return "UNKNOWN ERROR";
	}

	return "";
}

//=============================================================================

void MotorEPOS::formatSDOSendFrame_(void)
{
	if (_TypeOfOperation_ == READ_FROM_EPOS)
	{
		switch(_cmdSpec_)
		{
		case 1:
			_dataSendSDO_[0] = READ_OBJECT_SEND_DATA_1_BYTE;
			break;
		case 2:
			_dataSendSDO_[0] = READ_OBJECT_SEND_DATA_2_BYTE;
			break;
		case 4:
			_dataSendSDO_[0] = READ_OBJECT_SEND_DATA_4_BYTE;
			break;
		}
		_dataSendSDO_[4] = 0x00;
		_dataSendSDO_[5] = 0x00;
		_dataSendSDO_[6] = 0x00;
		_dataSendSDO_[7] = 0x00;
	}
	else
	{
		switch(_cmdSpec_)
		{
		case 1:
			_dataSendSDO_[0] = WRITE_OBJECT_SEND_DATA_1_BYTE;
			_dataSendSDO_[4] = (char) _Data_;
			_dataSendSDO_[5] = 0x00;
			_dataSendSDO_[6] = 0x00;
			_dataSendSDO_[7] = 0x00;
			break;
		case 2:
			_dataSendSDO_[0] = WRITE_OBJECT_SEND_DATA_2_BYTE;
			_dataSendSDO_[4] = (char) (_Data_ & 0xFF);
			_dataSendSDO_[5] = (char) ((_Data_ & 0xFFFF) >> 8);
			_dataSendSDO_[6] = 0x00;
			_dataSendSDO_[7] = 0x00;
			break;
		case 4:
			_dataSendSDO_[0] = WRITE_OBJECT_SEND_DATA_4_BYTE;
			_dataSendSDO_[4] = (char) (_Data_ & 0xFF);
			_dataSendSDO_[5] = (char) ((_Data_ & 0xFFFF) >> 8);
			_dataSendSDO_[6] = (char) ((_Data_ & 0xFFFFFF) >> 16);
			_dataSendSDO_[7] = (char) ((_Data_ & 0xFFFFFFFF) >> 24);
			break;
		}
	}
	_dataSendSDO_[1] = (char) (_Index_ & 0x00FF);
	_dataSendSDO_[2] = (char) (_Index_ >> 8);
	_dataSendSDO_[3] = (char) _SubIndex_;

	//! Also set Frame COB-ID!
	_COB_ID_ = COB_ID_SEND + motPar_.GetMotorID();

	return;
}

//=============================================================================

void MotorEPOS::formatSerialCommSendFrame_(void)
{
	_dataSendSerial_[0] = (char) ((_Index_ & 0xFFFF) >> 8);
	_dataSendSerial_[1] = (char) (_Index_ & 0xFF);
	_dataSendSerial_[2] = (char) _SubIndex_;
	_dataSendSerial_[3] = (char) motPar_.GetMotorID();

	switch(_cmdSpec_)
	{
	case 1:
		_dataSendSerial_[7] = (char) _Data_;
		_dataSendSerial_[6] = 0x00;
		_dataSendSerial_[5] = 0x00;
		_dataSendSerial_[4] = 0x00;
		break;
	case 2:
		_dataSendSerial_[7] = (char) (_Data_ & 0xFF);
		_dataSendSerial_[6] = (char) ((_Data_ & 0xFFFF) >> 8);
		_dataSendSerial_[5] = 0x00;
		_dataSendSerial_[4] = 0x00;
		break;
	case 4:
		_dataSendSerial_[7] = (char) (_Data_ & 0xFF);
		_dataSendSerial_[6] = (char) ((_Data_ & 0xFFFF) >> 8);
		_dataSendSerial_[5] = (char) ((_Data_ & 0xFFFFFF) >> 16);
		_dataSendSerial_[4] = (char) ((_Data_ & 0xFFFFFFFF) >> 24);
		break;
	}	

	if (_TypeOfOperation_ == READ_FROM_EPOS)
		_dataSendSerial_[8] = READ_FROM_EPOS;
	else
		_dataSendSerial_[8] = WRITE_TO_EPOS;
	
	return;
}

//=============================================================================

bool MotorEPOS::checkReceivedSDOFrameFormat(void)
{
	if (COB_ID_RECEIVE + motPar_.GetMotorID() == _COB_ID_) 
		return true;	//! Received Frame is SDO
	else
		return false;	//! Received Frame is PDO
}

//=============================================================================

void MotorEPOS::formatReceivedDataSDO_(void)
{
    //! Set all parameters from received SDO Frame
	if (MotorEPOS::checkReceivedSDOFrameFormat())
	{
		if (_TypeOfOperation_ == READ_FROM_EPOS) //! Reading from EPOS
		{
			switch(_cmdSpec_ = (_dataReceiveSDO_[0] & 0xFF))
			{
			case READ_OBJECT_RECEIVE_DATA_1_BYTE:
				_Data_ = (_dataReceiveSDO_[4] & 0xFF);
				break;
			case READ_OBJECT_RECEIVE_DATA_2_BYTE:
				_Data_ = (_dataReceiveSDO_[4] & 0xFF) | ((_dataReceiveSDO_[5] & 0xFF) << 8);
				break;
			case READ_OBJECT_RECEIVE_DATA_4_BYTE:
				_Data_ = (_dataReceiveSDO_[4] & 0xFF) | ((_dataReceiveSDO_[5] & 0xFF) << 8) | 
					((_dataReceiveSDO_[6] & 0xFF) << 16) | ((_dataReceiveSDO_[7] & 0xFF) << 24);
				break;
			}
		}
		else		//! Writing to EPOS
		{
			switch(_cmdSpec_ = (_dataReceiveSDO_[0] & 0xFF))
			{
			case WRITE_OBJECT_RECEIVE_DATA_1_BYTE: // The same code for 2 or 4 bytes
				_Data_ = 0;
				break;
			case ERROR_COMMAND_SPEC:
				_Data_ = (_dataReceiveSDO_[4] & 0xFF) | ((_dataReceiveSDO_[5] & 0xFF) << 8) | 
					((_dataReceiveSDO_[6] & 0xFF) << 16) | ((_dataReceiveSDO_[7] & 0xFF) << 24);
				_ErrorCode_ = _Data_;
				break;
			}
		}
		_Index_ = (_dataReceiveSDO_[1] & 0xFF) | ((_dataReceiveSDO_[2] & 0xFF) << 8);
		_SubIndex_ = (_dataReceiveSDO_[3] & 0xFF);
	}
	//! Set all parameters from received PDO Frame
	else
	{
		
	}
}

//=============================================================================

void MotorEPOS::formatReceivedDataSerial_(void)
{
	
	_Data_ = (_dataReceiveSerial_[0] & 0xFF) | ((_dataReceiveSerial_[1] & 0xFF) << 8)
		     | ((_dataReceiveSerial_[2] & 0xFF) << 16) | ((_dataReceiveSerial_[3] & 0xFF) << 24);

	//! Escape changing (or clearing) error code with reading status of the controller
	if (_Index_ != STATUSWORD_INDX)
	{
		_ErrorCode_ = (_dataReceiveSerial_[4] & 0xFF) | ((_dataReceiveSerial_[5] & 0xFF) << 8)
		     | ((_dataReceiveSerial_[6] & 0xFF) << 16) | ((_dataReceiveSerial_[7] & 0xFF) << 24);
	}
	_Index_ = 0;
	_SubIndex_ = 0;

	return;
}

//=============================================================================

bool MotorEPOS::UpdateCmdToEPOS_(void)
{
	bool success = false;
	struct timeb tp;
	time_t t1=0, t2=0;
	CommMsg msg;

	ftime(&tp);
	t1 = tp.millitm + (tp.time & 0xFFFFF) * 1000;

	do
	{
		ftime(&tp);
		t2 = tp.millitm + (tp.time & 0xFFFFF) * 1000;
	}while (abs((long)(t1-t2)) < DELAY_TIME_MS);

	switch (pComm_->Params().Interface())
	{
		case CommPar::RS232:
			msg.Id(motPar_.GetMotorID());
			msg.Data(_dataSendSerial_, 9);
			msg.Timestamp(pt::microsec_clock::local_time());
			break;

		case CommPar::CAN:
			msg.Id(_COB_ID_);
			msg.Data(_dataSendSDO_, 8);
			msg.Timestamp(pt::microsec_clock::local_time());
			break;

	}

	success = pComm_->Send( msg );

	memset(_dataReceiveSDO_, 0, 8);
	memset(_dataReceiveSerial_, 0, 9);
	
	if ( success )
	{
		success = pComm_->Receive( msg, 1000 );
		
		switch (pComm_->Params().Interface())
		{
			case CommPar::CAN:
				_COB_ID_= msg.Id();
				memcpy( _dataReceiveSDO_, msg.Data(), msg.Size() );
				formatReceivedDataSDO_();
				break;

			case CommPar::RS232:
				memcpy( _dataReceiveSerial_, msg.Data(), msg.Size() );
				formatReceivedDataSerial_();
				break;
		}
	}
	else
	{
		success = false;
		switch (pComm_->Params().Interface())
		{
			case CommPar::CAN:
				_COB_ID_= msg.Id();
				memcpy( _dataReceiveSDO_, msg.Data(), msg.Size() );
				formatReceivedDataSDO_();
				break;
			case CommPar::RS232:
				pComm_->Receive( msg, 1000 );
				memcpy( _dataReceiveSerial_, msg.Data(), msg.Size() );
				formatReceivedDataSerial_();
				break;
		}
	}

	return success;
}

//****************************************************************************************************************//
//                    DECLARATIONS OF PRIVATE FUNCTIONS FOR ALL EPOS OBJECTS IN OBJECT DICTIONARY                 //
//****************************************************************************************************************//

bool MotorEPOS::setCANBitrateUpdate_(CANBitrate Bitrate)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = CAN_BITRATE_SIZE;
	_Index_ = CAN_BITRATE_INDX;
	_SubIndex_ = CAN_BITRATE_SUB_INDX;
	_Data_ = Bitrate;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getCanBitrateUpdate_(CANBitrate &Bitrate)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = CAN_BITRATE_SIZE;
	_Index_ = CAN_BITRATE_INDX;
	_SubIndex_ = CAN_BITRATE_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Bitrate = (CANBitrate) _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setRS232BaudrateUpdate_(RS232Baudrate Baudrate)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = RS232_BAUDRATE_SIZE;
	_Index_ = RS232_BAUDRATE_INDX;
	_SubIndex_ = RS232_BAUDRATE_SUB_INDX;
	_Data_ = Baudrate;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getRS232BaudrateUpdate_(RS232Baudrate &Baudrate)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = RS232_BAUDRATE_SIZE;
	_Index_ = RS232_BAUDRATE_INDX;
	_SubIndex_ = RS232_BAUDRATE_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Baudrate = (RS232Baudrate) _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

// ++++++++++++++++++++++++++++++++++ Functions for members in "MotorParEPOS" ++++++++++++++++++++++++++++++++++++++++//

bool MotorEPOS::setMotorTypeUpdate_(MotorType Type)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = MOTOR_TYPE_SIZE;
	_Index_ = MOTOR_TYPE_INDX;
	_SubIndex_ = MOTOR_TYPE_SUB_INDX;
	_Data_ = Type;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getMotorTypeUpdate_(MotorType &Type)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = MOTOR_TYPE_SIZE;
	_Index_ = MOTOR_TYPE_INDX;
	_SubIndex_ = MOTOR_TYPE_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Type = (MotorType) _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setCurrentLimitUpdate_(long Current)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = CURRENT_LIMIT_SIZE;
	_Index_ = CURRENT_LIMIT_INDX;
	_SubIndex_ = CURRENT_LIMIT_SUB_INDX;
	_Data_ = Current;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getCurrentLimitUpdate_(long &Current)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = CURRENT_LIMIT_SIZE;
	_Index_ = CURRENT_LIMIT_INDX;
	_SubIndex_ = CURRENT_LIMIT_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Current = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setPolePairsUpdate_(long PolePairs)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = POLE_PAIRS_SIZE;
	_Index_ = POLE_PAIRS_INDX;
	_SubIndex_ = POLE_PAIRS_SUB_INDX;
	_Data_ = PolePairs;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getPolePairsUpdate_(long &PolePairs)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = POLE_PAIRS_SIZE;
	_Index_ = POLE_PAIRS_INDX;
	_SubIndex_ = POLE_PAIRS_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	PolePairs = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setThermalTimeConstUpdate_(long ThermalTime)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = THERMAL_TIME_CONST_SIZE;
	_Index_ = THERMAL_TIME_CONST_INDX;
	_SubIndex_ = THERMAL_TIME_CONST_SUB_INDX;
	_Data_ = ThermalTime;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getThermalTimeConstUpdate_(long &ThermalTime)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = THERMAL_TIME_CONST_SIZE;
	_Index_ = THERMAL_TIME_CONST_INDX;
	_SubIndex_ = THERMAL_TIME_CONST_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	ThermalTime = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setMaximalSpeedUpdate_(long MaxSpeed)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = MAX_SPEED_SIZE;
	_Index_ = MAX_SPEED_INDX;
	_SubIndex_ = MAX_SPEED_SUB_INDX;
	_Data_ = MaxSpeed;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getMaximalSpeedUpdate_(long &MaxSpeed)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = MAX_SPEED_SIZE;
	_Index_ = MAX_SPEED_INDX;
	_SubIndex_ = MAX_SPEED_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	MaxSpeed = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setEncoderResolutionUpdate_(long Resolution)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = ENCODER_PULSE_NUMBER_SIZE;
	_Index_ = ENCODER_PULSE_NUMBER_INDX;
	_SubIndex_ = ENCODER_PULSE_NUMBER_SUB_INDX;
	_Data_ = Resolution;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getEncoderResolutionUpdate_(long &Resolution)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = ENCODER_PULSE_NUMBER_SIZE;
	_Index_ = ENCODER_PULSE_NUMBER_INDX;
	_SubIndex_ = ENCODER_PULSE_NUMBER_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Resolution = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setPositionSensorTypeUpdate_(PosSensorType Type)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = POSITION_SENSOR_TYPE_SIZE;
	_Index_ = POSITION_SENSOR_TYPE_INDX;
	_SubIndex_ = POSITION_SENSOR_TYPE_SUB_INDX;
	_Data_ = Type;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getPositionSensorTypeUpdate_(PosSensorType &Type)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = POSITION_SENSOR_TYPE_SIZE;
	_Index_ = POSITION_SENSOR_TYPE_INDX;
	_SubIndex_ = POSITION_SENSOR_TYPE_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Type = (PosSensorType) _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

// ++++++++++++++++++++++++++++++++++ Functions for members in "RegParEPOS" ++++++++++++++++++++++++++++++++++++++++//

bool MotorEPOS::setCurrentRegPGainUpdate_(long Gain)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = CURRENT_REG_P_GAIN_SIZE;
	_Index_ = CURRENT_REG_P_GAIN_INDX;
	_SubIndex_ = CURRENT_REG_P_GAIN_SUB_INDX;
	_Data_ = Gain;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getCurrentRegPGainUpdate_(long &Gain)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = CURRENT_REG_P_GAIN_SIZE;
	_Index_ = CURRENT_REG_P_GAIN_INDX;
	_SubIndex_ = CURRENT_REG_P_GAIN_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Gain = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setCurrentRegIGainUpdate_(long Gain)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = CURRENT_REG_I_GAIN_SIZE;
	_Index_ = CURRENT_REG_I_GAIN_INDX;
	_SubIndex_ = CURRENT_REG_I_GAIN_SUB_INDX;
	_Data_ = Gain;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getCurrentRegIGainUpdate_(long &Gain)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = CURRENT_REG_I_GAIN_SIZE;
	_Index_ = CURRENT_REG_I_GAIN_INDX;
	_SubIndex_ = CURRENT_REG_I_GAIN_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Gain = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setVelocityRegPGainUpdate_(long Gain)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = VELOCITY_REG_P_GAIN_SIZE;
	_Index_ = VELOCITY_REG_P_GAIN_INDX;
	_SubIndex_ = VELOCITY_REG_P_GAIN_SUB_INDX;
	_Data_ = Gain;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getVelocityRegPGainUpdate_(long &Gain)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = VELOCITY_REG_P_GAIN_SIZE;
	_Index_ = VELOCITY_REG_P_GAIN_INDX;
	_SubIndex_ = VELOCITY_REG_P_GAIN_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Gain = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setVelocityRegIGainUpdate_(long Gain)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = VELOCITY_REG_I_GAIN_SIZE;
	_Index_ = VELOCITY_REG_I_GAIN_INDX;
	_SubIndex_ = VELOCITY_REG_I_GAIN_SUB_INDX;
	_Data_ = Gain;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getVelocityRegIGainUpdate_(long &Gain)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = VELOCITY_REG_I_GAIN_SIZE;
	_Index_ = VELOCITY_REG_I_GAIN_INDX;
	_SubIndex_ = VELOCITY_REG_I_GAIN_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Gain = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setPositionRegPGainUpdate_(long Gain)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = POSITION_REG_P_GAIN_SIZE;
	_Index_ = POSITION_REG_P_GAIN_INDX;
	_SubIndex_ = POSITION_REG_P_GAIN_SUB_INDX;
	_Data_ = Gain;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getPositionRegPGainUpdate_(long &Gain)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = POSITION_REG_P_GAIN_SIZE;
	_Index_ = POSITION_REG_P_GAIN_INDX;
	_SubIndex_ = POSITION_REG_P_GAIN_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Gain = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setPositionRegIGainUpdate_(long Gain)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = POSITION_REG_I_GAIN_SIZE;
	_Index_ = POSITION_REG_I_GAIN_INDX;
	_SubIndex_ = POSITION_REG_I_GAIN_SUB_INDX;
	_Data_ = Gain;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getPositionRegIGainUpdate_(long &Gain)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = POSITION_REG_I_GAIN_SIZE;
	_Index_ = POSITION_REG_I_GAIN_INDX;
	_SubIndex_ = POSITION_REG_I_GAIN_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Gain = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setPositionRegDGainUpdate_(long Gain)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = POSITION_REG_D_GAIN_SIZE;
	_Index_ = POSITION_REG_D_GAIN_INDX;
	_SubIndex_ = POSITION_REG_D_GAIN_SUB_INDX;
	_Data_ = Gain;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getPositionRegDGainUpdate_(long &Gain)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = POSITION_REG_D_GAIN_SIZE;
	_Index_ = POSITION_REG_D_GAIN_INDX;
	_SubIndex_ = POSITION_REG_D_GAIN_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Gain = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================
// ++++++++++++++++++++++++++++++++++ Functions for members in "MotionParEPOS" +++++++++++++++++++++++++++++++++++++++//

bool MotorEPOS::setOperationModeUpdate_(OperationModes OpMode)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = OPERATION_MODE_SIZE;
	_Index_ = OPERATION_MODE_INDX;
	_SubIndex_ = OPERATION_MODE_SUB_INDX;
	_Data_ = OpMode;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getOperationModeUpdate_(OperationModes &OpMode)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = OPERATION_MODE_SIZE;
	_Index_ = OPERATION_MODE_INDX;
	_SubIndex_ = OPERATION_MODE_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	OpMode = (OperationModes) _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setMaxProfileVelocityUpdate_(long MaxVelocity)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = MAX_PROFILE_VELOCITY_SIZE;
	_Index_ = MAX_PROFILE_VELOCITY_INDX;
	_SubIndex_ = MAX_PROFILE_VELOCITY_SUB_INDX;
	_Data_ = MaxVelocity;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getMaxProfileVelocityUpdate_(long &MaxVelocity)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = MAX_PROFILE_VELOCITY_SIZE;
	_Index_ = MAX_PROFILE_VELOCITY_INDX;
	_SubIndex_ = MAX_PROFILE_VELOCITY_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	MaxVelocity = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setProfileAccelerationUpdate_(long Acceleration)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = PROFILE_ACCELERATION_SIZE;
	_Index_ = PROFILE_ACCELERATION_INDX;
	_SubIndex_ = PROFILE_ACCELERATION_SUB_INDX;
	_Data_ = Acceleration;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getProfileAccelerationUpdate_(long &Acceleration)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = PROFILE_ACCELERATION_SIZE;
	_Index_ = PROFILE_ACCELERATION_INDX;
	_SubIndex_ = PROFILE_ACCELERATION_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Acceleration = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setProfileDecelerationUpdate_(long Deceleration)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = PROFILE_DECELERATION_SIZE;
	_Index_ = PROFILE_DECELERATION_INDX;
	_SubIndex_ = PROFILE_DECELERATION_SUB_INDX;
	_Data_ = Deceleration;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getProfileDecelerationUpdate_(long &Deceleration)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = PROFILE_DECELERATION_SIZE;
	_Index_ = PROFILE_DECELERATION_INDX;
	_SubIndex_ = PROFILE_DECELERATION_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Deceleration = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setQuickstopDecelerationUpdate_(long QuickDeceleration)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = QUICK_STOP_DECELERATION_SIZE;
	_Index_ = QUICK_STOP_DECELERATION_INDX;
	_SubIndex_ = QUICK_STOP_DECELERATION_SUB_INDX;
	_Data_ = QuickDeceleration;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getQuickstopDecelerationUpdate_(long &QuickDeceleration)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = QUICK_STOP_DECELERATION_SIZE;
	_Index_ = QUICK_STOP_DECELERATION_INDX;
	_SubIndex_ = QUICK_STOP_DECELERATION_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	QuickDeceleration = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setProfileVelocityUpdate_(long Velocity)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = PROFILE_VELOCITY_SIZE;
	_Index_ = PROFILE_VELOCITY_INDX;
	_SubIndex_ = PROFILE_VELOCITY_SUB_INDX;
	_Data_ = Velocity;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getProfileVelocityUpdate_(long &Velocity)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = PROFILE_VELOCITY_SIZE;
	_Index_ = PROFILE_VELOCITY_INDX;
	_SubIndex_ = PROFILE_VELOCITY_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Velocity = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setMotionProfileTypeUpdate_(MotionProfileTypes MotionType)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = MOTION_PROFILE_TYPE_SIZE;
	_Index_ = MOTION_PROFILE_TYPE_INDX;
	_SubIndex_ = MOTION_PROFILE_TYPE_SUB_INDX;
	_Data_ = MotionType;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getMotionProfileTypeUpdate_(MotionProfileTypes &MotionType)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = MOTION_PROFILE_TYPE_SIZE;
	_Index_ = MOTION_PROFILE_TYPE_INDX;
	_SubIndex_ = MOTION_PROFILE_TYPE_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	MotionType = (MotionProfileTypes) _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setMaxFollowingErrorUpdate_(long MaxError)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = MAX_FOLLOWING_ERROR_SIZE;
	_Index_ = MAX_FOLLOWING_ERROR_INDX;
	_SubIndex_ = MAX_FOLLOWING_ERROR_SUB_INDX;
	_Data_ = MaxError;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getMaxFollowingErrorUpdate_(long &MaxError)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = MAX_FOLLOWING_ERROR_SIZE;
	_Index_ = MAX_FOLLOWING_ERROR_INDX;
	_SubIndex_ = MAX_FOLLOWING_ERROR_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	MaxError = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setPositionWindowSizeUpdate_(long Size)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = POSITION_WINDOW_SIZE;
	_Index_ = POSITION_WINDOW_INDX;
	_SubIndex_ = POSITION_WINDOW_SUB_INDX;
	_Data_ = Size;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getPositionWindowSizeUpdate_(long &Size)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = POSITION_WINDOW_SIZE;
	_Index_ = POSITION_WINDOW_INDX;
	_SubIndex_ = POSITION_WINDOW_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Size = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setPositionWindowTimeUpdate_(long Time)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = POSITION_WINDOW_TIME_SIZE;
	_Index_ = POSITION_WINDOW_TIME_INDX;
	_SubIndex_ = POSITION_WINDOW_TIME_SUB_INDX;
	_Data_ = Time;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getPositionWindowTimeUpdate_(long &Time)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = POSITION_WINDOW_TIME_SIZE;
	_Index_ = POSITION_WINDOW_TIME_INDX;
	_SubIndex_ = POSITION_WINDOW_TIME_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Time = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setMinPositionLimitUpdate_(long PositionLimit)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = MIN_POSITION_LIMIT_SIZE;
	_Index_ = MIN_POSITION_LIMIT_INDX;
	_SubIndex_ = MIN_POSITION_LIMIT_SUB_INDX;
	_Data_ = PositionLimit;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getMinPositionLimitUpdate_(long &PositionLimit)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = MIN_POSITION_LIMIT_SIZE;
	_Index_ = MIN_POSITION_LIMIT_INDX;
	_SubIndex_ = MIN_POSITION_LIMIT_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	PositionLimit = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setMaxPositionLimitUpdate_(long PositionLimit)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = MAX_POSITION_LIMIT_SIZE;
	_Index_ = MAX_POSITION_LIMIT_INDX;
	_SubIndex_ = MAX_POSITION_LIMIT_SUB_INDX;
	_Data_ = PositionLimit;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getMaxPositionLimitUpdate_(long &PositionLimit)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = MAX_POSITION_LIMIT_SIZE;
	_Index_ = MAX_POSITION_LIMIT_INDX;
	_SubIndex_ = MAX_POSITION_LIMIT_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	PositionLimit = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================
// ++++++++++++++++++++++++++++++++++ Functions for reading current, velocity and position +++++++++++++++++++++++//

bool MotorEPOS::getActualCurrentUpdate_(long &Current)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = READ_ACTUAL_CURRENT_SIZE;
	_Index_ = READ_ACTUAL_CURRENT_INDX;
	_SubIndex_ = READ_ACTUAL_CURRENT_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Current = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getActualPositionUpdate_(long &Position)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = READ_ACTUAL_POSITION_SIZE;
	_Index_ = READ_ACTUAL_POSITION_INDX;
	_SubIndex_ = READ_ACTUAL_POSITION_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Position = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getActualVelocityUpdate_(long &Velocity)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = READ_ACTUAL_VELOCITY_SIZE;
	_Index_ = READ_ACTUAL_VELOCITY_INDX;
	_SubIndex_ = READ_ACTUAL_VELOCITY_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Velocity = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================
// ++++++++++++++++++++++++++++++++++ Setting reference for motor velocity and position +++++++++++++++++++++++//

bool MotorEPOS::setPositionRefUpdate_(long Position)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = SET_TARGET_POSITION_SIZE;
	_Index_ = SET_TARGET_POSITION_INDX;
	_SubIndex_ = SET_TARGET_POSITION_SUB_INDX;
	_Data_ = (long) Position;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::setVelocityRefUpdate_(long Velocity)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = SET_TARGET_VELOCITY_SIZE;
	_Index_ = SET_TARGET_VELOCITY_INDX;
	_SubIndex_ = SET_TARGET_VELOCITY_SUB_INDX;
	_Data_ = (long) Velocity;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================
// ++++++++++++++++++++++++++++++++++ Store or load parameters from device volatile memory +++++++++++++++++++++++//

bool MotorEPOS::saveParametersUpdate_(void)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = STORE_PARAMETERS_SIZE;
	_Index_ = STORE_PARAMETERS_INDX;
	_SubIndex_ = STORE_PARAMETERS_SUB_INDX;
	_Data_ = 0x65766173;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::loadParametersUpdate_(void)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = RESTORE_PARAMETERS_SIZE;
	_Index_ = RESTORE_PARAMETERS_INDX;
	_SubIndex_ = RESTORE_PARAMETERS_SUB_INDX;
	_Data_ = 0x64616F6C;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

// ++++++++++++++++++++++++++++++++++ Functions for reading EPOS analog inputs +++++++++++++++++++++++++++++++//

bool MotorEPOS::getAnalogInput1Update_(long &Input1)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = ANALOG_INPUT1_SIZE;
	_Index_ = ANALOG_INPUT1_INDX;
	_SubIndex_ = ANALOG_INPUT1_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Input1 = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::getAnalogInput2Update_(long &Input2)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = ANALOG_INPUT2_SIZE;
	_Index_ = ANALOG_INPUT2_INDX;
	_SubIndex_ = ANALOG_INPUT2_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Input2 = _Data_;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}


// ++++++++++++++++++++++++++++++++++ Functions for changing EPOS operating states +++++++++++++++++++++++++++++++//

bool MotorEPOS::shutDownUpdate_(void)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = CONTROLWORD_SIZE;
	_Index_ = CONTROLWORD_INDX;
	_SubIndex_ = CONTROLWORD_SUB_INDX;
	_Data_ = SHUT_DOWN;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::switchOnUpdate_(void)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = CONTROLWORD_SIZE;
	_Index_ = CONTROLWORD_INDX;
	_SubIndex_ = CONTROLWORD_SUB_INDX;
	_Data_ = SWITCH_ON;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::disableVoltageUpdate_(void)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = CONTROLWORD_SIZE;
	_Index_ = CONTROLWORD_INDX;
	_SubIndex_ = CONTROLWORD_SUB_INDX;
	_Data_ = DISABLE_VOLTAGE;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

 //=============================================================================

bool MotorEPOS::haltUpdate_(void)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = CONTROLWORD_SIZE;
	_Index_ = CONTROLWORD_INDX;
	_SubIndex_ = CONTROLWORD_SUB_INDX;
	_Data_ = HALT_ON;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::quickStopUpdate_(void)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = CONTROLWORD_SIZE;
	_Index_ = CONTROLWORD_INDX;
	_SubIndex_ = CONTROLWORD_SUB_INDX;
	_Data_ = QUICK_STOP;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

bool MotorEPOS::faultResetUpdate_(void)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = CONTROLWORD_SIZE;
	_Index_ = CONTROLWORD_INDX;
	_SubIndex_ = CONTROLWORD_SUB_INDX;
	_Data_ = FAULT_RESET;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (bResult) _ErrorCode_ = ERR_NOTHING;
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

// ++++++++++++++++++++++++++++++ Function for changing CONTROLWORD SDO and run motor ++++++++++++++++++++++++++++//

bool MotorEPOS::runMotorUpdate_(int WriteData)
{
	bool bResult = false;

	_TypeOfOperation_ = WRITE_TO_EPOS;
	_cmdSpec_ = CONTROLWORD_SIZE;
	_Index_ = CONTROLWORD_INDX;
	_SubIndex_ = CONTROLWORD_SUB_INDX;
	_Data_ = WriteData;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	if (_ErrorCode_ != ERR_NOTHING) GET_EPOS_STATE();

	return bResult;
}

//=============================================================================

// +++++++++++++++++++++++ Function for reading STATUSWORD SDO to determine target reached +++++++++++++++++++++++//

bool MotorEPOS::readSTATUSWORDUpdate_(int &Statusword)
{
	bool bResult = false;

	_TypeOfOperation_ = READ_FROM_EPOS;
	_cmdSpec_ = STATUSWORD_SIZE;
	_Index_ = STATUSWORD_INDX;
	_SubIndex_ = STATUSWORD_SUB_INDX;
	_Data_ = 0;
	formatSDOSendFrame_();
	formatSerialCommSendFrame_();
	bResult = UpdateCmdToEPOS_();
	Statusword = _Data_;

	return bResult;
}

//================================================================================================================//
//                   END OF DECLARATIONS OF PRIVATE FUNCTIONS FOR ALL EPOS OBJECTS IN OBJECT DICTIONARY           //
//================================================================================================================//

bool MotorEPOS::UpdateRegParRead(void)
{
	long Gain;

	if (!(getCurrentRegPGainUpdate_( Gain ))) return false;
	motPar_.RegParams().SetCurrRegGainP( (ushort) Gain);

	if (!(getCurrentRegIGainUpdate_( Gain ))) return false;
	motPar_.RegParams().SetCurrRegGainI( (ushort) Gain);
	
	if (!(getVelocityRegPGainUpdate_( Gain ))) return false;
	motPar_.RegParams().SetVelRegGainP( (ushort) Gain);
	
	if (!(getVelocityRegIGainUpdate_( Gain ))) return false;
	motPar_.RegParams().SetVelRegGainI( (ushort) Gain);
	
	if (!(getPositionRegPGainUpdate_( Gain ))) return false;
	motPar_.RegParams().SetPosRegGainP( (ushort) Gain);
	
	if (!(getPositionRegIGainUpdate_( Gain ))) return false;
	motPar_.RegParams().SetPosRegGainI( (ushort) Gain);
	
	if (!(getPositionRegDGainUpdate_( Gain ))) return false;
	motPar_.RegParams().SetPosRegGainD( (ushort) Gain);

	return true;
}

//=============================================================================

bool MotorEPOS::UpdateRegParWrite( void )
{

	if (!(setCurrentRegPGainUpdate_(motPar_.RegParams().GetCurrRegGainP() ))) return false;
	if (!(setCurrentRegIGainUpdate_(motPar_.RegParams().GetCurrRegGainI() ))) return false;
	if (!(setVelocityRegPGainUpdate_(motPar_.RegParams().GetVelRegGainP() ))) return false;
	if (!(setVelocityRegIGainUpdate_(motPar_.RegParams().GetVelRegGainI() ))) return false;
	if (!(setPositionRegPGainUpdate_(motPar_.RegParams().GetPosRegGainP() ))) return false;
	if (!(setPositionRegIGainUpdate_(motPar_.RegParams().GetPosRegGainI() ))) return false;
	if (!(setPositionRegDGainUpdate_(motPar_.RegParams().GetPosRegGainD() ))) return false;

	return true;
}

//=============================================================================

bool MotorEPOS::UpdateMotionParRead( void )
{
	OperationModes Mode;
	MotionProfileTypes Type;
	long Param;

	if (!(getOperationModeUpdate_(  Mode ))) return false;
	motPar_.MotionParams().SetMotorOperationMode(Mode);
	
	if (!(getMaxProfileVelocityUpdate_(  Param ))) return false;
	motPar_.MotionParams().SetMaximalProfileVelocity( (ushort) Param);
	
	if (!(getProfileAccelerationUpdate_(  Param ))) return false;
	motPar_.MotionParams().SetProfileAcceleration( (ulong) Param);
	
	if (!(getProfileDecelerationUpdate_(  Param ))) return false;
	motPar_.MotionParams().SetProfileDeceleration( (ulong) Param);

	if (!(getQuickstopDecelerationUpdate_(  Param ))) return false;
	motPar_.MotionParams().SetQuickStopDeceleration( (ulong) Param);

	if (!(getProfileVelocityUpdate_(  Param ))) return false;
	motPar_.MotionParams().SetProfileVelocity( (ushort) Param);

	if (!(getMotionProfileTypeUpdate_(  Type ))) return false;
	motPar_.MotionParams().SetMotionProfileType(Type);

	if (!(getMaxFollowingErrorUpdate_(  Param ))) return false;
	motPar_.MotionParams().SetMaxFollowingErr( (ulong) Param);

	if (!(getPositionWindowSizeUpdate_(  Param ))) return false;
	motPar_.MotionParams().SetPositionWindowSize( (ulong) Param);

	if (!(getPositionWindowTimeUpdate_(  Param ))) return false;
	motPar_.MotionParams().SetPositionWindowTime( (ushort) Param);

	if (!(getMinPositionLimitUpdate_(  Param ))) return false;
	motPar_.MotionParams().SetMinimalPositionLimit( (long) Param);

	if (!(getMaxPositionLimitUpdate_(  Param ))) return false;
	motPar_.MotionParams().SetMaximalPositionLimit( (long) Param);

	return true;
}

//=============================================================================

bool MotorEPOS::UpdateMotionParWrite( void )
{

	if (!(setOperationModeUpdate_(motPar_.MotionParams().GetMotorOperationMode() ))) return false;
	if (!(setMaxProfileVelocityUpdate_(motPar_.MotionParams().GetMaximalProfileVelocity() ))) return false;
	if (!(setProfileAccelerationUpdate_(motPar_.MotionParams().GetProfileAcceleration() ))) return false;
	if (!(setProfileDecelerationUpdate_(motPar_.MotionParams().GetProfileDeceleration() ))) return false;
	if (!(setQuickstopDecelerationUpdate_(motPar_.MotionParams().GetQuickStopDeceleration() ))) return false;
	if (!(setProfileVelocityUpdate_(motPar_.MotionParams().GetProfileVelocity() ))) return false;
	if (!(setMotionProfileTypeUpdate_(motPar_.MotionParams().GetMotionProfileType() ))) return false;
	if (!(setMaxFollowingErrorUpdate_(motPar_.MotionParams().GetMaxFollowingErr() ))) return false;
	if (!(setPositionWindowSizeUpdate_(motPar_.MotionParams().GetPositionWindowSize() ))) return false;
	if (!(setPositionWindowTimeUpdate_(motPar_.MotionParams().GetPositionWindowTime() ))) return false;
	if (!(setMinPositionLimitUpdate_(motPar_.MotionParams().GetMinimalPositionLimit() ))) return false;
	if (!(setMaxPositionLimitUpdate_(motPar_.MotionParams().GetMaximalPositionLimit() ))) return false;

	return true;
}

//=============================================================================

bool MotorEPOS::UpdateMotorParRead( void )
{
	MotorType Type;
	PosSensorType sensorType;
	long Param;

	if (!(getMotorTypeUpdate_(  Type ))) return false;
	motPar_.SetMotorType(Type);

	if (!(getCurrentLimitUpdate_(  Param ))) return false;
	motPar_.SetMaximalCurrent( (ulong) Param);

	if (!(getPolePairsUpdate_(  Param ))) return false;
	motPar_.SetNumberPolePairs( (ushort) Param);

	if (!(getThermalTimeConstUpdate_(  Param ))) return false;
	motPar_.SetThermalTimeConst( (ushort) Param);

	if (!(getMaximalSpeedUpdate_(  Param ))) return false;
	motPar_.SetMaximalSpeed( (ushort) Param);

	if (!(getEncoderResolutionUpdate_(  Param ))) return false;
	motPar_.SetEncoderResolution( (ushort) Param);

	if (!(getPositionSensorTypeUpdate_(  sensorType ))) return false;
	motPar_.SetSensorType(sensorType);

	return true;
}

//=============================================================================

bool MotorEPOS::UpdateMotorParWrite( void )
{

	//! Disable voltage on EPOS to change motor parameters
	if (!disableVoltageUpdate_()) return false;

	if (!(setMotorTypeUpdate_(motPar_.GetMotorType()))) return false;
	if (!(setCurrentLimitUpdate_(motPar_.GetMaximalCurrent()))) return false;
	if (!(setPolePairsUpdate_(motPar_.GetNumberPolePairs()))) return false;
	if (!(setThermalTimeConstUpdate_(motPar_.GetThermalTimeConst()))) return false;
	if (!(setMaximalSpeedUpdate_(motPar_.GetMaximalSpeed()))) return false;
	if (!(setEncoderResolutionUpdate_(motPar_.GetEncoderResolution()))) return false;
	if (!(setPositionSensorTypeUpdate_(motPar_.GetSensorType()))) return false;

	return true;
}

//=============================================================================

bool MotorEPOS::UpdateAnalogInputs( void )
{
	long Param;
	
	if (!motorConnected_) return false;

	if (!(getAnalogInput1Update_( Param ))) return false;
	anainput1_ = Param;

	if (!(getAnalogInput2Update_( Param ))) return false;
	anainput2_ = Param;

	return true;
}

//=============================================================================

/* virtual */
bool MotorEPOS::Connect( void )
{
	motorConnected_ = false;

	if (pComm_->Open())
	{
		//! Reset any possible errors on startup
		faultResetUpdate_();
		
		//! Shut down EPOS
		if (!shutDownUpdate_()) return false;

		//! Disable voltage on EPOS to change motor parameters
		if (!disableVoltageUpdate_()) return false;

		//! Write all parameters to EPOS device
		if (!UpdateMotorParWrite()) return false; 
		if (!UpdateRegParWrite()) return false;
		if (!UpdateMotionParWrite()) return false;

		motorConnected_ = true;
	}
	
	return true;
}

//=============================================================================

/* virtual */
bool MotorEPOS::Disconnect( void )
{
	motorConnected_ = false;

	//! Shut down EPOS
	if (!shutDownUpdate_()) return false;

	//! Disable voltage on EPOS and leave device in disable mode
	if (!disableVoltageUpdate_()) return false;

	return pComm_->Close();
}

//=============================================================================

/* virtual */
bool MotorEPOS::UpdateRead( void )
{
	if (!motorConnected_) return false;

	//! Reset any possible errors
	faultResetUpdate_();

	//! Read motor current, velocity and position
	if (!(UpdateReadMeasuredData())) return false;

	return true;
}

//=============================================================================

/* virtual */
bool MotorEPOS::UpdateWrite( void )
{
	if (!motorConnected_) return false;

	//! Reset any possible errors
	faultResetUpdate_();

	Status();
	if (_status_.Id() != MotorStatus::ENABLED)
	{
		//! Shut down EPOS
		if (!shutDownUpdate_()) return false;
	}

	//! Switch on EPOS
	if (!switchOnUpdate_()) return false;

	//! Set new references for velocity and position
	if (!(UpdateSetReferences())) return false;

	//! Run motor movement
	if (!RUN()) return false;

	return true;
}

//=============================================================================

/* virtual */
bool MotorEPOS::RUN(void)
{
	bool success = false;

	if (!motorConnected_) return false;

	//! Choose approirate setting and call "runMotorUpdate_" to start motor rotating

	switch (absoluteRun_)
	{
	case true:
		if (immediatelyRun_) 
		{
			success = runMotorUpdate_(ABSOLUTE_IMM_MOVE);
			break;
		}
		else
		{
			success = runMotorUpdate_(ABSOLUTE_MOVE);
			break;
		}
	case false:
		if (immediatelyRun_) 
		{
			success = runMotorUpdate_(RELATIVE_IMM_MOVE);
			break;
		}
		else
		{
			success = runMotorUpdate_(RELATIVE_MOVE);
			break;
		}
	}
	
	return success;
}

//=============================================================================

bool MotorEPOS::QUICKSTOP(void)
{
	if (!motorConnected_) return false;

	return quickStopUpdate_();
}

//=============================================================================

bool MotorEPOS::HALT(void)
{
	if (!motorConnected_) return false;

	return haltUpdate_();
}

//=============================================================================

bool MotorEPOS::IS_TARGET_REACHED(void)
{
	int Statusword=0, TargetReachedBit=0;
	bool Result = false;

	if (!motorConnected_) return false;

	if (!readSTATUSWORDUpdate_(Statusword)) return false;
	TargetReachedBit = ((Statusword & TARGET_REACHED_FLAG) >> 10);
	
	if (TargetReachedBit > 0) 
		Result = true;
	else
		Result = false;

	return Result;
}

//=============================================================================

DriveStates MotorEPOS::GET_EPOS_STATE(void)
{
	int Statusword = 0;
	DriveStates Result = FAULT;

	// First, we have to get error message from previous commands; "readSTATUSWORDUpdate_" 
	// function will/may clear error or make new one.
	_status_.Message(GetErrorDescription());

	if (!readSTATUSWORDUpdate_(Statusword)) return Result;

	switch (Statusword & 0x007F)
	{
	case N_READY_TO_SWITCH_ON:
		Result = N_READY_TO_SWITCH_ON;
		_status_.Id(MotorStatus::DISABLED);
		break;
	case READY_TO_SWITCH_ON:
		Result = READY_TO_SWITCH_ON;
		_status_.Id(MotorStatus::DISABLED);
		break;
	case SWITCHED_ON:
		Result = SWITCHED_ON;
		_status_.Id(MotorStatus::DISABLED);
		break;
	case OPERATION_ENABLE:
		Result = OPERATION_ENABLE;
		_status_.Id(MotorStatus::ENABLED);
		break;
	case UNKNOWN_STATE:
	case FAULT:
		Result = FAULT;
		_status_.Id(MotorStatus::ERROR);
		break;
	case QUICK_STOP_STATE:
		Result = QUICK_STOP_STATE;
		_status_.Id(MotorStatus::ENABLED);
		break;
	case SWITCH_ON_DISABLE:
		Result = SWITCH_ON_DISABLE;
		_status_.Id(MotorStatus::DISABLED);
		break;
	default:
		Result = UNKNOWN_STATE;
		_status_.Id(MotorStatus::ERROR);
		break;
	}

	return Result;
}

//=============================================================================

bool MotorEPOS::DISABLE_EPOS(void)
{
	//! Disable voltage on EPOS and leave device in disable mode
	if (!disableVoltageUpdate_()) return false;

	return true;
}

//=============================================================================

MotorStatus MotorEPOS::Status( void )
{
	GET_EPOS_STATE();

	return _status_;
}

//=============================================================================

bool MotorEPOS::UpdateReadMeasuredData( void )
{
	long Param;
	
	if (!motorConnected_) return false;

	if (!(getActualCurrentUpdate_( Param ))) return false;
	actualCurrent_ = Param;

	if (!(getActualPositionUpdate_( Param ))) return false;
	actualPosition_ = Param;

	if (!(getActualVelocityUpdate_( Param ))) return false;
	actualVelocity_ = Param;

	return true;
}

//=============================================================================

bool MotorEPOS::UpdateSetReferences( void )
{
	if (!motorConnected_) return false;

	if (!(setPositionRefUpdate_(positionRef_))) return false;
	if (!(setVelocityRefUpdate_(velocityRef_))) return false;
	
	return true;
}

//=============================================================================

}
