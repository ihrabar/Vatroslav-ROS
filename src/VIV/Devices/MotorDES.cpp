/*! \file	MotorDES.cpp
	\brief	MotorDES member declaration.
	//==========================================================================================//
	//                           DECLARATIONS OF THE MOTOR CLASSES                              //

	//                            Author: Kristijan Brkic, 2010.                                //
	//==========================================================================================//
 */

#include <iostream>
#include <memory.h>
#include <sys/timeb.h>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/local_time/local_time.hpp"

#include "Communication/CommPrint.hpp"
#include "MotorDES.hpp"

namespace Vatroslav
{
namespace pt = boost::posix_time;

// Specific data types used in this file.
typedef unsigned short ushort;
typedef unsigned long ulong;

//=============================================================================
// Object:						RegParDES
//=============================================================================

RegParDES::RegParDES( ushort currRegGainP, ushort currRegGainI, 
					 ushort velRegGainP, ushort velRegGainI):

									currRegGainP_(currRegGainP),
									currRegGainI_(currRegGainI),
									velRegGainP_(velRegGainP),
									velRegGainI_(velRegGainI)

{
	// Make sure that gain params are in correct range.

	if (currRegGainP < PAR_CURRENT_REG_P_GAIN_DWN)
	{
		currRegGainP_ = PAR_CURRENT_REG_P_GAIN_DWN;
	}
	else
	{
		if (currRegGainP > PAR_CURRENT_REG_P_GAIN_UP)
			currRegGainP_ = PAR_CURRENT_REG_P_GAIN_UP;
	}

	if (currRegGainI < PAR_CURRENT_REG_I_GAIN_DWN)
	{
		currRegGainI_ = PAR_CURRENT_REG_I_GAIN_DWN;
	}
	else
	{
		if (currRegGainI > PAR_CURRENT_REG_I_GAIN_UP)
			currRegGainI_ = PAR_CURRENT_REG_I_GAIN_UP;
	}

	//////////////////////////////////////////

	if (velRegGainP < PAR_VELOCITY_REG_P_GAIN_DWN)
	{
		velRegGainP_ = PAR_VELOCITY_REG_P_GAIN_DWN;
	}
	else
	{
		if (velRegGainP > PAR_VELOCITY_REG_P_GAIN_UP)
			velRegGainP_ = PAR_VELOCITY_REG_P_GAIN_UP;
	}

	if (velRegGainI < PAR_VELOCITY_REG_I_GAIN_DWN)
	{
		velRegGainI_ = PAR_VELOCITY_REG_I_GAIN_DWN;
	}
	else
	{
		if (velRegGainI > PAR_VELOCITY_REG_I_GAIN_UP)
			velRegGainI_ = PAR_VELOCITY_REG_I_GAIN_UP;
	}
}

//=============================================================================

ushort RegParDES::GetCurrRegGainP(void) const 
{ 
	return currRegGainP_; 
}

//=============================================================================

void RegParDES::SetCurrRegGainP(ushort currRegGainP) 
{
	currRegGainP_ = currRegGainP;

	if (currRegGainP < PAR_CURRENT_REG_P_GAIN_DWN)
	{
		currRegGainP_ = PAR_CURRENT_REG_P_GAIN_DWN;
	}
	else
	{
		if (currRegGainP > PAR_CURRENT_REG_P_GAIN_UP)
			currRegGainP_ = PAR_CURRENT_REG_P_GAIN_UP;
	}
}

//=============================================================================

ushort RegParDES::GetCurrRegGainI(void) const 
{ 
	return currRegGainI_; 
}

//=============================================================================

void RegParDES::SetCurrRegGainI(ushort currRegGainI)
{
	currRegGainI_ = currRegGainI;

	if (currRegGainI < PAR_CURRENT_REG_I_GAIN_DWN)
	{
		currRegGainI_ = PAR_CURRENT_REG_I_GAIN_DWN;
	}
	else
	{
		if (currRegGainI > PAR_CURRENT_REG_I_GAIN_UP)
			currRegGainI_ = PAR_CURRENT_REG_I_GAIN_UP;
	}
}

//=============================================================================

ushort RegParDES::GetVelRegGainP(void) const 
{ 
	return velRegGainP_; 
}

//=============================================================================

void RegParDES::SetVelRegGainP(ushort velRegGainP)
{
	velRegGainP_ = velRegGainP;

	if (velRegGainP < PAR_VELOCITY_REG_P_GAIN_DWN)
	{
		velRegGainP_ = PAR_VELOCITY_REG_P_GAIN_DWN;
	}
	else
	{
		if (velRegGainP > PAR_VELOCITY_REG_P_GAIN_UP)
			velRegGainP_ = PAR_VELOCITY_REG_P_GAIN_UP;
	}
}

//=============================================================================

ushort RegParDES::GetVelRegGainI(void) const 
{ 
	return velRegGainI_; 
}

//=============================================================================

void RegParDES::SetVelRegGainI(ushort velRegGainI)
{
	velRegGainI_ = velRegGainI;

	if (velRegGainI < PAR_VELOCITY_REG_I_GAIN_DWN)
	{
		velRegGainI_ = PAR_VELOCITY_REG_I_GAIN_DWN;
	}
	else
	{
		if (velRegGainI > PAR_VELOCITY_REG_I_GAIN_UP)
			velRegGainI_ = PAR_VELOCITY_REG_I_GAIN_UP;
	}
}


//=========================================================================================================
// Object:									OperationParDES
//=========================================================================================================

OperationParDES::OperationParDES(DESOperationModes motorOperationMode, bool currSpeedSettingByAnalogueInput,
				bool accelerationEnabled, bool stopMotorBySoftware, bool setMaxSpeedBySoftware,
				bool setOffsetBySoftware, bool setMaxCurrBySoftware, bool setRegGainsBySoftware,
				bool enableSystemBySoftware, bool regulationModeByBits, MonitorSignalType monitorSignal,
				bool monitoringSignalByBits):

									motorOperationMode_(motorOperationMode),
									currSpeedSettingByAnalogueInput_(currSpeedSettingByAnalogueInput),
									accelerationEnabled_(accelerationEnabled),
									stopMotorBySoftware_(stopMotorBySoftware),
									setMaxSpeedBySoftware_(setMaxSpeedBySoftware),
									setOffsetBySoftware_(setOffsetBySoftware),
									setMaxCurrBySoftware_(setMaxCurrBySoftware),
									setRegGainsBySoftware_(setRegGainsBySoftware),
									enableSystemBySoftware_(enableSystemBySoftware),
									regulationModeByBits_(regulationModeByBits),
									monitorSignal_(monitorSignal),
									monitoringSignalByBits_(monitoringSignalByBits)
{
}

//=============================================================================

DESOperationModes OperationParDES::GetOperationMode(void) const
{
	return motorOperationMode_;
}

//=============================================================================

void OperationParDES::SetOperationMode(DESOperationModes motorOperationMode)
{
	motorOperationMode_ = motorOperationMode;
}

//=============================================================================

bool OperationParDES::GetCurrSpeedSettingByAnalogueInput(void) const
{
	return currSpeedSettingByAnalogueInput_;
}

//=============================================================================

void OperationParDES::SetCurrSpeedSettingByAnalogueInput(bool currSpeedSettingByAnalogueInput)
{
	currSpeedSettingByAnalogueInput_ = currSpeedSettingByAnalogueInput;
}

//=============================================================================

bool OperationParDES::GetAccelerationEnabled(void) const
{
	return accelerationEnabled_;
}

//=============================================================================

void OperationParDES::SetAccelerationEnabled(bool accelerationEnabled)
{
	accelerationEnabled_ = accelerationEnabled;
}

//=============================================================================

bool OperationParDES::GetStopMotorBySoftware(void) const
{
	return stopMotorBySoftware_;
}

//=============================================================================

void OperationParDES::SetStopMotorBySoftware(bool stopMotorBySoftware)
{
	stopMotorBySoftware_ = stopMotorBySoftware;
}

//=============================================================================

bool OperationParDES::GetMaxSpeedBySoftware(void) const
{
	return setMaxSpeedBySoftware_;
}

//=============================================================================

void OperationParDES::SetMaxSpeedBySoftware(bool setMaxSpeedBySoftware)
{
	setMaxSpeedBySoftware_ = setMaxSpeedBySoftware;
}

//=============================================================================

bool OperationParDES::GetOffsetBySoftware(void) const
{
	return setOffsetBySoftware_;
}

//=============================================================================

void OperationParDES::SetOffsetBySoftware(bool setOffsetBySoftware)
{
	setOffsetBySoftware_ = setOffsetBySoftware;
}

//=============================================================================

bool OperationParDES::GetMaxCurrentBySoftware(void) const
{
	return setMaxCurrBySoftware_;
}

//=============================================================================

void OperationParDES::SetMaxCurrentBySoftware(bool setMaxCurrBySoftware)
{
	setMaxCurrBySoftware_ = setMaxCurrBySoftware;
}

//=============================================================================

bool OperationParDES::GetRegGainsBySoftware(void) const
{
	return setRegGainsBySoftware_;
}

//=============================================================================

void OperationParDES::SetRegGainsBySoftware(bool setRegGainsBySoftware)
{
	setRegGainsBySoftware_ = setRegGainsBySoftware;
}

//=============================================================================

bool OperationParDES::GetEnableSystemBySoftware(void) const
{
	return enableSystemBySoftware_;
}

//=============================================================================

void OperationParDES::SetEnableSystemBySoftware(bool enableSystemBySoftware)
{
	enableSystemBySoftware_ = enableSystemBySoftware;
}

//=============================================================================

bool OperationParDES::GetRegulationModeBySoftware(void) const
{
	return regulationModeByBits_;
}

//=============================================================================

void OperationParDES::SetRegulationModeBySoftware(bool regulationModeByBits)
{
	regulationModeByBits_ = regulationModeByBits;
}

//=============================================================================

MonitorSignalType OperationParDES::GetMonitoringSignalType(void) const
{
	return monitorSignal_;
}

//=============================================================================

void OperationParDES::SetMonitoringSignalType(MonitorSignalType monitorSignal)
{
	monitorSignal_ = monitorSignal;
}

//=============================================================================

bool OperationParDES::GetMonitoringSignalBySoftware(void) const
{
	return monitoringSignalByBits_;
}

//=============================================================================

void OperationParDES::SetMonitoringSignalBySoftware(bool monitoringSignalByBits)
{
	monitoringSignalByBits_ = monitoringSignalByBits;
}


//=============================================================================
// Object:						MotorParDES
//=============================================================================

MotorParDES::MotorParDES(ushort ID, RegParDES regPar, OperationParDES operationPar,
					 ushort noPolePairs, ushort maxVelocity, ushort acceleration,
					 ushort peakCurrent, ushort maxContCurrent, ushort encResolution,
					 ushort maxVelocityCurrMode):

											ID_(ID),
											regPar_(regPar),
											operationPar_(operationPar),
											noPolePairs_(noPolePairs),
											maxPermissSpeed_(maxVelocity),
											acceleration_(acceleration),
											peakCurrent_(peakCurrent),
											nominalCurrent_(maxContCurrent),
											encResolution_(encResolution),
											maxVelocityCurrMode_(maxVelocityCurrMode)
{
	if (noPolePairs < PAR_POLE_PAIRS_DWN)
	{
		noPolePairs_ = PAR_POLE_PAIRS_DWN;
	}
	else
	{
		if (noPolePairs > PAR_POLE_PAIRS_UP)
			noPolePairs_ = PAR_POLE_PAIRS_UP;
	}

	if (maxVelocity < PAR_MAX_SPEED_DWN)
	{
		maxPermissSpeed_ = PAR_MAX_SPEED_DWN;
	}
	else
	{
		if (maxVelocity > PAR_MAX_SPEED_UP)
			maxPermissSpeed_ = PAR_MAX_SPEED_UP;
	}

	if (acceleration < PAR_ACCELERATION_DWN)
	{
		acceleration_ = PAR_ACCELERATION_DWN;
	}
	else
	{
		if (acceleration > PAR_ACCELERATION_UP)
			acceleration_ = PAR_ACCELERATION_UP;
	}

	if (peakCurrent < PAR_PEAK_CURRENT_DWN)
	{
		peakCurrent_ = PAR_PEAK_CURRENT_DWN;
	}
	else
	{
		if (peakCurrent > PAR_PEAK_CURRENT_UP)
			peakCurrent_ = PAR_PEAK_CURRENT_UP;
	}

	if (maxContCurrent < PAR_MAX_CONT_CURR_DWN)
	{
		nominalCurrent_ = PAR_MAX_CONT_CURR_DWN;
	}
	else
	{
		if (maxContCurrent > PAR_MAX_CONT_CURR_UP)
			nominalCurrent_ = PAR_MAX_CONT_CURR_UP;
	}

	if (encResolution < PAR_ENC_RESOLUTION_DWN)
	{
		encResolution_ = PAR_ENC_RESOLUTION_DWN;
	}
	else
	{
		if (encResolution > PAR_ENC_RESOLUTION_UP)
			encResolution_ = PAR_ENC_RESOLUTION_UP;
	}

	if (maxVelocityCurrMode < PAR_MAX_SPEED_CURR_MODE_DWN)
	{
		maxVelocityCurrMode_ = PAR_MAX_SPEED_CURR_MODE_DWN;
	}
	else
	{
		if (maxVelocityCurrMode > PAR_MAX_SPEED_CURR_MODE_UP)
			maxVelocityCurrMode_ = PAR_MAX_SPEED_CURR_MODE_UP;
	}
}

//=============================================================================

ushort MotorParDES::GetMotorID(void) const
{
	return ID_;
}

//=============================================================================

RegParDES& MotorParDES::RegParams(void)
{
	return regPar_;
}

//=============================================================================

OperationParDES& MotorParDES::OperationParams(void)
{
	return operationPar_;
}

//=============================================================================

ushort MotorParDES::GetNumberPolePairs(void) const
{
	return noPolePairs_;
}

//=============================================================================

void MotorParDES::SetNumberPolePairs(ushort noPolePairs)
{
	noPolePairs_ = noPolePairs;

	if (noPolePairs < PAR_POLE_PAIRS_DWN)
	{
		noPolePairs_ = PAR_POLE_PAIRS_DWN;
	}
	else
	{
		if (noPolePairs > PAR_POLE_PAIRS_UP)
			noPolePairs_ = PAR_POLE_PAIRS_UP;
	}
}

//=============================================================================

ushort MotorParDES::GetMaximalSpeed(void) const
{
	return maxPermissSpeed_;
}

//=============================================================================

void MotorParDES::SetMaximalSpeed(ushort maxPermissSpeed)
{
	maxPermissSpeed_ = maxPermissSpeed;

	if (maxPermissSpeed < PAR_MAX_SPEED_DWN)
	{
		maxPermissSpeed_ = PAR_MAX_SPEED_DWN;
	}
	else
	{
		if (maxPermissSpeed > PAR_MAX_SPEED_UP)
			maxPermissSpeed_ = PAR_MAX_SPEED_UP;
	}
}

//=============================================================================

ushort MotorParDES::GetMotorAcceleration(void) const
{
	return acceleration_;
}

//=============================================================================

void MotorParDES::SetMotorAcceleration(ushort acceleration)
{
	acceleration_ = acceleration;

	if (acceleration < PAR_ACCELERATION_DWN)
	{
		acceleration_ = PAR_ACCELERATION_DWN;
	}
	else
	{
		if (acceleration > PAR_ACCELERATION_UP)
			acceleration_ = PAR_ACCELERATION_UP;
	}
}

//=============================================================================

ushort MotorParDES::GetMotorPeakCurrent(void) const
{
	return peakCurrent_;
}

//=============================================================================

void MotorParDES::SetMotorPeakCurrent(ushort peakCurrent)
{
	peakCurrent_ = peakCurrent;

	if (peakCurrent < PAR_PEAK_CURRENT_DWN)
	{
		peakCurrent_ = PAR_PEAK_CURRENT_DWN;
	}
	else
	{
		if (peakCurrent > PAR_PEAK_CURRENT_UP)
			peakCurrent_ = PAR_PEAK_CURRENT_UP;
	}
}

//=============================================================================

ulong MotorParDES::GetMaximalCurrent(void) const
{
	return nominalCurrent_;
}

//=============================================================================

void MotorParDES::SetMaximalCurrent(ulong nominalCurrent)
{
	nominalCurrent_ = (ushort) nominalCurrent;

	if (nominalCurrent < PAR_MAX_CONT_CURR_DWN)
	{
		nominalCurrent_ = PAR_MAX_CONT_CURR_DWN;
	}
	else
	{
		if (nominalCurrent > PAR_MAX_CONT_CURR_UP)
			nominalCurrent_ = PAR_MAX_CONT_CURR_UP;
	}
}

//=============================================================================

ushort MotorParDES::GetEncoderResolution(void) const
{
	return encResolution_;
}

//=============================================================================

void MotorParDES::SetEncoderResolution(ushort encResolution)
{
	encResolution_ = encResolution;

	if (encResolution < PAR_ENC_RESOLUTION_DWN)
	{
		encResolution_ = PAR_ENC_RESOLUTION_DWN;
	}
	else
	{
		if (encResolution > PAR_ENC_RESOLUTION_UP)
			encResolution_ = PAR_ENC_RESOLUTION_UP;
	}
}

//=============================================================================

ushort MotorParDES::GetMaxVelocityCurrentMode(void) const
{
	return maxVelocityCurrMode_;
}

//=============================================================================

void MotorParDES::SetMaxVelocityCurrentMode(ushort maxVelocityCurrMode)
{
	maxVelocityCurrMode_ = maxVelocityCurrMode;

	if (maxVelocityCurrMode < PAR_MAX_SPEED_CURR_MODE_DWN)
	{
		maxVelocityCurrMode_ = PAR_MAX_SPEED_CURR_MODE_DWN;
	}
	else
	{
		if (maxVelocityCurrMode > PAR_MAX_SPEED_CURR_MODE_UP)
			maxVelocityCurrMode_ = PAR_MAX_SPEED_CURR_MODE_UP;
	}
}

//=============================================================================
// Object:						  MotorDES
//=============================================================================

MotorDES::MotorDES( MotorParDES motpar, CommPtr pComm ) : pComm_( pComm ),
                                                motPar_( motpar )
{
	//! Clear data buffers for sending and receiving SDO frames
	memset(_dataSendSDO_, 0, 8);
	memset(_dataReceiveSDO_, 0, 8);

	//! Clear all object variables
	motorConnected_ = false;

	actualCurrent_ = 0;
	actualPosition_ = 0;
	actualVelocity_ = 0;

	currentRef_ = 0;
	positionRef_ = 0;
	velocityRef_ = 0;

	//! Clear errors at motor object construction
	_ErrorCode_ = DES_ERROR_NONE;
	_status_.Message("Indeterminated status!");
	_status_.Id(MotorStatus::DISABLED);

	//! Set all remaining variables in this class
	opCode_ = READ_SYS_STATUS_OPCODE;
	dataLength_ = READ_SYS_STATUS_LEN;
	functionResponse_ = READ_SYS_STATUS_RESPONSE;
	_SDO_ID_ = CAN_RX_ID + motPar_.GetMotorID();

}

//=============================================================================

MotorDES::~MotorDES()
{
	motorConnected_ = false;
}

//=============================================================================

/* virtual */
long MotorDES::Current(void) const
{
	return actualCurrent_;
}

//=============================================================================

/* virtual */
long MotorDES::Position(void) const
{
	return actualPosition_;
}

//=============================================================================

/* virtual */
long MotorDES::Velocity(void) const
{
	return actualVelocity_;
}

//=============================================================================

/* virtual */
void MotorDES::CurrentRef(long Current)
{
	currentRef_ = (short) Current;
}

//=============================================================================

/* virtual */
void MotorDES::VelocityRef(long Velocity)
{
	velocityRef_ = (short) Velocity;
}

//=============================================================================

/* virtual */
long MotorDES::CurrentRef(void) const
{
	return currentRef_;
}

//=============================================================================

/* virtual */
long MotorDES::VelocityRef(void) const
{
	return velocityRef_;
}

//=============================================================================

long MotorDES::GetErrorCode(void) const
{
	return _ErrorCode_;
}

//=============================================================================

std::string MotorDES::GetErrorDescription(void)
{
	switch (_ErrorCode_)
	{
		case DES_ERROR_NONE: 
			return "No error!";

		case DES_HALL_ERROR:
			return "Hall sensor error!";

		case DES_INDX_PROCESSING_ERROR:
			return "Index processing error!";

		case DES_WRONG_ENC_RESOLUTION:
			return "Wrong setting of encoder resolution!";

		case DES_HALL_SENSOR_3_ERROR:
			return "Hall sensor 3 not found!";

		case DES_OVER_CURRENT:
			return "Over current error!";

		case DES_OVER_VOLTAGE:
			return "Over voltage error!";

		case DES_SPEED_ERROR:
			return "Over speed error!";

		case DES_LOW_VOLTAGE:
			return "Supply voltage too low for operation!";

		case DES_ANGLE_DETECTION:
			return "Angle detection error!";

		case DES_OVER_TEMPERATURE:
			return "Over temperature error!";

		case DES_PARAM_OUT_RANGE:
			return "Parameter out of range!";

		case DES_CAN_WARNING:
			return "CAN - Warning Status!";

		case DES_CAN_PASSIVE_STATUS_ERROR:
			return "CAN - Error Passive Status!";

		case DES_CAN_BUS_OFF:
			return "CAN - Bus Off Status!";

		case DES_CAN_ACKNOWLEDGE_ERROR:
			return "CAN - Acknowledge Error!";

		case DES_CAN_STUFF_ERROR:
			return "CAN - Stuff Error!";

		case DES_CAN_CRC_ERROR:
			return "CAN - CRC Error!";

		case DES_CAN_STUCK_ERROR:
			return "CAN - Stuck at dominant Error!";

		case DES_CAN_BIT_FLAG_ERROR:
			return "CAN - Bit Error Flag!";

		case DES_CAN_FORM_ERROR:
			return "CAN - Form Error Flag!";

		case DES_CAN_PDO_FREQ_TOO_HIGH:
			return "CAN - PDO accessing frequency is too high!";

		case DES_CAN_PDO_OVERFLOW:
			return "CAN - PDO overflow to lose sending message!";

		case DES_CAN_TXPDO_ABORT_ACK:
			return "CAN - TxPDO abort acknowledge in sending a message!";

		case DES_CAN_TXSDO_ABORT_ACK:
			return "CAN - TxSDO abort acknowledge in sending a message!";

		case DES_CAN_RXPDO_MSG_LOST:
			return "CAN - RxPDO receive message lost!";

		case DES_CAN_RXSDO_MSG_LOST:
			return "CAN - RxSDO receive message lost!";

		default: 
			return "UNKNOWN ERROR";
	}

	return "";
}

//=============================================================================

void MotorDES::delay_(ushort milisec)
{
	struct timeb tp;
	time_t t1=0, t2=0;

	ftime(&tp);
	t1 = tp.millitm + (tp.time & 0xFFFFF) * 1000;

	do
	{
		ftime(&tp);
		t2 = tp.millitm + (tp.time & 0xFFFFF) * 1000;
	}while (abs((long)(t1-t2)) < milisec);

}

//=============================================================================

bool MotorDES::checkReceivedSDOFrameFormat(void)
{
	if ( CAN_TX_ID + motPar_.GetMotorID() == _SDO_ID_ )
		return true;	//! Received Frame is SDO
	else
		return false;	//! Received Frame is PDO
}

//=============================================================================

void MotorDES::formatSDOSendFrame_(ushort Param1, ushort Param2, ushort Param3)
{	
	_dataSendSDO_[0] = DUMMY;
	_dataSendSDO_[1] = opCode_;
	_dataSendSDO_[2] = (char) (Param1 & 0xFF);
	_dataSendSDO_[3] = (char) ((Param1 >> 8) & 0xFF);
	_dataSendSDO_[4] = (char) (Param2 & 0xFF);
	_dataSendSDO_[5] = (char) ((Param2 >> 8) & 0xFF);
	_dataSendSDO_[6] = (char) (Param3 & 0xFF);
	_dataSendSDO_[7] = (char) ((Param3 >> 8) & 0xFF);

	// Format RxSDO ID value
	_SDO_ID_ = (CAN_RX_ID + motPar_.GetMotorID());
}

//=============================================================================

bool MotorDES::UpdateCmdToDES_(void)
{
	bool success = false;
	struct timeb tp;
	time_t t1=0, t2=0;
	CommMsg msg( _SDO_ID_ , _dataSendSDO_, 8, pt::microsec_clock::local_time());
	
	delay_(DELAY_TIME_MS_DES);

	memset(_dataReceiveSDO_, 0, 8);

	ftime(&tp);
	t1 = tp.millitm + (tp.time & 0xFFFFF) * 1000;

	success = pComm_->Send( msg );

	if ( success && functionResponse_ )
	{
		do {
			success = pComm_->Receive( msg, 1000 );

			ftime(&tp);
			t2 = tp.millitm + (tp.time & 0xFFFFF) * 1000;
		} while ( (msg.Id() == 0) || (abs((long)(t1-t2)) < DELAY_TIME_RESPONSE) );

		_SDO_ID_ = msg.Id();

		memcpy( _dataReceiveSDO_, msg.Data(), msg.Size() );
	}

	return success;
}

//================================================================================================================//
//                         Declarations of DES command functions and related parameters                           //
//================================================================================================================//

//+++++++++++++++++++++++++++++++++++++++ "Status Functions" members +++++++++++++++++++++++++++++++++++++++++++++//

bool MotorDES::readSysStatusUpdate_(ushort Dummy, ushort &Status)
{
	bool bResult = false;

	opCode_ = READ_SYS_STATUS_OPCODE;
	dataLength_ = READ_SYS_STATUS_LEN;
	functionResponse_ = READ_SYS_STATUS_RESPONSE;
	formatSDOSendFrame_(Dummy, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();
	Status = 0;
	if (checkReceivedSDOFrameFormat())  // the received message is correct; SDO
		Status = (_dataReceiveSDO_[2] & 0xFF) | ((_dataReceiveSDO_[3] & 0xFF) << 8);
	
	return bResult;
}

//================================================================================================================//

bool MotorDES::readErrorUpdate_(ushort Dummy, ushort &Error)
{
	bool bResult = false;

	opCode_ = READ_ERROR_OPCODE;
	dataLength_ = READ_ERROR_LEN;
	functionResponse_ = READ_ERROR_RESPONSE;
	formatSDOSendFrame_(Dummy, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();
	Error = 0;
	if (checkReceivedSDOFrameFormat())  // the received message is correct; SDO
		Error = (_dataReceiveSDO_[2] & 0xFF) | ((_dataReceiveSDO_[3] & 0xFF) << 8);

	return bResult;
}

//================================================================================================================//

bool MotorDES::clearErrorUpdate_(ushort Dummy)
{
	bool bResult = false;

	opCode_ = CLEAR_ERROR_OPCODE;
	dataLength_ = CLEAR_ERROR_LEN;
	functionResponse_ = CLEAR_ERROR_RESPONSE;
	formatSDOSendFrame_(Dummy, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

//================================================================================================================//

bool MotorDES::resetUpdate_(ushort Dummy)
{
	bool bResult = false;

	opCode_ = RESET_OPCODE;
	dataLength_ = RESET_LEN;
	functionResponse_ = RESET_RESPONSE;
	formatSDOSendFrame_(Dummy, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

//================================================================================================================//

bool MotorDES::enableUpdate_(DESSystemStates newState)
{
	bool bResult = false;

	opCode_ = ENABLE_OPCODE;
	dataLength_ = ENABLE_LEN;
	functionResponse_ = ENABLE_RESPONSE;
	formatSDOSendFrame_( (ushort) newState, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

// +++++++++++++++++++++++++++++++++++ "System Parameter Functions" members ++++++++++++++++++++++++++++++++++++++//

bool MotorDES::readTempParamUpdate_(ushort paramNb, DataFormat dataFormat, ulong &value)
{
	bool bResult = false;

	opCode_ = READ_TEMP_PARAM_OPCODE;
	dataLength_ = READ_TEMP_PARAM_LEN;
	functionResponse_ = READ_TEMP_PARAM_RESPONSE;
	formatSDOSendFrame_(paramNb, (ushort) dataFormat, DUMMY);
	bResult = UpdateCmdToDES_();
	value = 0;
	if (checkReceivedSDOFrameFormat())  // the received message is correct; SDO
	{
		value = (ulong) ((_dataReceiveSDO_[2] & 0xFF) | ((_dataReceiveSDO_[3] & 0xFF) << 8) | ((_dataReceiveSDO_[4] & 0xFF) << 16) |
				((_dataReceiveSDO_[5] & 0xFF) << 24) );
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::setTempParamUpdate_(ushort paramNb, DataFormat dataFormat, ushort value)
{
	bool bResult = false;

	opCode_ = SET_TEMP_PARAM_OPCODE;
	dataLength_ = SET_TEMP_PARAM_LEN;
	functionResponse_ = SET_TEMP_PARAM_RESPONSE;
	formatSDOSendFrame_(paramNb, (ushort) dataFormat, value);
	bResult = UpdateCmdToDES_();

	return bResult;
}

//================================================================================================================//

bool MotorDES::resetTempParamUpdate_(ushort Dummy)
{
	bool bResult = false;

	opCode_ = RESET_TEMP_PARAM_OPCODE;
	dataLength_ = RESET_TEMP_PARAM_LEN;
	functionResponse_ = RESET_TEMP_PARAM_RESPONSE;
	formatSDOSendFrame_(Dummy, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

//================================================================================================================//

bool MotorDES::saveTempParamUpdate_(ushort Dummy)
{
	bool bResult = false;

	opCode_ = SAVE_TEMP_PARAM_OPCODE;
	dataLength_ = SAVE_TEMP_PARAM_LEN;
	functionResponse_ = SAVE_TEMP_PARAM_RESPONSE;
	formatSDOSendFrame_(Dummy, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

//================================================================================================================//

bool MotorDES::sysParSetDefaultUpdate_(ushort Dummy)
{
	bool bResult = false;

	opCode_ = SYS_PAR_SET_DEFAULT_OPCODE;
	dataLength_ = SYS_PAR_SET_DEFAULT_LEN;
	functionResponse_ = SYS_PAR_SET_DEFAULT_RESPONSE;
	formatSDOSendFrame_(Dummy, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

// ++++++++++++++++++++++++++++++++++++++++ "Setting Functions" members ++++++++++++++++++++++++++++++++++++++++++//

bool MotorDES::setVelocityUpdate_(short newVelocity)
{
	bool bResult = false;

	opCode_ = SET_VELOCITY_OPCODE;
	dataLength_ = SET_VELOCITY_LEN;
	functionResponse_ = SET_VELOCITY_RESPONSE;
	formatSDOSendFrame_(newVelocity, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

//================================================================================================================//

bool MotorDES::setCurrentUpdate_(short newCurrent)
{
	bool bResult = false;

	opCode_ = SET_CURRENT_OPCODE;
	dataLength_ = SET_CURRENT_LEN;
	functionResponse_ = SET_CURRENT_RESPONSE;
	formatSDOSendFrame_(newCurrent, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

//================================================================================================================//

bool MotorDES::stopMotionUpdate_(ushort Dummy)
{
	bool bResult = false;

	opCode_ = STOP_MOTION_OPCODE;
	dataLength_ = STOP_MOTION_LEN;
	functionResponse_ = STOP_MOTION_RESPONSE;
	formatSDOSendFrame_(Dummy, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

// ++++++++++++++++++++++++++++++++++++++++ "Monitor Functions" members ++++++++++++++++++++++++++++++++++++++++++//

bool MotorDES::readVelocityIsMustUpdate_(MonitorValueTypes Type, short &isVelocity, short &mustVelocity)
{
	bool bResult = false;

	opCode_ = READ_VELOCITY_IS_MUST_OPCODE;
	dataLength_ = READ_VELOCITY_IS_MUST_LEN;
	functionResponse_ = READ_VELOCITY_IS_MUST_RESPONSE;
	formatSDOSendFrame_( (ushort) Type, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();
	isVelocity = 0;
	mustVelocity = 0;
	if (checkReceivedSDOFrameFormat())  // the received message is correct; SDO
	{
		isVelocity = (_dataReceiveSDO_[2] & 0xFF) | ((_dataReceiveSDO_[3] & 0xFF) << 8);
		mustVelocity = (_dataReceiveSDO_[4] & 0xFF) | ((_dataReceiveSDO_[5] & 0xFF) << 8);
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::readCurrentIsMustUpdate_(MonitorValueTypes Type, short &isCurrent, short &mustCurrent)
{
	bool bResult = false;

	opCode_ = READ_CURRENT_IS_MUST_OPCODE;
	dataLength_ = READ_CURRENT_IS_MUST_LEN;
	functionResponse_ = READ_CURRENT_IS_MUST_RESPONSE;
	formatSDOSendFrame_( (ushort) Type, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();
	isCurrent = 0;
	mustCurrent = 0;
	if (checkReceivedSDOFrameFormat())  // the received message is correct; SDO
	{
		isCurrent = (_dataReceiveSDO_[2] & 0xFF) | ((_dataReceiveSDO_[3] & 0xFF) << 8);
		mustCurrent = (_dataReceiveSDO_[6] & 0xFF) | ((_dataReceiveSDO_[7] & 0xFF) << 8);
	}

	return bResult;
}

// +++++++++++++++++++++++++++++++++ "CAN Bus Configuration Functions" members +++++++++++++++++++++++++++++++++++//

bool MotorDES::setModuleIDUpdate_(ushort moduleID)
{
	bool bResult = false;

	opCode_ = SET_MODULE_ID_OPCODE;
	dataLength_ = SET_MODULE_ID_LEN;
	functionResponse_ = SET_MODULE_ID_RESPONSE;
	formatSDOSendFrame_(moduleID, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

//================================================================================================================//

bool MotorDES::setTPDOIDUpdate_(ushort tpdoID)
{
	bool bResult = false;

	opCode_ = SET_TPDO_ID_OPCODE;
	dataLength_ = SET_TPDO_ID_LEN;
	functionResponse_ = SET_TPDO_ID_RESPONSE;
	formatSDOSendFrame_(tpdoID, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

//================================================================================================================//

bool MotorDES::setRPDOIDUpdate_(ushort rpdoID)
{
	bool bResult = false;

	opCode_ = SET_RPDO_ID_OPCODE;
	dataLength_ = SET_RPDO_ID_LEN;
	functionResponse_ = SET_RPDO_ID_RESPONSE;
	formatSDOSendFrame_(rpdoID, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

//================================================================================================================//

bool MotorDES::readModuleIDUpdate_(ushort Dummy, ushort &moduleID)
{
	bool bResult = false;

	opCode_ = READ_MODULE_ID_OPCODE;
	dataLength_ = READ_MODULE_ID_LEN;
	functionResponse_ = READ_MODULE_ID_RESPONSE;
	formatSDOSendFrame_(Dummy, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();
	moduleID = 0;
	if (checkReceivedSDOFrameFormat())  // the received message is correct; SDO
		moduleID = (_dataReceiveSDO_[2] & 0xFF) | ((_dataReceiveSDO_[3] & 0xFF) << 8);

	return bResult;
}

//================================================================================================================//

bool MotorDES::setCANBitrateUpdate_(DESCANBitrate bitrate)
{
	bool bResult = false;

	opCode_ = SET_CAN_BITRATE_OPCODE;
	dataLength_ = SET_CAN_BITRATE_LEN;
	functionResponse_ = SET_CAN_BITRATE_RESPONSE;
	formatSDOSendFrame_((ushort) bitrate, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

//================================================================================================================//

bool MotorDES::readCANErrorUpdate_(ushort Dummy, ushort &ErrorCode)
{
	bool bResult = false;

	opCode_ = READ_CAN_ERROR_OPCODE;
	dataLength_ = READ_CAN_ERROR_LEN;
	functionResponse_ = READ_CAN_ERROR_RESPONSE;
	formatSDOSendFrame_(Dummy, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();
	ErrorCode = 0;
	if (checkReceivedSDOFrameFormat())  // the received message is correct; SDO
		ErrorCode = (_dataReceiveSDO_[2] & 0xFF) | ((_dataReceiveSDO_[3] & 0xFF) << 8);

	return bResult;
}

//================================================================================================================//

bool MotorDES::configPDOUpdate_(PDOConfiguration action)
{
	bool bResult = false;

	opCode_ = CONFIG_PDO_OPCODE;
	dataLength_ = CONFIG_PDO_LEN;
	functionResponse_ = CONFIG_PDO_RESPONSE;
	formatSDOSendFrame_((ushort) action, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

//================================================================================================================//

bool MotorDES::resetCANErrorUpdate_(ushort Dummy)
{
	bool bResult = false;

	opCode_ = RESET_CAN_ERROR_OPCODE;
	dataLength_ = RESET_CAN_ERROR_LEN;
	functionResponse_ = RESET_CAN_ERROR_RESPONSE;
	formatSDOSendFrame_(Dummy, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

//================================================================================================================//

bool MotorDES::resetCANUpdate_(ushort Dummy)
{
	bool bResult = false;

	opCode_ = RESET_CAN_OPCODE;
	dataLength_ = RESET_CAN_LEN;
	functionResponse_ = RESET_CAN_RESPONSE;
	formatSDOSendFrame_(Dummy, DUMMY, DUMMY);
	bResult = UpdateCmdToDES_();

	return bResult;
}

//================================================================================================================//
//                     End of declarations of DES command functions and related parameters                        //
//================================================================================================================//





//================================================================================================================//
//                                    DES System Parameters declarations                                          //
//================================================================================================================//

//+++++++++++++++++++++++++++++++++++ Functions for modifying regulator parameters +++++++++++++++++++++++++++++++//

bool MotorDES::setCurrentRegPGainUpdate_(ushort Gain)
{
	bool bResult;

	bResult = setTempParamUpdate_(PAR_CURRENT_REG_P_GAIN_NB, WORD_DATA, Gain);
	if (bResult)
	{
		bResult = saveTempParamUpdate_(DUMMY);
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::getCurrentRegPGainUpdate_(ushort &Gain)
{
	ulong tmpGain = PAR_CURRENT_REG_P_GAIN_DWN;
	bool result = readTempParamUpdate_(PAR_CURRENT_REG_P_GAIN_NB, WORD_DATA, tmpGain);
	Gain = static_cast<ushort>(tmpGain);

	return result;
}

//================================================================================================================//

bool MotorDES::setCurrentRegIGainUpdate_(ushort Gain)
{
	bool bResult;

	bResult = setTempParamUpdate_(PAR_CURRENT_REG_I_GAIN_NB, WORD_DATA, Gain);
	if (bResult)
	{
		bResult = saveTempParamUpdate_(DUMMY);
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::getCurrentRegIGainUpdate_(ushort &Gain)
{
	ulong tmpGain = PAR_CURRENT_REG_I_GAIN_DWN;
	bool result = readTempParamUpdate_(PAR_CURRENT_REG_I_GAIN_NB, WORD_DATA, tmpGain);
	Gain = static_cast<ushort>(tmpGain);

	return result;
}

//================================================================================================================//

bool MotorDES::setVelocityRegPGainUpdate_(ushort Gain)
{
	bool bResult;

	bResult = setTempParamUpdate_(PAR_VELOCITY_REG_P_GAIN_NB, WORD_DATA, Gain);
	if (bResult)
	{
		bResult = saveTempParamUpdate_(DUMMY);
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::getVelocityRegPGainUpdate_(ushort &Gain)
{
	ulong tmpGain = PAR_VELOCITY_REG_P_GAIN_DWN;
	bool result = readTempParamUpdate_(PAR_VELOCITY_REG_P_GAIN_NB, WORD_DATA, tmpGain);
	Gain = static_cast<ushort>(tmpGain);

	return result;
}

//================================================================================================================//

bool MotorDES::setVelocityRegIGainUpdate_(ushort Gain)
{
	bool bResult;

	bResult = setTempParamUpdate_(PAR_VELOCITY_REG_I_GAIN_NB, WORD_DATA, Gain);
	if (bResult)
	{
		bResult = saveTempParamUpdate_(DUMMY);
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::getVelocityRegIGainUpdate_(ushort &Gain)
{
	ulong tmpGain = PAR_VELOCITY_REG_I_GAIN_DWN;
	bool result = readTempParamUpdate_(PAR_VELOCITY_REG_I_GAIN_NB, WORD_DATA, tmpGain);
	Gain = static_cast<ushort>(tmpGain);

	return result;
}


//+++++++++++++++++++++++++++ Functions for modifying motor and sensor data parameters ++++++++++++++++++++++++++//

bool MotorDES::setPeakCurrentUpdate_(ushort Current)
{
	bool bResult;

	bResult = setTempParamUpdate_(PAR_PEAK_CURRENT_NB, WORD_DATA, Current);
	if (bResult)
	{
		bResult = saveTempParamUpdate_(DUMMY);
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::getPeakCurrentUpdate_(ushort &Current)
{
	ulong tmpCurrent = PAR_PEAK_CURRENT_DWN;
	bool result = readTempParamUpdate_(PAR_PEAK_CURRENT_NB, WORD_DATA, tmpCurrent);
	Current = static_cast<ushort>(tmpCurrent);

	return result;
}

//================================================================================================================//

bool MotorDES::setNominalCurrentUpdate_(ushort Current)
{
	bool bResult;

	bResult = setTempParamUpdate_(PAR_MAX_CONT_CURR_NB, WORD_DATA, Current);
	if (bResult)
	{
		bResult = saveTempParamUpdate_(DUMMY);
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::getNominalCurrentUpdate_(ushort &Current)
{
	ulong tmpCurrent = PAR_MAX_CONT_CURR_DWN;
	bool result = readTempParamUpdate_(PAR_MAX_CONT_CURR_NB, WORD_DATA, tmpCurrent);
	Current = static_cast<ushort>(tmpCurrent);

	return result;
}

//================================================================================================================//

bool MotorDES::setEncoderResolutionUpdate_(ushort Resolution)
{
	bool bResult;

	bResult = setTempParamUpdate_(PAR_ENC_RESOLUTION_NB, WORD_DATA, Resolution);
	if (bResult)
	{
		bResult = saveTempParamUpdate_(DUMMY);
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::getEncoderResolutionUpdate_(ushort &Resolution)
{
	ulong tmpResolution = PAR_ENC_RESOLUTION_DWN;
	bool result = readTempParamUpdate_(PAR_ENC_RESOLUTION_NB, WORD_DATA, tmpResolution);
	Resolution = static_cast<ushort>(tmpResolution);

	return result;
}

//================================================================================================================//

bool MotorDES::setPolePairsUpdate_(ushort PolePairs)
{
	bool bResult;

	bResult = setTempParamUpdate_(PAR_POLE_PAIRS_NB, WORD_DATA, PolePairs);
	if (bResult)
	{
		bResult = saveTempParamUpdate_(DUMMY);
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::getPolePairsUpdate_(ushort &PolePairs)
{
	ulong tmpPolePairs = PAR_POLE_PAIRS_DWN;
	bool result = readTempParamUpdate_(PAR_POLE_PAIRS_NB, WORD_DATA, tmpPolePairs);
	PolePairs = static_cast<ushort>(tmpPolePairs);

	return result;
}

//+++++++++++++++++++++++++++++++++++ Functions for modifying motion parameters ++++++++++++++++++++++++++++++++++//

bool MotorDES::setMaximalSpeedUpdate_(ushort Speed)
{
	bool bResult;

	bResult = setTempParamUpdate_(PAR_MAX_SPEED_NB, WORD_DATA, Speed);
	if (bResult)
	{
		bResult = saveTempParamUpdate_(DUMMY);
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::getMaximalSpeedUpdate_(ushort &Speed)
{
	ulong tmpSpeed = PAR_MAX_SPEED_DWN;
	bool result = readTempParamUpdate_(PAR_MAX_SPEED_NB, WORD_DATA, tmpSpeed);
	Speed = static_cast<ushort>(tmpSpeed);

	return result;
}

//================================================================================================================//

bool MotorDES::setMotorAccelerationUpdate_(ushort Acceleration)
{
	bool bResult;

	bResult = setTempParamUpdate_(PAR_ACCELERATION_NB, WORD_DATA, Acceleration);
	if (bResult)
	{
		bResult = saveTempParamUpdate_(DUMMY);
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::getMotorAccelerationUpdate_(ushort &Acceleration)
{
	ulong tmpAcceleration = PAR_ACCELERATION_DWN;
	bool result = readTempParamUpdate_(PAR_ACCELERATION_NB, WORD_DATA, tmpAcceleration);
	Acceleration = static_cast<ushort>(tmpAcceleration);

	return result;
}

//================================================================================================================//

bool MotorDES::setMaxSpeedCurrentModeUpdate_(ushort Speed)
{
	bool bResult;

	bResult = setTempParamUpdate_(PAR_MAX_SPEED_CURR_MODE_NB, WORD_DATA, Speed);
	if (bResult)
	{
		bResult = saveTempParamUpdate_(DUMMY);
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::getMaxSpeedCurrentModeUpdate_(ushort &Speed)
{
	ulong tmpSpeed = PAR_MAX_SPEED_CURR_MODE_DWN;
	bool result = readTempParamUpdate_(PAR_MAX_SPEED_CURR_MODE_NB, WORD_DATA, tmpSpeed);
	Speed = static_cast<ushort>(tmpSpeed);

	return result;
}

//================================================================================================================//

bool MotorDES::setErrorProcUpdate_(ErrorProc Processing)
{
	bool bResult;

	bResult = setTempParamUpdate_(PAR_ERROR_PROC_NB, WORD_DATA, (ushort) Processing);
	if (bResult)
	{
		bResult = saveTempParamUpdate_(DUMMY);
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::getErrorProcUpdate_(ErrorProc &Processing)
{
	ulong tmpProcessing = static_cast<ulong>( DISABLE_EP );
	bool result = readTempParamUpdate_(PAR_ERROR_PROC_NB, WORD_DATA, tmpProcessing);
	Processing = static_cast<ErrorProc>(tmpProcessing);

	return result;
}

//+++++++++++++++++++++++++++++++ Functions for reading some CAN communication parameters ++++++++++++++++++++++++//

bool MotorDES::getCANmoduleIDUpdate_(ushort &ID)
{
	ulong tmpID = PAR_CAN_MODULE_ID_DWN;
	bool result = readTempParamUpdate_(PAR_CAN_MODULE_ID_NB, WORD_DATA, tmpID);
	ID = static_cast<ushort>(tmpID);

	return result;
}

//================================================================================================================//

bool MotorDES::getCANRxPDOIDUpdate_(ushort &RxPDO_ID)
{
	ulong tmpRxPDO_ID = PAR_CAN_RXPDO_ID_DWN;
	bool result = readTempParamUpdate_(PAR_CAN_RXPDO_ID_NB, WORD_DATA, tmpRxPDO_ID);
	RxPDO_ID = static_cast<ushort>(tmpRxPDO_ID);

	return result;
}

//================================================================================================================//

bool MotorDES::getCANTxPDOIDUpdate_(ushort &TxPDO_ID)
{
	ulong tmpTxPDO_ID = PAR_CAN_TXPDO_ID_DWN + 1;
	bool result = readTempParamUpdate_(PAR_CAN_TXPDO_ID_NB, WORD_DATA, tmpTxPDO_ID);
	TxPDO_ID = static_cast<ushort>(tmpTxPDO_ID);

	return result;
}

//================================================================================================================//

bool MotorDES::getCANRxSDOIDUpdate_(ushort &RxSDO_ID)
{
	ulong tmpRxSDO_ID = PAR_CAN_RXSDO_ID_DWN;
	bool result = readTempParamUpdate_(PAR_CAN_RXSDO_ID_NB, WORD_DATA, tmpRxSDO_ID);
	RxSDO_ID = static_cast<ushort>(tmpRxSDO_ID);

	return result;
}

//================================================================================================================//

bool MotorDES::getCANTxSDOIDUpdate_(ushort &TxSDO_ID)
{
	ulong tmpTxSDO_ID = PAR_CAN_TXSDO_ID_DWN;
	bool result = readTempParamUpdate_(PAR_CAN_TXSDO_ID_NB, WORD_DATA, tmpTxSDO_ID);
	TxSDO_ID = static_cast<ushort>(tmpTxSDO_ID);

	return result;
}

//================================================================================================================//
//                                 End of DES System Parameters declarations                                      //
//================================================================================================================//




//================================================================================================================//
//                                     DES Status Variables declarations                                          //
//================================================================================================================//

bool MotorDES::getSystemOperatingStatusUpdate_(ushort &SystemStatus)
{
	ulong tmpSystemStatus = 0;
	bool result = readTempParamUpdate_(PAR_SYS_OPER_STATUS_NB, WORD_DATA, tmpSystemStatus);
	SystemStatus = static_cast<short>(tmpSystemStatus);

	return result;
}

//================================================================================================================//

bool MotorDES::getActualMeanCurrD_AxisUpdate_(short &MeanCurrent)
{
	ulong tmpMeanCurrent = 0;
	bool result = readTempParamUpdate_(PAR_MEAN_CURR_D_AXIS_NB, WORD_DATA, tmpMeanCurrent);
	MeanCurrent = static_cast<short>(tmpMeanCurrent);

	return result;
}

//================================================================================================================//

bool MotorDES::getActualMeanCurrQ_AxisUpdate_(short &MeanCurrent)
{
	ulong tmpMeanCurrent = 0;
	bool result = readTempParamUpdate_(PAR_MEAN_CURR_Q_AXIS_NB, WORD_DATA, tmpMeanCurrent);
	MeanCurrent = static_cast<short>(tmpMeanCurrent);

	return result;
}

//================================================================================================================//

bool MotorDES::getCurrentSettingValueUpdate_(short &Current)
{
	ulong tmpCurrent = 0;
	bool result = readTempParamUpdate_(PAR_CURR_SETTING_VALUE_NB, WORD_DATA, tmpCurrent);
	Current = static_cast<short>(tmpCurrent);

	return result;
}

//================================================================================================================//

bool MotorDES::getSpeedSettingValueUpdate_(short &Speed)
{
	ulong tmpSpeed = 0;
	bool result = readTempParamUpdate_(PAR_SPEED_SETTING_VALUE_NB, WORD_DATA, tmpSpeed);
	Speed = static_cast<short>(tmpSpeed);

	return result;
}

//================================================================================================================//

bool MotorDES::getSpeedActualMeanValueUpdate_(short &Speed)
{
	ulong tmpSpeed = 0;
	bool result = readTempParamUpdate_(PAR_MEAN_SPEED_VALUE_NB, WORD_DATA, tmpSpeed);
	Speed = static_cast<short>(tmpSpeed);

	return result;
}

//================================================================================================================//

bool MotorDES::getAbsoluteRotorPositionUpdate_(long &Position)
{
	ulong tmpPosition = 0;
	bool result = readTempParamUpdate_(PAR_ABSOLUTE_ROTOR_POS_NB, LWORD_DATA, tmpPosition);
	Position = static_cast<short>(tmpPosition);

	return result;
}

//================================================================================================================//

bool MotorDES::getStandardErrorUpdate_(short &Error)
{
	ulong tmpError = 0;
	bool result = readTempParamUpdate_(PAR_CAN_ERROR_NB, WORD_DATA, tmpError);
	Error = static_cast<short>(tmpError);

	return result;
}

//================================================================================================================//

bool MotorDES::getActualCurrentQAxisUpdate_(short &Current)
{
	ulong tmpCurrent = 0;
	bool result = readTempParamUpdate_(PAR_ACT_CURR_VALUE_NB, WORD_DATA, tmpCurrent);
	Current = static_cast<short>(tmpCurrent);

	return result;
}

//================================================================================================================//

bool MotorDES::getActualSpeedValueUpdate_(short &Speed)
{
	ulong tmpSpeed = 0;
	bool result = readTempParamUpdate_(PAR_ACT_SPEED_VALUE_NB, WORD_DATA, tmpSpeed);
	Speed = static_cast<short>(tmpSpeed);

	return result;

}

//================================================================================================================//

bool MotorDES::getEncoderCounterUpdate_(short &Counter)
{
	ulong tmpCounter = 0;
	bool result = readTempParamUpdate_(PAR_ENC_COUNTER_NB, WORD_DATA, tmpCounter);
	Counter = static_cast<short>(tmpCounter);

	return result;
}

//================================================================================================================//
//                                  End of DES Status Variables declarations                                      //
//================================================================================================================//


bool MotorDES::UpdateOperationParRead( void )
{
	bool bResult = true;
	ulong operation_bits = 0;

	bResult = readTempParamUpdate_(PAR_SYS_CONFIG_NB, WORD_DATA, operation_bits);
	if (bResult)
	{
		motPar_.OperationParams().SetCurrSpeedSettingByAnalogueInput( (operation_bits & 0x1) !=0 );
		motPar_.OperationParams().SetAccelerationEnabled( !( ((operation_bits & 0x2) >> 1) !=0 ) );
		motPar_.OperationParams().SetOperationMode(static_cast<DESOperationModes> ((operation_bits & 0x4) >> 2) );
		motPar_.OperationParams().SetMonitoringSignalType(static_cast<MonitorSignalType> ((operation_bits & 0x10) >> 4) );
		motPar_.OperationParams().SetStopMotorBySoftware( ((operation_bits & 0x80) >> 7) !=0 );
		motPar_.OperationParams().SetMaxSpeedBySoftware( ((operation_bits & 0x100) >> 8) !=0 );
		motPar_.OperationParams().SetOffsetBySoftware( ((operation_bits & 0x200) >> 9) !=0 );
		motPar_.OperationParams().SetMaxCurrentBySoftware( ((operation_bits & 0x400) >> 10) !=0 );
		motPar_.OperationParams().SetRegGainsBySoftware( ((operation_bits & 0x800) >> 11) !=0 );
		motPar_.OperationParams().SetEnableSystemBySoftware( ((operation_bits & 0x1000) >> 12) !=0 );
		motPar_.OperationParams().SetMonitoringSignalBySoftware( ((operation_bits & 0x2000) >> 13) !=0 );
		motPar_.OperationParams().SetRegulationModeBySoftware( ((operation_bits & 0x8000) >> 15) !=0 );
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::UpdateOperationParWrite( void )
{
	bool bResult = true;
	ushort operation_bits = 0;

	//! Disable DES at first to make possible to change params
	if (!enableUpdate_(DISABLE_STATE)) return false;

	operation_bits = ((((((((((((((((static_cast<int> (motPar_.OperationParams().GetCurrSpeedSettingByAnalogueInput())) |
					 (static_cast<int> (!motPar_.OperationParams().GetAccelerationEnabled()) << 1) )|
					 (static_cast<int> (motPar_.OperationParams().GetOperationMode()) << 2 ) )|
					 (0 << 3) )|
					 (static_cast<int> (motPar_.OperationParams().GetMonitoringSignalType() << 4) ) )|
					 (0 << 5) )|
					 (0 << 6) )|
					 (static_cast<int> (motPar_.OperationParams().GetStopMotorBySoftware()) << 7) )|
					 (static_cast<int> (motPar_.OperationParams().GetMaxSpeedBySoftware()) << 8) )|
					 (static_cast<int> (motPar_.OperationParams().GetOffsetBySoftware()) << 9) )|
					 (static_cast<int> (motPar_.OperationParams().GetMaxCurrentBySoftware()) << 10 ) )|
					 (static_cast<int> (motPar_.OperationParams().GetRegGainsBySoftware()) << 11) )|
					 (static_cast<int> (motPar_.OperationParams().GetEnableSystemBySoftware()) << 12) )|
					 (static_cast<int> (motPar_.OperationParams().GetMonitoringSignalBySoftware()) << 13) )|
					 (0 << 14) )|
					 (static_cast<int> (motPar_.OperationParams().GetRegulationModeBySoftware()) << 15));

	bResult = setTempParamUpdate_(PAR_SYS_CONFIG_NB, WORD_DATA, operation_bits);
	if (bResult)
	{
		bResult = saveTempParamUpdate_(DUMMY);
	}

	return bResult;
}

//================================================================================================================//

bool MotorDES::UpdateRegParRead( void )
{
	ushort Gain;

	if (!getCurrentRegPGainUpdate_(Gain)) return false;
	motPar_.RegParams().SetCurrRegGainP(Gain);

	if (!getCurrentRegIGainUpdate_(Gain)) return false;
	motPar_.RegParams().SetCurrRegGainI(Gain);

	if (!getVelocityRegPGainUpdate_(Gain)) return false;
	motPar_.RegParams().SetVelRegGainP(Gain);

	if (!getVelocityRegIGainUpdate_(Gain)) return false;
	motPar_.RegParams().SetVelRegGainI(Gain);

	return true;
}

//================================================================================================================//

bool MotorDES::UpdateRegParWrite( void )
{
	//! Disable DES at first to make possible to change params
	if (!enableUpdate_(DISABLE_STATE)) return false;

	if (!setCurrentRegPGainUpdate_(motPar_.RegParams().GetCurrRegGainP())) return false;
	if (!setCurrentRegIGainUpdate_(motPar_.RegParams().GetCurrRegGainI())) return false;
	if (!setVelocityRegPGainUpdate_(motPar_.RegParams().GetVelRegGainP())) return false;
	if (!setVelocityRegIGainUpdate_(motPar_.RegParams().GetVelRegGainI())) return false;

	return true;
}

//================================================================================================================//

bool MotorDES::UpdateMotorParRead( void )
{
	ushort Param;

	if (!getPolePairsUpdate_(Param)) return false;
	motPar_.SetNumberPolePairs(Param);

	if (!getMaximalSpeedUpdate_(Param)) return false;
	motPar_.SetMaximalSpeed(Param);

	if (!getMotorAccelerationUpdate_(Param)) return false;
	motPar_.SetMotorAcceleration(Param);

	if (!getPeakCurrentUpdate_(Param)) return false;
	motPar_.SetMotorPeakCurrent(Param);

	if (!getNominalCurrentUpdate_(Param)) return false;
	motPar_.SetMaximalCurrent(Param);

	if (!getEncoderResolutionUpdate_(Param)) return false;
	motPar_.SetEncoderResolution(Param);

	return true;
}

//================================================================================================================//

bool MotorDES::UpdateMotorParWrite( void )
{
	//! Disable DES at first to make possible to change params
	if (!enableUpdate_(DISABLE_STATE)) return false;

	if (!setPolePairsUpdate_(motPar_.GetNumberPolePairs())) return false;
	if (!setMaximalSpeedUpdate_(motPar_.GetMaximalSpeed())) return false;
	if (!setMotorAccelerationUpdate_(motPar_.GetMotorAcceleration())) return false;
	if (!setPeakCurrentUpdate_(motPar_.GetMotorPeakCurrent())) return false;
	if (!setNominalCurrentUpdate_((ushort) motPar_.GetMaximalCurrent())) return false;
	if (!setEncoderResolutionUpdate_(motPar_.GetEncoderResolution())) return false;
	if (!setMaxSpeedCurrentModeUpdate_(motPar_.GetMaxVelocityCurrentMode())) return false;

	return true;
}

//================================================================================================================//

/* virtual */
bool MotorDES::Connect( void )
{
	motorConnected_ = false;

	if (pComm_->Open())
	{
		//! Reset system at first
		if (!resetUpdate_(DUMMY)) return false;

		//! Reset also CAN communication
		if (!resetCANUpdate_(DUMMY)) return false;

		//! Reset any possible errors on startup
		if (!clearErrorUpdate_(DUMMY)) return false;

		//! Reset CAN error
		if (!resetCANErrorUpdate_(DUMMY)) return false;

 		//! Write all parameters to DES device
		if (!UpdateOperationParWrite()) return false;
		if (!UpdateRegParWrite()) return false;
		if (!UpdateMotorParWrite()) return false;

		motorConnected_ = true;
	}

	delay_(DELAY_TIME_CONNECT_DES);
	
	return true;
}

//================================================================================================================//

/* virtual */
bool MotorDES::Disconnect( void )
{
	motorConnected_ = false;
	
	if (!DISABLE_DES()) return false;

	return pComm_->Close();
}

//================================================================================================================//

/* virtual */
bool MotorDES::UpdateRead( void )
{
	if (!motorConnected_) return false;

	//! Reset any possible errors
	clearErrorUpdate_(DUMMY);

	//! Reset CAN error
	if (!resetCANErrorUpdate_(DUMMY)) return false;

	//! Read motor current, velocity and position
	if (!(UpdateReadMeasuredData())) return false;

	return true;
}

//================================================================================================================//

/* virtual */
bool MotorDES::UpdateWrite( void )
{
	if (!motorConnected_) return false;

	//! Reset any possible errors
	clearErrorUpdate_(DUMMY);

	//! Reset CAN error
	if (!resetCANErrorUpdate_(DUMMY)) return false;

	//! Set new references for velocity and position
	if (!(UpdateSetReferences())) return false;

	return true;
}

//=============================================================================

bool MotorDES::UpdateReadMeasuredData( void )
{
	short Param1 = 0, Param2 = 0;
	
	if (!motorConnected_) return false;

	switch (motPar_.OperationParams().GetMonitoringSignalType())
	{
	case SPEED_SIGNAL:
		readVelocityIsMustUpdate_( MEAN_VALUES, Param1, Param2 );
		actualVelocity_ = Param1;
		break;

	case TORQUE_SIGNAL:
		readCurrentIsMustUpdate_( MEAN_VALUES, Param1, Param2 );
		actualCurrent_ = Param1;
		break;
	}

	getEncoderCounterUpdate_( Param1 );
	actualPosition_ = Param1;

	return true;
}

//=============================================================================

bool MotorDES::UpdateSetReferences( void )
{
	if (!motorConnected_) return false;

	//! Enable DES to make possible any motions
	if (!enableUpdate_(ENABLE_STATE)) return false;

	switch (motPar_.OperationParams().GetOperationMode())
	{
		case DES_VELOCITY_MODE:
			if (!setVelocityUpdate_(velocityRef_)) return false;
			break;
			
		case DES_CURRENT_MODE:
			if (!setCurrentUpdate_(currentRef_)) return false;
			break;
	}
	
	return true;
}

//=============================================================================

bool MotorDES::QUICKSTOP(void)
{
	if (!motorConnected_) return false;

	velocityRef_ = 0;

	return stopMotionUpdate_(DUMMY);
}

//=============================================================================

bool MotorDES::HALT(void)
{
	if (!motorConnected_) return false;

	velocityRef_ = 0;
	
	return setVelocityUpdate_(velocityRef_);
}

//=============================================================================

bool MotorDES::IS_TARGET_REACHED(void)
{
	short Param1 = 0, Param2 = 0;

	if (!motorConnected_) return false;

	switch (motPar_.OperationParams().GetOperationMode())
	{
		case DES_VELOCITY_MODE:
			readVelocityIsMustUpdate_( MEAN_VALUES, Param1, Param2 );
			actualVelocity_ = Param1;
			
			if ( (abs(actualVelocity_ - velocityRef_)) <= REACHED_TARGET_VEL_TOL )
				return true;
			else 
				return false;

			break;
			
		case DES_CURRENT_MODE:
			readCurrentIsMustUpdate_( MEAN_VALUES, Param1, Param2 );
			actualCurrent_ = Param1;

			if ( (abs(actualCurrent_ - currentRef_)) <= REACHED_TARGET_CURR_TOL )
				return true;
			else 
				return false;

			break;
	}
	return false;
}

//=============================================================================

bool MotorDES::DISABLE_DES(void)
{
	// Quickstop DES
	QUICKSTOP();

	//! Disable DES
	if (!enableUpdate_(DISABLE_STATE)) return false;
	
	delay_(DELAY_TIME_OTHER_DES);
	
	return true;
}

//=============================================================================

MotorStatus MotorDES::Status( void )
{
	ushort Error, Status;

	readErrorUpdate_(DUMMY, Error);

	if ( ((Error & 0x8000) >> 15) !=0 )
	{
		if ( ((Error & 0x2000) >> 13) !=0 ) _ErrorCode_ = DES_PARAM_OUT_RANGE;
		if ( ((Error & 0x800) >> 11) !=0 ) _ErrorCode_ = DES_OVER_TEMPERATURE;
		if ( ((Error & 0x100) >> 8) !=0 ) _ErrorCode_ = DES_ANGLE_DETECTION;
		if ( ((Error & 0x80) >> 7) !=0 ) _ErrorCode_ = DES_LOW_VOLTAGE;
		if ( ((Error & 0x40) >> 6) !=0 ) _ErrorCode_ = DES_SPEED_ERROR;
		if ( ((Error & 0x20) >> 5) !=0 ) _ErrorCode_ = DES_OVER_VOLTAGE;
		if ( ((Error & 0x10) >> 4) !=0 ) _ErrorCode_ = DES_OVER_CURRENT;
		if ( ((Error & 0x8) >> 3) !=0 ) _ErrorCode_ = DES_HALL_SENSOR_3_ERROR;
		if ( ((Error & 0x4) >> 2) !=0 ) _ErrorCode_ = DES_WRONG_ENC_RESOLUTION;
		if ( ((Error & 0x2) >> 1) !=0 ) _ErrorCode_ = DES_INDX_PROCESSING_ERROR;
		if ( ((Error & 0x1) ) !=0 ) _ErrorCode_ = DES_HALL_ERROR;
	}

	readCANErrorUpdate_(DUMMY, Error);

	if ( ((Error & 0x8000) >> 15) !=0 )
	{
		if ( ((Error & 0x4000) >> 14) !=0 ) _ErrorCode_ = DES_CAN_RXSDO_MSG_LOST;
		if ( ((Error & 0x2000) >> 13) !=0 ) _ErrorCode_ = DES_CAN_RXPDO_MSG_LOST;
		if ( ((Error & 0x1000) >> 12) !=0 ) _ErrorCode_ = DES_CAN_TXSDO_ABORT_ACK;
		if ( ((Error & 0x800) >> 11) !=0 ) _ErrorCode_ = DES_CAN_TXPDO_ABORT_ACK;
		if ( ((Error & 0x400) >> 10) !=0 ) _ErrorCode_ = DES_CAN_PDO_OVERFLOW;
		if ( ((Error & 0x200) >> 9) !=0 ) _ErrorCode_ = DES_CAN_PDO_FREQ_TOO_HIGH;
		if ( ((Error & 0x100) >> 8) !=0 ) _ErrorCode_ = DES_CAN_FORM_ERROR;
		if ( ((Error & 0x80) >> 7) !=0 ) _ErrorCode_ = DES_CAN_BIT_FLAG_ERROR;
		if ( ((Error & 0x40) >> 6) !=0 ) _ErrorCode_ = DES_CAN_STUCK_ERROR;
		if ( ((Error & 0x20) >> 5) !=0 ) _ErrorCode_ = DES_CAN_CRC_ERROR;
		if ( ((Error & 0x10) >> 4) !=0 ) _ErrorCode_ = DES_CAN_STUFF_ERROR;
		if ( ((Error & 0x8) >> 3) !=0 ) _ErrorCode_ = DES_CAN_ACKNOWLEDGE_ERROR;
		if ( ((Error & 0x4) >> 2) !=0 ) _ErrorCode_ = DES_CAN_BUS_OFF;
		if ( ((Error & 0x2) >> 1) !=0 ) _ErrorCode_ = DES_CAN_PASSIVE_STATUS_ERROR;
		if ( ((Error & 0x1) ) !=0 ) _ErrorCode_ = DES_CAN_WARNING;
	}

	_status_.Message(GetErrorDescription());

	if (_ErrorCode_ != DES_ERROR_NONE)
	{
		_status_.Id(MotorStatus::ERROR);
		return _status_;
	}
	
	readSysStatusUpdate_(DUMMY, Status);

	if ( ((Status & 0x400) >> 10) !=0 )
		_status_.Id(MotorStatus::ENABLED);
	else
		_status_.Id(MotorStatus::DISABLED);
	
	return _status_;
}

}