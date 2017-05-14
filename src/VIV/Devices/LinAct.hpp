#ifndef VATROSLAV_LINACT_HPP
#define VATROSLAV_LINACT_HPP

#include "Device.hpp"
#include "../Communication/Communication.hpp"
#include "../Communication/canTopicPublisher.hpp"


namespace Vatroslav
{

class LinAct : public Device
{

public:
	//! Constructor...
	/*!
		LinAct with ID CAN identifier
	 */

	LinAct(int id, CommPtr pComm);

	//! Update device status
	/*!
	
	*/

	bool UpdateRead();

	//! Set Device status
	/*!
	
	*/

	bool UpdateWrite();

	//! Turn on LinAct
	/*!
	*/
	bool On();

	//! Turn on LinAct slow
	/*!
	*/
	bool OnSlow();

	//! Turn off LinAct
	/*!
	*/
	bool Off();

	//! Turn off LinAct slow
	/*!
	*/
	bool OffSlow();

	//! Turn off motor
	/*
	*/
	bool MotorOff();

	//! Read microcontroller constants
	/*
		Start postion, end position and maximal current
	*/

	bool ReadConstants();
	
	enum constant {start_position,end_position,max_current};

	//! Change microcontroller constants
	/*
	*/

	bool ChangeConstant(constant con, int value);

	//! Get constant value;
	/*
	*/
	int GetConstant(constant con);

	enum status_enum {off, on, motor_off, changing_state_on,changing_state_off, other};



	//! Get status

	status_enum Status();

	//! Get position;
	int GetPosition();

	//! Get position;
	int GetCurrent();


	//! Connect
	/*!
	*/

	bool Connect();

	//! Disconnect
	/*!
	*/
	bool Disconnect();

private:

	int status_,set_state,slow_;
	int current_, position_,state_,value_,start_pos_,end_pos_,max_curr_;
	int id_;
	bool open_;
	CommPtr pComm_; 
};
}
#endif
