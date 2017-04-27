/*!	\file	Motor.hpp
	\brief	Abstract motor interface.

 */

#ifndef VATROSLAV_MOTOR_HPP
#define VATROSLAV_MOTOR_HPP

#include <string>

#include "Device.hpp"

namespace Vatroslav
{

//! Motor status.
/*! Class for reporting motor status.

	\todo Figure out what else goes here...
 */
class MotorStatus
{
 public:
	
	//! Possible status flags.
	/*! 
		ENABLED		-	Drive is ready for operation.
		DISABLED	-	Drive is in standby mode.
		ERROR		-	Drive is in fault state.
	 */
	enum StatusType { ENABLED = 0, DISABLED = 1, ERROR = 2};
	
	MotorStatus( StatusType status = DISABLED,
				 const std::string& message = "" )
				 : status_( status ), message_( message )
	{

	}

	//! Get status identifier.
	/*!

	 */
	StatusType Id( void ) const
	{
		return status_;
	}

	//! Set status identifier.
	/*!

	 */
	void Id( StatusType status )
	{
		status_ = status;
	}

	//! Get status message.
	/*!

	 */
	const std::string& Message( void ) const
	{
		return message_;
	}

	//! Set status message.
	/*!

	 */
	void Message( const std::string& message )
	{
		message_ = message;
	}

protected:

	StatusType status_;
	std::string message_;

};

//=============================================================================

//! Motor interface.
/*! Basic methods that all motor devices must support.

 */
class Motor : public Device
{
 public:
	
	virtual ~Motor( void )
	{

	}

	//! Get the actual current value.
	/*! 
		@return The actual current in [A].
	 */
	virtual long Current( void ) const = 0;

	//! Get the actual velocity value.
	/*!
		@return The actual velocity in [rpm (Revolutions per Minute)].
	 */
	virtual long Velocity( void ) const = 0;

	//! Get the velocity reference.
	/*!
		@return Velocity reference in [rpm (Revolutions per Minute)].
	 */
	virtual long VelocityRef( void ) const = 0;

	//! Set the velocity reference.
	/*!
		@param vel_ref Velocity reference in [rpm (Revolutions per Minute)].
	 */
	virtual void VelocityRef( long vel_ref ) = 0;

	//! Get the actual position value.
	/*!
		@return The actual position in [steps (quadcounts = 4*Encoder Counts / Revolution)].
	 */
	virtual long Position( void ) const = 0;

	//! Get the position reference.
	/*!
		@return Position reference in [steps (quadcounts = 4*Encoder Counts / Revolution)].
	 */
	virtual long PositionRef( void ) const = 0;

	//! Set the position reference.
	/*!
		@param pos_ref Position reference in [steps (quadcounts = 4*Encoder Counts / Revolution)].
	 */
	virtual void PositionRef( long pos_ref ) = 0;


	//! Stop motor immediatly with high deceleration.
	/*! Use the Get/SetQuickStopDeceleration on the MotionPar object to get/set
		the deceleration value.

		@return Returns true if this operation succeed
	 */
	virtual bool QUICKSTOP( void ) = 0;

	//! Bring motor gradually to a stop.
	/*! The deceleration can be specified through the 
		Get/SetProfileDeceleration methods from the MotionPar object.

		@return Returns true if this operation succeed
	 */
	virtual bool HALT( void ) = 0;

	//! Has the motion target been reached.
	/*!
		@return Returns true if the actual motion value is 
				"close enough" to its setpoint.

		The motion value that is checked depends on the current operation
		mode. It is the velocity in "Profile velocity mode" or the position
		in "Profile position mode".
	 */
	virtual bool IS_TARGET_REACHED( void ) = 0;

	//! Connect to Motor.
	/*!

	 */
	virtual bool Connect( void ) = 0;

	//! Disconnect from Motor.
	/*!
		
	 */
	virtual bool Disconnect( void ) = 0;

	//! Update motor data by reading from sensors.
	/*!

	 */
	virtual bool UpdateRead( void ) = 0;

	//! Update motor by writing setpoints.
	/*!

	 */
	virtual bool UpdateWrite( void ) = 0;

	//! Get status information.
	/*!

	 */
	virtual MotorStatus Status( void ) = 0;
};

}

#endif