/*!	\file	Flipper.hpp
	\brief	Flipper class declaration.

 */

#ifndef VATROSLAV_FLIPPER_HPP
#define VATROSLAV_FLIPPER_HPP
#define MAX_SPEED_DRIVE 8000
#define MAX_SPEED_FLIP 1000

#include <memory>
#include <cassert>

#include <boost/shared_ptr.hpp>

#include "../Communication/Communication.hpp"
#include "MotorEPOS.hpp"
#include "LinAct.hpp"
#include "Sensor.hpp"
namespace Vatroslav
{

typedef boost::shared_ptr<MotorEPOS> MotorPtr;
typedef boost::shared_ptr<LinAct> LinActPtr;
typedef boost::shared_ptr<AngleSensor> AngleSensorPtr;

//! Flipper parameters.
/*! Class for holding flipper parameters.

 */
class FlipperPar
{
 public:

	//! Constructor
	/*!
		@param p_motor	Flipper motor.
		@param p_solenoid Flipper solenoid.
		@param smallR	Small cogwheel radius, in m
		@param largeR	Large cogwheel radius, in m
	 */
	FlipperPar( double smallR,
				double largeR
				)
		: smallR_( smallR ), largeR_( largeR_ )
	{
	
	}

	//! Get track radius.
	/*! Distance between small cogwheel axis and track surface.
		
		@return Small cogwheel radius, in [m].
	 */
	double SmallR( void ) const
	{
		return smallR_;
	}

	//! Get track radius
	/*! Distance between small cogwheel axis and track surface.

		@return Large cogwheel radius, in [m].
	 */
	double LargeR( void ) const
	{
		return largeR_;
	}

 private:

	//! Track parameters.
	double smallR_;
	double largeR_;
};

//! The Flipper class.
/*! Class for controlling one Flipper of the robot.

 */
class Flipper : public Device
{
 public:
	
	//! Flipper states.
	/*! The states have the following meaning:
			STOP:	w_mot = 0, solenoid = off
			DRIVE:	w_mot = v, solenoid = off
			FLIP:	w_mot = w, solenoid = off

	 */
	enum StateType { ON,OFF,CHANGING,STOP };
	enum Mode {DRIVE,FLIP};
	//! Constructor.
	/*!

	 */
	Flipper( MotorPtr p_motor,
			 LinActPtr p_linact,
			 const FlipperPar& par ,
			 AngleSensorPtr p_angle)
		: p_motor_( p_motor ), 
		  p_linact_( p_linact ),
		  pars_( par ),
		  v_ref_( 0 ),
		  flag_(0),
		  state_( STOP ),
		  p_angle_(p_angle)
	{
		motormod_=0;
	}

	//! Destructor.
	/*!

	 */
	~Flipper()
	{

	}

	//! Connect to Device.
	/*!

	 */
	virtual bool Connect( void );

	//! Disconnect from Device.
	/*!
		
	 */
	virtual bool Disconnect( void );

	//! Update device data.
	/*!

	 */
	virtual bool UpdateRead( void );
	//! Update device with data.
	/*!

	 */
	virtual bool UpdateWrite( void );


	//! Give translational velocity setpoint.
	/*! The translational velocity at which the Flipper track is moving.
		
		@param v	desired translational velocity, in m/s

		Setting the translational velocity setpoint automatically resets the
		rotational velocity setpoint to w_ref = 0;
	 */
	void VelRef( int v );
	//! Read the translational velocity setpoint.
	/*!

	 */
	double VelRef( void )
	{
		return v_ref_;
	}
	//! Quick stop the drive
	/*!

	 */
	bool QuickStop(){
		p_motor_->QUICKSTOP();
	}

	//! Give rotational velocity setpoint.
	/*! The rotational velocity of the flipper.
		
		@param w	desired rotational velocity, in rad/s

		Setting the rotational velocity setpoint automatically resets the
		translational velocity setpoint to v_ref = 0;
	 */


	//! Read Flipper state.
	/*! The state is updated on each call to Flipper::Update, based on current
		motor and solenoid states.
	 */
	StateType State( void )
	{
		return state_;
	}

	//! Access the motor.
	/*!

	 */
	MotorPtr GetMotor( void )
	{
		return p_motor_;
	}

	//! Access the solenoid.
	/*!

	 */
	LinActPtr GetLinAct( void )
	{
		return p_linact_;
	}

	//! Change mode 
	/*!

	*/
	void ChangeMode(Mode m);
	//! Return data to send over wireless
	/*!
	*/
	int WirelessData(char* data);

	//! Return data to send over wireless
	/*!
	*/
	bool SetAngle(int angle);	
 private:

	 //! Forbid copying and assigning Flipper objects.
	 Flipper( const Flipper& flipper );
	 Flipper& operator=( const Flipper& flipper);

	 //! Motor
	 MotorPtr p_motor_;
	 //! Solenoid
	 LinActPtr p_linact_;
	 
	 AngleSensorPtr p_angle_;

	 //! Flipper parameters.
	 FlipperPar pars_;

	 //! Translational velocity setpoint.
	 int v_ref_;
	 int flag_;
	 int motormod_;
	 int angle_;
	 boost::posix_time::ptime time_;
	 //! Current flipper state.
	 StateType state_;
	 Mode m_;
};

typedef boost::shared_ptr<Flipper> FlipperPtr;

}

#endif
