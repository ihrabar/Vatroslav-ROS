/*! \file	Timer.hpp
	\brief	Timer class.

	A simple timer, based on boost::posix_time::microsec_clock::local_time()
	
	\todo Should be checked for accuracy and resolution.
 */

#ifndef VATROSLAV_TIMER_HPP
#define VATROSLAV_TIMER_HPP

#include "boost/date_time/posix_time/posix_time_types.hpp"
#include "boost/date_time/local_time/local_time.hpp"

namespace Vatroslav
{

//!	A simple timer class.
/*! The timer is based on boost::posix_time::microsec_clock::local_time().
	Theoretically it has microsec resolution. In practice, the resolution
	depends on the platform and should be checked.

	\todo Check resolution & accuracy.
	\todo	The current implementation is good only for simple time measurement.
			Call Reset(), call Start(), run the code you want to measure, call
			Stop() and then call Time() to get the elapsed time. A better
			implementation (supporting repeated Start/Stop cycles) is comming
			soon...
			
 */
class Timer
{
 public:
	//! Constructor
	/*! 
		The timer is not running on construction.
	 */
	 Timer() : t_start_( boost::posix_time::microsec_clock::local_time() ),
			   t_stop_( boost::posix_time::microsec_clock::local_time() ),
			  running_( false )
	{
		
	}

	//! Start the timer.
	/*!

	 */
	void Start( void )
	{
		running_ = true;
	}

	//! Stop the timer.
	/*!  

	 */
	void Stop( void )
	{
		t_stop_ = boost::posix_time::microsec_clock::local_time();
		running_ = false;
	}

	//! Reset the timer
	/*! Timer state (running or not) is not changed on reset.
		
	 */
	void Reset( void )
	{
		t_start_ = boost::posix_time::microsec_clock::local_time();
		t_stop_ = t_start_;
	}

	//! Return elapsed time.
	/*! If timer is running, it returns elapsed time

	 */
	boost::posix_time::time_duration Time( void )
	{
		boost::posix_time::time_duration elapsed_time;
		if ( running_ )
		{
			elapsed_time = boost::posix_time::microsec_clock::local_time() 
				- t_start_;
		}
		else
		{
			elapsed_time = t_stop_ - t_start_;
		}
		return elapsed_time;
	}
 private:

	// Timer variables
	boost::posix_time::ptime t_start_;
	boost::posix_time::ptime t_stop_;
	bool running_;
};

}

#endif
