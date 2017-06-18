/*! \file	Device.hpp
	\brief	Abstract device interface.
	Remote edit test. Just remove this line!
	Test...

 */

#ifndef VATROSLAV_DEVICE_HPP
#define	VATROSLAV_DEVICE_HPP

#include <boost/shared_ptr.hpp>

namespace Vatroslav
{

//! Abstract Device base class.
/*!
	
 */
class Device
{
 public:
	
	//! Destructor.
	/*!

	 */
	virtual ~Device( void )
	{

	}

	//! Connect to Device.
	/*!

	 */
	virtual bool Connect( void ) = 0;

	//! Disconnect from Device.
	/*!
		
	 */
	virtual bool Disconnect( void ) = 0;

	//! Update device data by reading from device.
	/*!

	 */
	virtual bool UpdateRead( void ) = 0;

	//! Update device data by writing to device.
	/*!

	 */
	virtual bool UpdateWrite( void ) = 0;

};

typedef boost::shared_ptr<Device> DevicePtr;

}

#endif