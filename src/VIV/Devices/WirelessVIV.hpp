/*! \file	WirelessVIV.hpp
	\brief	Wireless communication over RS232-HAC TELECOM RF MODULES

 */

#ifndef VIV_Wireless_HPP
#define VIV_Wireless_HPP
#include "Kinematics.hpp"

#include <cassert>
#include <iostream>
#include "boost/date_time/posix_time/posix_time_types.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/local_time/local_time.hpp"
#include "boost/asio.hpp"
#include "boost/shared_ptr.hpp"
#include <boost/crc.hpp>  // for boost::crc_32_type

#include "../Communication/TimeoutSerial.h"

//#include "VTKsceneConfigure.h" // Include configuration header.


typedef unsigned char BYTE;

namespace Vatroslav{

class WirelessMessage{
public:
	//! Default constructor
	/*!
	*/
	WirelessMessage(){
		n_=0;
	}
	//! Get message
	/*!
	*/
	int GetMessage(unsigned char ** data);
	//! Set message
	/*!
	*/
	void SetMessage(unsigned char *data,int n);
private:
	unsigned char data_[20];
	int n_;
};

//! Serial communication implementation for all OS platforms.
/*!

*/
//class WirelessVIV
typedef boost::shared_ptr<Sensors> SensorsPtr;
typedef boost::shared_ptr<Kinematics> KinematicsPtr;
typedef boost::shared_ptr<Power> PowerPtr;
typedef boost::shared_ptr<RotateCamera> RotateCameraPtr;

class WirelessVIV 
{

 public:

	//! Constructor
	 WirelessVIV(KinematicsPtr kin,SensorsPtr sen,PowerPtr pow,RotateCameraPtr rot):kin_(kin),
											sen_(sen),
											pow_(pow),
											rot_(rot)
	 {
		InBuff[0]=0;
	 }

	//! Destructor
	~WirelessVIV( );

	//! Open function...
	/*!
		This function opens serial communication port defined
		with string name and sets its handle.
		
		@param par	Communication parameters. Currently just a placeholder.
	 */
	bool Open(std::string port );

	//! Close function...
	/*!

	 */
	bool Close( void );

	/*Calculate 8bit CRC*/
	unsigned char UtilCalcCRC8(const unsigned char* pbyBuffer, unsigned int nLength);

	//! Send message over rs232
	/*!

	 */
	bool SendMessage();

	//! read input buffer, if true then message is ready
	bool ReceiveMessage(int timeout);
	//! Empty buffer
	int EmptyBuffer();


 private:
	//parsing
	bool Parser();
	
	bool Receive();

	 //! input buffer
	 std::string InBuff;
	 //! Whether the channel is open or not.
	 bool open_;
	 //! Communication parameters.
	  TimeoutSerial *serial;

	  KinematicsPtr kin_;
	  SensorsPtr sen_;
	PowerPtr pow_;
	RotateCameraPtr rot_;


};

}

#endif //SEGWAY_SERIAL_BOOST_HPP
