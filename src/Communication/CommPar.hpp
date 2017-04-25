/*!	\file	CommPar.hpp
	\brief	Communication parameters class

 */

#ifndef VATROSLAV_COMM_PAR_HPP
#define VATROSLAV_COMM_PAR_HPP

#include <string>
#include <iostream>

namespace Vatroslav
{

//! Communication parameters
/*!
	RS232 and CAN parameters are bundled together. The parameters
	that don't make sense for a specific type of communication should simply
	be ignored.
	
 */
class CommPar
{
 public:
	
	// To save some typing...
	typedef unsigned short ushort;
	typedef unsigned long ulong;

	//! Supported communication interfaces.
	enum InterfaceType { RS232, CAN };
	enum CanDriverType { NONE, IXXAT, UNO,USB_WIN };
	enum SerialDriverType { NO, WIN, BOOST };

	//! RS232 constructor.
	/*!
		Use to construct RS232 communication parameters.
	 */
	CommPar( SerialDriverType driver,
			 const std::string& port,
			 ulong baudrate = 9600,
			 ushort startBits = 1, 
			 ushort dataBits = 8,
			 char parity = 0,
			 ushort stopBits = 0 )	: interface_( RS232 ),
									  baudrate_( baudrate ),
									  port_( port ),
									  startBits_( startBits ),
									  dataBits_( dataBits ),
									  parity_( parity ),
									  stopBits_( stopBits ),
									  canDriver_( NONE ),
									  serialDriver_( driver )
	{ 
		
	}

	//! Get RS232 port string
	std::string Port( void ) const { return port_; }

	//! Set RS232 port string
	void Port( std::string port ) { port_ = port; }

	//! Get number of start bits
	ushort StartBits( void ) const { return startBits_; }

	//! Set number of start bits
	void StartBits( ushort startBits ) { startBits_ = startBits; }

	//! Get number of data bits
	ushort DataBits( void ) const  { return dataBits_; }

	//! Set number of data bits
	void DataBits( ushort dataBits ) { dataBits_ = dataBits; }

	//! Get parity
	char Parity( void ) const { return parity_; }

	//! Set parity
	void Parity( char parity ) { parity_ = parity; }

	//! Get number of stop bits
	ushort StopBits( void ) const { return stopBits_; }

	//! Set number of stop bits
	void StopBits( ushort stopBits ) { stopBits_ = stopBits; }

	//! CAN constructor.
	/*!
		Use to construct CAN communication parameters.
	 */
	CommPar( CanDriverType driver = NONE,
			 ulong baudrate = 125000,
			 std::string port = "can0")	: interface_( CAN ),
										  baudrate_( baudrate ),
										  port_( port ),
										  startBits_( 0 ),
										  dataBits_( 0 ),
										  parity_( 0 ),
										  stopBits_( 0 ),
										  canDriver_( driver ),
										  serialDriver_( NO )
	{
		
	}
/*	CommPar( CanDriverType driver = NONE,
			 ulong baudrate = 125000,
			 std::string port="can0")	: interface_( CAN ),
										  baudrate_( baudrate ),
										  port_( port ),
										  startBits_( 0 ),
										  dataBits_( 0 ),
										  parity_( 0 ),
										  stopBits_( 0 ),
										  canDriver_( driver )
	{
		
	}
*/

	//! Get CAN driver type.
	CanDriverType CanDriver( void ) const { return canDriver_; }
	
	//! Set CAN driver type.
	void CanDriver( CanDriverType driver ) { canDriver_ = driver; }

	//!Get SERIAL driver type
	SerialDriverType SerialDriver( void ) const { return serialDriver_; }

	//! Set SERIAL driver type
	void SerialDriver( SerialDriverType driver ) { serialDriver_ = driver; }

	//! Get interface type
	InterfaceType Interface( void ) const { return interface_; }

	//! Get Baudrate
	/*!
		@return Baudrate in bits/second (?)
	 */
	ulong Baudrate( void ) const { return baudrate_; }

	//! Set Baudrate
	/*!
		@param baudrate in bits/second (?)
	 */
	void Baudrate( ulong baudrate ) { baudrate_ = baudrate;}


 private:

	// Type of interface.
	InterfaceType interface_;
	
	// Common parameters.
	ulong baudrate_;

	// RS232 parameters
	std::string port_;
	ushort startBits_;
	ushort dataBits_;
	char parity_;
	ushort stopBits_;

	// CAN parameters	
	CanDriverType canDriver_;
	// Serial parameters
	SerialDriverType serialDriver_;
};

bool operator==( const CommPar& lhs, const CommPar& rhs );

}

#endif
