/*!	\file CommPrint.cpp

 */

#include "CommPrint.hpp"

namespace Vatroslav
{

//=============================================================================

std::ostream& operator<<( std::ostream& os, const CommPar& par )
{
	os << "Interface type: ";
	switch ( par.Interface() )
	{
		case CommPar::CAN:
			os << "CAN ";
			break;
		case CommPar::RS232:
			os << "RS232 ";
			break;
	}
		
	os << "\nBaudrate: " << par.Baudrate() << "\nPort: " << par.Port();
	
	os << "\nCan driver: ";
	switch ( par.CanDriver() )
	{
		case CommPar::NONE:
			os << "NONE";
			break;
		case CommPar::IXXAT:
			os << "IXXAT";
			break;
		case CommPar::UNO:
			os << "UNO";
			break;
		case CommPar::USB_WIN:
			os<< "USB_WIN";
			break;
	}

	os	<< "\nStart bits: " << par.StartBits() << "\nData bits: " << par.DataBits()
		<< "\nParity: " << par.Parity() << "\nStop bits: " << par.StopBits();

	return os;
}

//=============================================================================

std::ostream& operator<<( std::ostream& os, const CommMsg& msg )
{
	os << "Id: " << std::hex << std::setfill('0') << msg.Id() << " " 
	   << " Data: ";
	
	//! \todo More efficient implementation here ?
	for ( size_t i = 0; i < msg.Size(); i++ )
	{
		std::ostringstream buffer;
		buffer << std::hex << std::setfill('0') << std::setw(4)
			   << static_cast<unsigned short>( *( msg.Data() + i ) );
		std::string tmp( buffer.str(), 2, 2 );
		os << tmp;
	}
	os << " Size: " << msg.Size() << "bytes"
		<< " Timestamp: " << msg.Timestamp();

	return os;
}

}
