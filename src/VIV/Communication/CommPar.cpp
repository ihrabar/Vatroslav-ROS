/*!	\file	CommPar.hpp
	\brief	Communication parameter member & operator definitions.

 */

#include "CommPar.hpp"
#include <iostream>

namespace Vatroslav
{

bool operator==( const CommPar& lhs, const CommPar& rhs )
{
	bool equal = true;
	
	equal = equal && ( lhs.Interface() == rhs.Interface() );
	equal = equal && ( lhs.Baudrate() == rhs.Baudrate() );
	equal = equal && ( lhs.Port() == rhs.Port() );
	equal = equal && ( lhs.StartBits() == rhs.StartBits() );
	equal = equal && ( lhs.DataBits() == rhs.DataBits() );
	equal = equal && ( lhs.Parity() == rhs.Parity() );
	equal = equal && ( lhs.StopBits() == rhs.StopBits() );
	equal = equal && ( lhs.CanDriver() == rhs.CanDriver() );

	return equal;
}

}
