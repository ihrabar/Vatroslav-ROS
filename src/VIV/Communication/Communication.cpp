/*! \file	Communication.cpp
	\brief	Member definitions for communication classes.

 */

#include <boost/make_shared.hpp>

#include "Communication.hpp"
#include "CommBlocking.hpp"

namespace Vatroslav
{

//=============================================================================



//=============================================================================

/* static */
CommPtr Communication::Create( CommType type, const CommPar& par )
{
	switch ( type )
	{
		case BLOCKING:
			return ( boost::make_shared<CommBlocking>( par ) );
			break;
		default:
			return ( boost::make_shared<CommBlocking>( par ) );
	}
	
}

//=============================================================================

}
