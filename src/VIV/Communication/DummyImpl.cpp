/*!	\file	DummyImpl.cpp
	\brief	Dummy implementation member definitions.

 */

#include "DummyImpl.hpp"
#include "CommPrint.hpp"

namespace Vatroslav
{

//===============================================================================

/* virtual */
bool DummyImpl::Open( const CommPar& par )
{
	std::cout << "Failed to create communication channel\n" << par
		<< "\ncreated a dummy implementation instead " << std::endl;

	return false;
}

//-----------------------------------------------------------------------------

/* virtual */
bool DummyImpl::Close( )
{
	std::cout << "Closing the dummy implemntation." << std::endl;

	return false;
}

//-----------------------------------------------------------------------------

/* virtual */
bool DummyImpl::Send( const CommMsg& msg )
{
	std::cout << "Dummy-Sending: " << msg << std::endl;

	return false;
}

//-----------------------------------------------------------------------------

/* virtual */
bool DummyImpl::Receive( CommMsg& msg, unsigned short timeout )
{
	std::cout << "Dummy-Receiving: " << msg << std::endl;

	return false;
}

//===============================================================================

}
