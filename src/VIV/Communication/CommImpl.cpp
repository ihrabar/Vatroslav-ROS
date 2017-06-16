/*!	\file	CommImpl.cpp
	\brief	General implementations.

 */

#include <iostream>

#define WIN32_LEAN_AND_MEAN

#include "CommImpl.hpp"
#include "DummyImpl.hpp"
#include "SerialBoost.hpp"
#include "CanIxxat.hpp"
#include "CanAdv.hpp"
//#include "CanUsb.hpp"
#include "SerialWin.hpp"



namespace Vatroslav
{

//-----------------------------------------------------------------------------

/* static */
CommImpl& CommImpl::Create( const CommPar& par )
{
	switch ( par.Interface() )
	{
		case CommPar::CAN:
			switch ( par.CanDriver() )
			{
#ifdef VATROSLAV_IXXAT_WIN
				case CommPar::IXXAT:
					return CanIxxatSingleton::Instance();
					break;
#endif
#ifdef VATROSLAV_CANUSB_WIN
				case CommPar::USB_WIN:
					return CanUsbSingleton::Instance();
					break;
#endif
#ifdef VATROSLAV_UNO
				case CommPar::UNO:
					return CanAdvSingleton::Instance();
					break;
#endif
				default:
					// ovaj dummy se implementira
					return DummyImplSingleton::Instance();
			}
			break;

		case CommPar::RS232:
			switch ( par.SerialDriver() )
			{
#ifdef VATROSLAV_SERIAL_WIN
				case CommPar::WIN:
					return SerialWinSingleton::Instance();
					break;
#endif
				case CommPar::BOOST:
					//return SerialBoostSingleton::Instance();
					//break;
				default:

					return DummyImplSingleton::Instance();
			}
			break;

		default:
			return DummyImplSingleton::Instance();
	}
}

//-----------------------------------------------------------------------------

}

