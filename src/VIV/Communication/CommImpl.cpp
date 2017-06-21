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


//using namespace canROS_UNO;
namespace Vatroslav
{

//-----------------------------------------------------------------------------

/* static */
CommImpl& CommImpl::Create( const CommPar& par )
{

//#define CAN_ROS_UNO

	switch ( par.Interface() )
	{
		case Vatroslav::CommPar::CAN:
			switch ( par.CanDriver() )
			{
/*#ifdef VATROSLAV_IXXAT_WIN
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
				std::cout << "CommPar UNO workspace" << std::endl;
				case CommPar::UNO:
					return CanAdvSingleton::Instance();
					break;
#endif*/
//#ifdef CAN_ROS_UNO
  	            std::cout << "CommPar ROS_UNO workspace" << std::endl;
				case Vatroslav::CommPar::UNO:
					return CanAdvSingleton::Instance();
					break;
//#endif
				default:
					// ovaj dummy se implementira
					std::cout << "DummyImpl_1" << std::endl;
					std::cout << ((par.CanDriver() == Vatroslav::CommPar::UNO) ? "true" : "false") << std::endl;
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

