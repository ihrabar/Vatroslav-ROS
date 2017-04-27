/*! \file	Can_Ixxat.cpp

 */

#ifdef VATROSLAV_IXXAT_WIN

#include <cstring>	// for memecpy
#include <algorithm>

#include "CanIxxat.hpp"
#undef min	// remove min macro 

namespace Vatroslav
{

//-----------------------------------------------------------------------------

/* static */
const size_t CanIxxat::vciMsgMax_ = 8;

//-----------------------------------------------------------------------------

CanIxxat::CanIxxat() : open_(false), par_( CommPar( CommPar::NONE ) ), 
					   hDevice_(NULL), ctrlNo_(0), hCtrl_(NULL), hChn_(NULL)
{

}

//-----------------------------------------------------------------------------

CanIxxat::~CanIxxat()
{
	if ( open_ )
	{
		this->Close();
	}
}

//-----------------------------------------------------------------------------

/* virtual */
bool CanIxxat::Open( const CommPar& par )
{
	bool success = false;

	if ( !open_ )
	{
		success = this->SelectDevice( par );
		
		if ( success )
		{
			success = this->InitSocket( par );
		}
		

		if ( success )
		{
			// First two messages on the bus are for initialization only,
			// they must be discarded...
			CANMSG msg;
			canChannelReadMessage(hChn_, 100, &msg);
			canChannelReadMessage(hChn_, 100, &msg);

			open_ = true;
			par_ = par;
		}
	}
	else
	{
		// The communication channel is already open, check parameters
		if ( par_ == par )
		{
			success = true;
		}
	}
	return success;
}

//-----------------------------------------------------------------------------

/* virtual */
bool CanIxxat::Close()
{
	if ( open_ )
	{
		canChannelClose(hChn_);
		canControlClose(hCtrl_);
		vciDeviceClose(hDevice_);
		open_ = false;
	}
	return !open_;
}

//-----------------------------------------------------------------------------

/* virtual */
bool CanIxxat::Send( const CommMsg& msg)
{
	bool success = false;
	if ( open_ )
	{
		// Fill the VCI CANMSG structure with data
		CANMSG vciMsg;
		vciMsg.dwTime = 0;
		vciMsg.dwMsgId = msg.Id();    // CAN message identifier
		vciMsg.uMsgInfo.Bytes.bType  = CAN_MSGTYPE_DATA;
		size_t msgSize = std::min( msg.Size(), vciMsgMax_ );
		vciMsg.uMsgInfo.Bytes.bFlags = CAN_MAKE_MSGFLAGS( msgSize, 0, 0, 0, 0 );
		memcpy( vciMsg.abData, msg.Data(), msgSize );

		// Write the CAN message into the transmit FIFO
		HRESULT hResult = canChannelSendMessage(hChn_, INFINITE, &vciMsg);
		if ( hResult == VCI_OK )
		{
			success = true;
		}
	}
	return success;
}

//-----------------------------------------------------------------------------

/* virtual */
bool CanIxxat::Receive(CommMsg& msg, unsigned short timeout)
{
	// Read the message from the bus
	bool success = false;
	if ( open_ )
	{
		CANMSG  vciMsg;
		HRESULT hResult = canChannelReadMessage( hChn_, timeout, &vciMsg);
		
		
		if ( hResult == VCI_OK )
		{
			// Fill CommMsg data from vciMsg
			msg.Id( vciMsg.dwMsgId );
			msg.Data( (const char*) vciMsg.abData, vciMsg.uMsgInfo.Bits.dlc );
			
			//! \todo Use CANMSG.dwTime for the timestamp (figure out the format)
			msg.Timestamp( boost::posix_time::microsec_clock::local_time() );
			
			success = true;
		}
	}
	return success;
}

//-----------------------------------------------------------------------------

bool CanIxxat::SelectDevice( const CommPar& par )
{
	bool success = false;
	HANDLE hEnum = NULL;	// enumerator handle
	VCIDEVICEINFO devInfo;  // device info

	// open the device list
	HRESULT hResult = vciEnumDeviceOpen( &hEnum );

	// retrieve information about the first device within the device list
	if ( hResult == VCI_OK )
	{
		hResult = vciEnumDeviceNext( hEnum, &devInfo );
	}

	// close the device list (no longer needed)
	vciEnumDeviceClose( hEnum );

	// open the device
	if ( hResult == VCI_OK )
	{
		hResult = vciDeviceOpen(devInfo.VciObjectId, &hDevice_);
	}

	if ( hResult == VCI_OK )
	{
		success = true;
	}
	return success;
}

//-----------------------------------------------------------------------------

bool CanIxxat::InitSocket( const CommPar& par ) 
{
	HRESULT hResult = VCI_E_INVHANDLE;
	
	// Create a message channel
	if (hDevice_ != NULL)
	{
		// Create and initialize a message channel
		hResult = canChannelOpen( hDevice_, ctrlNo_, FALSE, &hChn_ );

		// initialize the message channel
		if (hResult == VCI_OK)
		{
			UINT16 wRxFifoSize  = 1024;
			UINT16 wRxThreshold = 1;
			UINT16 wTxFifoSize  = 128;
			UINT16 wTxThreshold = 1;

			hResult = canChannelInitialize( hChn_,
									  wRxFifoSize, wRxThreshold,
									  wTxFifoSize, wTxThreshold);
		}

		// Activate the CAN channel
	    if (hResult == VCI_OK)
		{
			hResult = canChannelActivate(hChn_, TRUE);
		}

		// Open the CAN controller
	    if (hResult == VCI_OK)
		{
			// this function fails if the controller is in use
	        // by another application.
			hResult = canControlOpen(hDevice_, ctrlNo_, &hCtrl_);
		}

		// Initialize the CAN controller
		if (hResult == VCI_OK)
		{ 
			switch ( par.Baudrate() )
			{
				case 10000:
					hResult = canControlInitialize( hCtrl_, CAN_OPMODE_STANDARD,
                                 CAN_BT01_10KB);
					break;
				case 20000:
					hResult = canControlInitialize( hCtrl_, CAN_OPMODE_STANDARD,
                                 CAN_BT01_20KB);
					break;
				case 50000:
					hResult = canControlInitialize( hCtrl_, CAN_OPMODE_STANDARD,
                                 CAN_BT01_50KB);
					break;
				case 100000:
					hResult = canControlInitialize( hCtrl_, CAN_OPMODE_STANDARD,
                                 CAN_BT01_100KB);
					break;
				case 125000:
					hResult = canControlInitialize( hCtrl_, CAN_OPMODE_STANDARD,
                                 CAN_BT01_125KB);
					break;
				case 250000:
					hResult = canControlInitialize( hCtrl_, CAN_OPMODE_STANDARD,
                                 CAN_BT01_250KB);
					break;
				case 500000:
					hResult = canControlInitialize( hCtrl_, CAN_OPMODE_STANDARD,
                                 CAN_BT01_500KB);
					break;
				case 800000:
					hResult = canControlInitialize( hCtrl_, CAN_OPMODE_STANDARD,
                                 CAN_BT01_800KB);
					break;
				case 1000000:
					hResult = canControlInitialize( hCtrl_, CAN_OPMODE_STANDARD,
                                 CAN_BT01_1000KB);
					break;
				default:
					hResult = VCI_E_INVALIDARG;
			}
		}

		// Set the acceptance filter
		if (hResult == VCI_OK)
		{ 
			hResult = canControlSetAccFilter( hCtrl_, CAN_FILTER_STD,
                                     CAN_ACC_CODE_ALL, CAN_ACC_MASK_ALL);
		}

	    // Start the CAN controller
		if (hResult == VCI_OK)
		{
			hResult = canControlStart(hCtrl_, TRUE);
	    }	
	}

	if ( hResult == VCI_OK )
	{
		return true;
	}
	else
	{
		return false;
	}
}

//-----------------------------------------------------------------------------

}

#endif // VATROSLAV_IXXAT_WIN
