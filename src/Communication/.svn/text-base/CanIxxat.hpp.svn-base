/*! \file	Can_Ixxat.hpp
	\brief	CAN communication implementation for the IXXAT ??? CAN board.

 */

#ifdef VATROSLAV_IXXAT_WIN

#ifndef VATROSLAV_CAN_IXXAT_HPP
#define VATROSLAV_CAN_IXXAT_HPP

#include <cassert>

#include <vcinpl.h>

#define LOKI_CLASS_LEVEL_THREADING	// Loki's threading policy
#include <loki/Singleton.h>

#include "CommImpl.hpp"

namespace Vatroslav
{

//! CAN communication implmentation for the IXXAT ??? CAN board.
/*! 

	\note	You can't use this class directly. Instead, use it's singleton
			implementation, CanIxxat !
 */
class CanIxxat : public CommImpl
{
 public:

	//! Open function...
	/*!
		@param par	Communication parameters. Currently just a placeholder.
	 */
	virtual bool Open( const CommPar& par );

	//! Close function...
	/*!

	 */
	virtual bool Close( void );

	//! Send a message...
	/*!

	 */
	virtual bool Send( const CommMsg& msg );

	//! Receive a message...
	/*!

	 */
	virtual bool Receive( CommMsg& msg, unsigned short timeout );

	//! Get communication parameters.
	/*!

	 */
	virtual const CommPar& Params( void )
	{
		return par_;
	}

 private:
	
	//! Constructor
	/*!
		All the construction facilities are made private, because the class is
		ment to be used only as a Singleton !
	 */
	CanIxxat( );

	//! Copy constructor
	CanIxxat( const CanIxxat& original );

	//! Destructor
	~CanIxxat( );

	//! Assignment operator
	CanIxxat& operator=( const CanIxxat& rhs );

	//! Address-of operator
	CanIxxat* operator&( void );

	//! Select the appropriate device for CAN communication.
	/*! 
		VCI-internal stuff. Current implementation always selects the first
		device on the VCI device list.
	 */
	bool SelectDevice( const CommPar& par );
	
	//! Initialize the communication socket.
	/*!
		VCI-internal stuff.
	 */
	bool InitSocket( const CommPar& par );
	
	//! Whether the channel is open or not.
	bool open_;
	//! Communication parameters.
	CommPar par_;
	//! Device handle
	HANDLE hDevice_;
	//! Controller number, always 0
	LONG   ctrlNo_; 
	// controller handle 
	HANDLE hCtrl_;
	// channel handle
	HANDLE hChn_;    

	// Maximum size of VCI-defined CANMSG data field, in bytes.
	// Defined in cantype.h
	static const size_t vciMsgMax_;

	// We need to grant friend access, in order for 
	// Loki::SingletonHolder to work.
	friend struct Loki::CreateUsingNew<CanIxxat>;
};

//! Define the CanIxxat singleton for public use.
typedef Loki::SingletonHolder<CanIxxat, Loki::CreateUsingNew,
							  Loki::PhoenixSingleton> CanIxxatSingleton;
}

#endif	// VATROSLAV_CAN_IXXAT_HPP

#endif	// VATROSLAV_IXXAT_WIN