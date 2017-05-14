/*! \file	Can_Adv.hpp
	\brief	CAN communication implementation for the Advantech ??? CAN board.

 */
//#ifdef VATROSLAV_UNO

//#ifndef VATROSLAV_CAN_ADV_HPP
#ifndef VATROSLAV_CAN_PUBLISHER_HPP
#define VATROSLAV_CAN_PUBLISHER_HPP

#include <cassert>

#define LOKI_CLASS_LEVEL_THREADING	// Loki's threading policy
#include <loki/Singleton.h>
#include "can4linux.h"
#include "CommImpl.hpp"

namespace Vatroslav
{

//! CAN communication implmentation for the ADVANTECH ??? CAN board.
/*! 

	\note	You can't use this class directly. Instead, use it's singleton
			implementation, canTopicPublisherantech !
 */
class canTopicPublisher : public CommImpl
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

//private:

	//! Copy constructor
	canTopicPublisher( const canTopicPublisher& original );

	
	//! Constructor
	/*!
		All the construction facilities are made private, because the class is
		ment to be used only as a Singleton !
	 */
	canTopicPublisher( );



	//! Destructor
	~canTopicPublisher( );

	//! Assignment operator
	canTopicPublisher& operator=( const canTopicPublisher& rhs );

	//! Address-of operator
	canTopicPublisher* operator&( void );

	//!Sets bitrate of CAN communication

	int set_bitrate(int can_fd,int baud);

	bool open_;
	CommPar par_;
	//! Device descripotor
	int fd;
	// We need to grant friend access, in order for 
	// Loki::SingletonHolder to work.
	friend struct Loki::CreateUsingNew<canTopicPublisher>;
};

//! Define the CanADV singleton for public use.
typedef Loki::SingletonHolder<canTopicPublisher, Loki::CreateUsingNew,
							  Loki::PhoenixSingleton> canTopicPublisherSingleton;
}

//#endif	// VATROSLAV_CAN_ADV_HPP

#endif	// VATROSLAV_UNO

