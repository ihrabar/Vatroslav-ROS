/*! \file	DummyImpl.hpp
	\brief	Dummy communication channel definition.
 */

#ifndef VATROSLAV_DUMMY_IMPL_HPP
#define VATROSLAV_DUMMY_IMPL_HPP

#define LOKI_CLASS_LEVEL_THREADING	// Loki's threading policy
#include <loki/Singleton.h>

#include "CommImpl.hpp"

//=============================================================================

namespace Vatroslav
{

//! A dummy communication implementation.
/*!
	Created when the user requests an unsupported communication object.
 */
class DummyImpl : public CommImpl
{
 public:
	
	//! Open the communication.
	virtual bool Open( const CommPar& par );

	//! Close the communication.
	virtual bool Close( );

	//! Send data.
	virtual bool Send( const CommMsg& msg );

	//! Receive data.
	/*!
		@param timeout	Amount of time to wait for the incoming message, in 
						msec (?)
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

	//! Make all construction facilities private
	DummyImpl( ) : par_( CommPar( CommPar::NONE ) )
	{

	}

	DummyImpl( const DummyImpl& );
	~DummyImpl( ) { }
	DummyImpl& operator=( const DummyImpl& );
	DummyImpl* operator&( void );

	CommPar par_;

	// We need to grant friend access, in order for 
	// Loki::SingletonHolder to work.
	friend struct Loki::CreateUsingNew<DummyImpl>;
};

//! Define the DummyImpl singleton for public use.
typedef Loki::SingletonHolder<DummyImpl, Loki::CreateUsingNew,
							  Loki::PhoenixSingleton> DummyImplSingleton;

//=============================================================================

}

#endif
