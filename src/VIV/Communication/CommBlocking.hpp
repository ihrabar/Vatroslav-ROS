/*!	\file	CommBlocking.hpp
	\brief	Blocking communication class definition.

 */

#ifndef VATROSLAV_COMM_BLOCKING_HPP
#define VATROSLAV_COMM_BLOCKING_HPP

#include "Communication.hpp"
#include "CommImpl.hpp"

namespace Vatroslav
{

//! Blocking communication implementation.
/*!

 */
class CommBlocking : public Communication
{
 public:
	 
	 //! Constructor
	 /*!

	  */
	 CommBlocking( const CommPar& par ) : param_( par ), 
		 impl_( CommImpl::Create( par ) )
	 {
		
	 }

	 //! Open the communication channel.
	 /*!

	  */
	 virtual bool Open( void ) 
	 { 
		 return impl_.Open( param_ ); 	
	 }

	//! Close the communication channel.
	/*!

	 */
	 virtual bool Close( void ) 
	 { 
		 return impl_.Close( ); 
	 }

	//! Send data.
	/*!
		@param msg	Message to send.
	 */
	virtual bool Send( const CommMsg& msg ) { return impl_.Send( msg ); }

	//! Receive data.
	/*!
		@param timeout	Amount of time to wait for the incoming message, in 
						usec (?)
		@return The received message.
	 */
	virtual bool Receive( CommMsg& msg, unsigned short timeout )
	{
		return impl_.Receive( msg, timeout );
	}

	//! Get communication parameters.
	/*!

	 */
	virtual const CommPar& Params( void )
	{
		return impl_.Params( );
	}

 private:

	// Reference to the acutal communication channel.
	CommImpl& impl_;
	// Communication channel parameters.
	CommPar param_;

};

}

#endif
