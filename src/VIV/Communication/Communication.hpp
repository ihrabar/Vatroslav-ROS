/*! \file	Communication.hpp
	\brief	Communication interfaces

 */

#ifndef VATROSLAV_COMMUNICATION_HPP
#define VATROSLAV_COMMUNICATION_HPP

#include "boost/shared_ptr.hpp"

#include "CommMsg.hpp"
#include "CommPar.hpp"

namespace Vatroslav
{

// Forward-declare so that we can declare the shared_ptr.
class Communication;

typedef boost::shared_ptr<Communication> CommPtr;

//! Communication interface.
/*!

 */
class Communication
{
 public:

	 //! Supported communication types.
	 enum CommType { BLOCKING };

	 //! Open the communication channel.
	 /*!

	  */
	 virtual bool Open( void ) = 0;

	//! Close the communication channel.
	/*!

	 */
	virtual bool Close( void ) = 0;

	//! Send data.
	/*!
		@param msg	Message to send.
	 */
	virtual bool Send( const CommMsg& msg ) = 0;

	//! Receive data.
	/*!
		@param timeout	Amount of time to wait for the incoming message, in 
						usec (?)
		@return The received message.
	 */
	virtual bool Receive( CommMsg& msg, unsigned short timeout ) = 0;

	//! Get communication parameters.
	/*!

	 */
	virtual const CommPar& Params( void ) = 0;

	//! Create the appropriate communication object.
	/*!
		Creates a new communication object from the supplied parameters.

		@return Shared pointer to a newly-created communication object.
				
	 */
	static CommPtr Create( CommType type, const CommPar& par );
};

} // Vatroslav

#endif
