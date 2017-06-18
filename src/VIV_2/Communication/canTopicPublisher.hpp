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
#include "Communication.hpp"
#include "CommMsg.hpp"

#include <vatroslav/CanMsg.h>

using namespace Vatroslav;

namespace Vatroslav {


	void canTopicCallback(const vatroslav::CanMsg& msg);

	bool Send( const CommMsg& msg );

	//! Receive a message...
	/*!

	 */
	bool Receive( CommMsg& msg, unsigned short timeout );
	
	//! Get communication parameters.
	/*!

	 */
}


//#endif	// VATROSLAV_CAN_ADV_HPP

#endif	// VATROSLAV_UNO

