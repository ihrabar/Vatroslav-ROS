/*! \file	CommPrint.hpp
	\brief	Helper classes for stream output.

 */

#ifndef VATROSLAV_COMM_PRINT_HPP
#define VATROSLAV_COMM_PRINT_HPP

#include <ostream>
#include <iomanip>
#include <sstream>

#include "boost/date_time/posix_time/posix_time.hpp"

#include "CommImpl.hpp"

namespace Vatroslav
{

//=============================================================================

std::ostream& operator<<( std::ostream& os, const CommPar& par );

//=============================================================================

std::ostream& operator<<( std::ostream& os, const CommMsg& msg );

//=============================================================================

}

#endif
