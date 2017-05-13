/*! \file	canTopicPublisher.cpp

 */

#ifdef VATROSLAV_UNO

#include "ros/ros.h"

#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdio.h>

#include <cstring>	// for memecpy
#include <algorithm>
#include <iostream>
#include "canTopicPublisher.hpp"
#include <errno.h>
//#undef min	// remove min macro 

/////////////////////////////////// TU PROMJENITI DA NE RADIR DIREKT NA CAN ANEGO DA DALJE NA CAN NODE


namespace Vatroslav
{
namespace pt = boost::posix_time;

//-----------------------------------------------------------------------------

canTopicPublisher::canTopicPublisher() : open_(false), par_( CommPar( CommPar::NONE ) )
{

}

//-----------------------------------------------------------------------------

canTopicPublisher::~canTopicPublisher()
{
	if ( open_ )
	{
		this->Close();
	}
}

//-----------------------------------------------------------------------------

/* virtual */


bool canTopicPublisher::Open( const CommPar& par )
{
	return true;
		
}

//-----------------------------------------------------------------------------

int canTopicPublisher::set_bitrate(int can_fd, int baud )
{
   return 0;
}

//-----------------------------------------------------------------------------

/* virtual */
bool canTopicPublisher::Close()
{
	return true;
}

//-----------------------------------------------------------------------------

/* virtual */
bool canTopicPublisher::Send( const CommMsg& por1)
{

	vatroslav::CanMsg por2;
	const boost::posix_time::ptime temp(por1.Timestamp());

	std::string temp2 = boost::posix_time::to_iso_string(temp);

	por2.id = por1.Id();
	por2.data = std::string ( por1.Data(), por1.Size() );
	por2.size = por1.Size();
	por2.time = temp2;

  	sendToCAN.publish(por2);
	return 0;

}

//-----------------------------------------------------------------------------

/* virtual */
bool canTopicPublisher::Receive(CommMsg& msg, unsigned short timeout)
{
	
	return success;
}

//-----------------------------------------------------------------------------



}

#endif // VATROSLAV_UNO

