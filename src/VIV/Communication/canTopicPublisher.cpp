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
#include "CommMsg.hpp"
//#undef min	// remove min macro 

/////////////////////////////////// TU PROMJENITI DA NE RADIR DIREKT NA CAN ANEGO DA DALJE NA CAN NODE
//ros::Publisher sendToCAN;

namespace Vatroslav
{
namespace pt = boost::posix_time;

list<Vatroslav::CommMsg> msgList;	// vector for storing data from subscriber


	//sendToCAN = n.advertise<vatroslav::CanMsg>("sendCAN", 1000);

	// %Tag(SUBSCRIBER)%
	//subCAN = n.subscribe("receiveCAN", 1000, canTopicCallback);
	// %EndTag(SUBSCRIBER)%

//-----------------------------------------------------------------------------
//definiranje subscribera i liste koju on puni a ostatak programa na zahtjev prazni
int dummy()
{
return 0;
};


void canTopicCallback(const vatroslav::CanMsg& por)
{
	 char result_data[] = {0 ,0, 0, 0, 0, 0, 0, 0};
	result_data[0] = (char) por.data[0];
	result_data[1] = (char) por.data[1];
	result_data[2] = (char) por.data[2];
	result_data[3] = (char) por.data[3];
	result_data[4] = (char) por.data[4];
	result_data[5] = (char) por.data[5];
	result_data[6] = (char) por.data[6];
	result_data[7] = (char) por.data[7];
		
	Vatroslav::CommMsg result((unsigned short)1, result_data, (size_t) por.size, boost::posix_time::from_iso_string(por.time));
	
	msgList.push_back(*result);

  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/* virtual */
bool Send( const CommMsg& por1)
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
bool Receive(CommMsg& msg, unsigned short timeout)
{

	boost::posix_time::time_duration dur;
	pt::ptime t1,t2;

	t1=boost::posix_time::microsec_clock::local_time();

		while (msgList.empty()){
			t2=boost::posix_time::microsec_clock::local_time();
			dur=t2-t1;

			if (dur.total_milliseconds()>timeout){
				success=false;
				break;
			}
			if (!msgList.empty()){
				msg = msgList.pop_front();
				success=true;
				break;
			}

		}
	return success;

}

//-----------------------------------------------------------------------------



}

#endif // VATROSLAV_UNO

