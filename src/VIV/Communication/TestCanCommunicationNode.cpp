//#ifdef VATROSLAV_UNO

#include "ros/ros.h"
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include "CommMsg.hpp"

#include <cassert>
#include <cstring>	// for memecpy
#include <algorithm>
#include <iostream>
#include "CanAdv.hpp"
#include <errno.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include "Communication.hpp"
#include "SerialBoost.hpp"
#include "../Devices/WirelessVIV.hpp"
#include <unistd.h>
#include <string.h>

#include <vatroslav/CanMsg.h>
//#undef min	// remove min macro 

//using namespace boost;


// OVO JE CAN NODE
// TU CE SE SLATI NA CAN SABIRNICU I PRIMLJENE PORUKE PUBLISHATI SVIM OSTALIM NODEOVIMA
//DODATI MAIN U KOJEM CE SE INICIJALIYIRATI KOMUNIKAICJA
// SVI OSTLAI NODEOVI CE IMATI LISTENER CALL BACK KOJI SE PRIMATI CAN PORUKU I PRIHVACATI JE AKO JE YA NJEGA INACE ODBACITI

Vatroslav::CommPtr* p_comm12;
ros::Publisher can_received1;
// global ROS publisher handles

// %Tag(CALLBACK)%



void canCallback1(const vatroslav::CanMsg& por)
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
	
	ROS_INFO("canTopicPublisher poslao %s", por.data);
	//(*p_comm12)->Send(result);

  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}
// %EndTag(CALLBACK)%

int publishCAN1(const Vatroslav::CommMsg& por1){

//pretvaranje u oblik pogodan za slanje	
	vatroslav::CanMsg por2;
	const boost::posix_time::ptime temp(por1.Timestamp());

	std::string temp2 = boost::posix_time::to_iso_string(temp);

	por2.id = por1.Id();
	//memcpy(&por2.data[0], por1.Data(), por1.Size());
	por2.data = std::string ( por1.Data(), por1.Size() );
	por2.size = por1.Size();
	por2.time = temp2;
	//por2.time = std::string ( por1.Timestamp(), 4 );
	//memcpy(&por2.time, por1.Timestamp(), 4);//
	//memcpy(&por2.time,por1.Data(), 32);//32bita jer je u CanMsg definirano int32 kao prostor za zapis vremena

  	can_received1.publish(por2);
	return 0;
}




int main( int argc, char* argv[] )
{	
	
	ros::init(argc, argv, "TestCanCommunicationNode");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);

	can_received1 = n.advertise<vatroslav::CanMsg>("sendCAN", 1000);
	
	//Vatroslav::CommPar par( Vatroslav::CommPar::UNO,125000,"can1" );
	//Vatroslav::CommPtr p_comm( Vatroslav::Communication::Create( Vatroslav::Communication::BLOCKING, par ) );
		
	//p_comm->Open();
	//p_comm12 = &p_comm;
	
	/*Pero mojPero(p_comm);*/

	char data[] = { 3,2,3,4,5,6,7,8};
	Vatroslav::CommMsg msg( 1, data, 8, boost::posix_time::microsec_clock::local_time() );


	publishCAN1(msg);

	// %Tag(SUBSCRIBER)%
  	ros::Subscriber sub = n.subscribe("reciveCAN", 1000, canCallback1);
	// %EndTag(SUBSCRIBER)%

	ros::spin();
	return 0;
	
}

