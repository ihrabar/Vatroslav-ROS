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

Vatroslav::CommPtr* p_comm2;
ros::Publisher can_received;
// global ROS publisher handles

// %Tag(CALLBACK)%



void canCallback(const vatroslav::CanMsg& por)
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

	(*p_comm2)->Send(result);

  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}
// %EndTag(CALLBACK)%

int publishCAN(const Vatroslav::CommMsg& por1){

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

  	can_received.publish(por2);
	return 0;
}


//int callbackCAN
//  http://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/

/*class Pero
	{
	public:
		static Vatroslav::CommPtr moj_can;
		
		Pero_(Vatroslav::CommPtr neki_can)
			{
			Pero.moj_can = neki_can;
			}		
		
		void canCallback2(const vatroslav::CanMsg& por)
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

				this->moj_can;

			  //ROS_INFO("I heard: [%s]", msg->data.c_str());
			}
		
	};*/



int main( int argc, char* argv[] )
{	
	
	ros::init(argc, argv, "canCommunicationNode");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);

	can_received = n.advertise<vatroslav::CanMsg>("reciveCAN", 1000);
	
	Vatroslav::CommPar par( Vatroslav::CommPar::UNO,125000,"can1" );
	Vatroslav::CommPtr p_comm( Vatroslav::Communication::Create( Vatroslav::Communication::BLOCKING, par ) );
		
	p_comm->Open();
	p_comm2 = &p_comm;
	
	/*Pero mojPero(p_comm);*/

	char data[] = { 3,2,3,4,5,6,7,8};
	Vatroslav::CommMsg msg( 1, data, 8, boost::posix_time::microsec_clock::local_time() );


	publishCAN(msg);

	// %Tag(SUBSCRIBER)%
  	ros::Subscriber sub = n.subscribe("sendCAN", 1000, canCallback);
	// %EndTag(SUBSCRIBER)%

	ros::spin();
	return 0;
	
}

