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

#include <vatroslav/CanMsg.h>
//#undef min	// remove min macro 




// OVO JE CAN NODE
// TU CE SE SLATI NA CAN SABIRNICU I PRIMLJENE PORUKE PUBLISHATI SVIM OSTALIM NODEOVIMA
//DODATI MAIN U KOJEM CE SE INICIJALIYIRATI KOMUNIKAICJA
// SVI OSTLAI NODEOVI CE IMATI LISTENER CALL BACK KOJI SE PRIMATI CAN PORUKU I PRIHVACATI JE AKO JE YA NJEGA INACE ODBACITI


ros::Publisher can_pub;
// global ROS publisher handles


int publishCAN( Vatroslav::CommMsg por1){
//pretvaranje u oblik pogodan za slanje	
	vatroslav::CanMsg por2;
	por2.id = por1.Id();
	memcpy(&por2.data,por1.Data(),por1.Size());
	por2.size = por1.Size();
	memcpy(&por2.time,por1.Data(), 32);//32 jer je u CanMsg definirano int32 kao prostor za zapis vremena

  	can_pub.publish(por2);
	return 0;
}


//int callbackCAN


int main( int argc, char* argv[] )
{

	ros::init(argc, argv, "canCommunicationNode");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);

	can_pub = n.advertise<vatroslav::CanMsg>("publishCAN", 1000);
	
	Vatroslav::CommPar par( Vatroslav::CommPar::UNO,125000,"can1" );
	Vatroslav::CommPtr p_comm( Vatroslav::Communication::Create( Vatroslav::Communication::BLOCKING, par ) );
	p_comm->Open();

	char data[] = { 3,2,3,4,5,6,7,8};
	Vatroslav::CommMsg msg( 1, data, 8, boost::posix_time::microsec_clock::local_time() );


	publishCAN(msg);

	
}

