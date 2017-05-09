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
//#include <vatroslav/CanMsg.h>
//#undef min	// remove min macro 




// OVO JE CAN NODE
// TU CE SE SLATI NA CAN SABIRNICU I PRIMLJENE PORUKE PUBLISHATI SVIM OSTALIM NODEOVIMA
//DODATI MAIN U KOJEM CE SE INICIJALIYIRATI KOMUNIKAICJA
// SVI OSTLAI NODEOVI CE IMATI LISTENER CALL BACK KOJI SE PRIMATI CAN PORUKU I PRIHVACATI JE AKO JE YA NJEGA INACE ODBACITI


ros::Publisher can_pub;
// global ROS publisher handles


int publishCAN( Vatroslav::CommMsg por){
	
  	//can_pub.publish(por);
	return 0;
}


//int callbackCAN


int main( int argc, char* argv[] )
{

	ros::init(argc, argv, "canCommunicationNode");
	ros::NodeHandle n;

	//can_pub = n.advertise<Vatroslav::CommMsg>("publishCAN", 1000);
	
	Vatroslav::CommPar par( Vatroslav::CommPar::UNO,125000,"can1" );
	Vatroslav::CommPtr p_comm( Vatroslav::Communication::Create( Vatroslav::Communication::BLOCKING, par ) );
	p_comm->Open();

	char data[] = { 3,2,3,4,5,6,7,8};
	Vatroslav::CommMsg msg( 1, data, 8, boost::posix_time::microsec_clock::local_time() );


	publishCAN(msg);

	ros::Rate loop_rate(1);
}

