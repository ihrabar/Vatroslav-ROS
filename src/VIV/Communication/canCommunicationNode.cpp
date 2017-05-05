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
#include "CanAdv.hpp"
#include <errno.h>
//#undef min	// remove min macro 




// OVO JE CAN NODE
// TU CE SE SLATI NA CAN SABIRNICU I PRIMLJENE PORUKE PUBLISHATI SVIM OSTALIM NODEOVIMA
//DODATI MAIN U KOJEM CE SE INICIJALIYIRATI KOMUNIKAICJA
// SVI OSTLAI NODEOVI CE IMATI LISTENER CALL BACK KOJI SE PRIMATI CAN PORUKU I PRIHVACATI JE AKO JE YA NJEGA INACE ODBACITI

namespace pt = boost::posix_time;

//-----------------------------------------------------------------------------

CanAdv::CanAdv() : open_(false), par_( CommPar( CommPar::NONE ) )
{

}

//-----------------------------------------------------------------------------

CanAdv::~CanAdv()
{
	if ( open_ )
	{
		this->Close();
	}
}

//-----------------------------------------------------------------------------

/* virtual */


bool CanAdv::Open( const CommPar& par )
{
	bool success = false;
	char device[50];
			int pom=0;
	if (!open_){
		sprintf(device, "/dev/%s",par.Port().c_str());
		std::cout << device << std::endl;

		this->fd = open(device, O_RDWR|O_NONBLOCK);

		//std::cout << "grrrrr" << fd << std::endl;
		//std::cout << errno << std::endl;

		
		if (fd>=0) {
			switch ( par.Baudrate() )
			{
			case 10000:
				set_bitrate(this->fd,10);
				break;
			case 20000:
				set_bitrate(this->fd,20);
				break;
			case 50000:
				set_bitrate(this->fd,50);
				break;
			case 100000:
				set_bitrate(this->fd,100);
				break;
			case 125000:
				pom=set_bitrate(this->fd,125);
				break;
			case 250000:
				set_bitrate(this->fd,250);
				break;
			case 500000:
				set_bitrate(this->fd,500);
				break;
			case 800000:
				set_bitrate(this->fd,800);
				break;
			case 1000000:
				set_bitrate(this->fd,1000);
				break;
			default:
				set_bitrate(this->fd,125);
			}
			open_ = true;
			par_ = par;
			success=true;
		}
	}
	else{
		if (par_==par){
			success=true;
		}
	}
	//std::cout << "Hra_test_OPEN_1->errno: " << errno << " fd: " << fd << " pom: " << pom << std::endl;
	return success;
		
}

//-----------------------------------------------------------------------------

int CanAdv::set_bitrate(int can_fd, int baud )
{
	Config_par_t  cfg;
	volatile Command_par_t cmd;

   cmd.cmd = CMD_STOP;
   ioctl(can_fd, CAN_IOCTL_COMMAND, &cmd);

   cfg.target = CONF_TIMING; 
   cfg.val1   = baud;
   ioctl(can_fd, CAN_IOCTL_CONFIG, &cfg);

   cmd.cmd = CMD_START;
   ioctl(can_fd, CAN_IOCTL_COMMAND, &cmd);
   return 0;
}

//-----------------------------------------------------------------------------

/* virtual */
bool CanAdv::Close()
{
	if ( open_ )
	{
		close(this->fd);
		open_ = false;
	}
	return !open_;
}

//-----------------------------------------------------------------------------

/* virtual */
bool CanAdv::Send( const CommMsg& msg)
{
	bool success = false;
	if ( open_ )
	{
		size_t msgSize =msg.Size();
		if (msgSize>8) msgSize=8;
		canmsg_t tx;
		tx.flags=0;
		tx.id=msg.Id();
		tx.length=msgSize;
		memcpy(&tx.data[0],msg.Data(),msgSize);
	
		int pom2=write(fd,&tx,1);
		//int pom2=ioctl(fd, CAN_IOCTL_SEND, &tx);

		if (pom2>0)
		{
			//std::cout << "Hra_test_uspjesno_SEND_1->errno: " << errno << " fd: " << fd << " pom: " << pom2 << std::endl;
			success = true;
		}
		else
		{
			//std::cout << "Hra_test_SEND_1->errno: " << errno << " fd: " << fd << std::endl;
		}
				
	}
	else
	{
		//std::cout << "Nije otvoren" << std::endl;
	}

	return success;
}

//-----------------------------------------------------------------------------

/* virtual */
bool CanAdv::Receive(CommMsg& msg, unsigned short timeout)
{
	// Read the message from the bus
	bool success = false;
	boost::posix_time::time_duration dur;
	pt::ptime t1,t2;
	canmsg_t rx;
//	Receive_par_t par;
	CanStatusPar_t status;
	if ( open_ )
	{
	///*	
		//std::cout << "Hra_test_RECEIVE_1->errno: " << errno << " fd: " << fd << std::endl;
		t1=boost::posix_time::microsec_clock::local_time();
		//std::cout << "Hra_test_RECEIVE_2->errno: " << errno << " fd: " << fd << std::endl;
		while (1){
			t2=boost::posix_time::microsec_clock::local_time();
			dur=t2-t1;
			if (dur.total_milliseconds()>timeout){
				//std::cout << "Hra_test_RECEIVE_timeout->errno: " << errno << " fd: " << fd << std::endl;
				success=false;
				break;
			}
			ioctl(this->fd,CAN_IOCTL_STATUS, &status);
			if (status.rx_buffer_used>0){
				//std::cout << "Hra_test_RECEIVE1_used (" << status.rx_buffer_used << ") ->errno: " << errno << " fd: " << fd << std::endl;
				int pom3=read(this->fd,&rx,1);
				//std::cout << "Hra_test_RECEIVE1.1_read:" << pom3 << "\n" << std::endl;
				//if (read(this->fd,&rx,1)>0){
					if ((pom3)>0){
					//std::cout << "Hra_test_RECEIVE2_read->errno: " << errno << " fd: " << fd << std::endl;
					// Fill CommMsg data from vciMsg
					msg.Id( rx.id );
					msg.Data( (const char*) rx.data,rx.length );
					//! \todo Use CANMSG.dwTime for the timestamp (figure out the format)
					msg.Timestamp( boost::posix_time::microsec_clock::local_time() ); 
					success = true;
					break;
				}
				else{
					//std::cout << "Hra_test_RECEIVE_PUKNO_________!!!! " << errno << " fd: " << fd << std::endl;
				}
			}
		}
	//*/
	}
	return success;
}

//-----------------------------------------------------------------------------


int main( int argc, char* argv[] )
{

	ros::init(argc, argv, "vatroslav");
	ros::NodeHandle n;
	ros::Rate loop_rate(1);
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	std_msgs::String msg2;
	std::stringstream ss;
	ss << "hello world " << std::cout;
	msg2.data = ss.str();

	chatter_pub.publish(msg2);
	ros::spinOnce();

		CommPar par( CommPar::UNO,125000,"can1" );

//	std::cout << par << std::endl;	
//	if( CommPtr p_comm( Communication::Create( Communication::BLOCKING, par ) ) ) printf("Hra_test_1\n");
	CommPtr p_comm( Communication::Create( Communication::BLOCKING, par ) );
//	p_comm->Open();
	
	//int debug;
	p_comm->Open();
}

