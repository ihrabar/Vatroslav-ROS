/*! \file	comm_main.cpp
	
	Currently just used so that we have something to compile and test. 
	In the future, this might become the file defining entry points
	for a shared library.
 */

#include <iostream>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/local_time/local_time.hpp"

#include "Communication.hpp"
#include "CommPrint.hpp"
#include "../Utils/Timer.hpp"
#include "sys/mman.h"

using namespace boost::posix_time;
using namespace Vatroslav;

int main()
{ 
	 std::cout << "Communication test program..." << std::endl;
	int i;
	CommPar par( CommPar::UNO,  125000,"can1" );
//	CommPar par( "COM1:");

	std::cout << par << std::endl;	
	CommPtr p_comm( Communication::Create( Communication::BLOCKING, par ) );
	std::cout << "Opened = " << p_comm->Open() << std::endl;
//	char data[] = { 0x40, 0xF6, 0x60, 0x01, 0x00, 0x0, 0x0, 0x0 };
	char data2[]={2};

	//CommMsg msg( 0x12c, data, 1, microsec_clock::local_time() );
	CommMsg msg1( 0x12c, data2, 1, microsec_clock::local_time() );

	//std::cout << "Transmitting data:\n" << msg << std::endl;
	int br;
	
	for (i=0;i<10;i++) {
			std::cout << "nalazim se u FOR petlji, iteracija: " << i << std::endl;
			std::cout << microsec_clock::local_time() << std::endl;
			char data[]={i};
			CommMsg msg( 0x12c, data, 1, microsec_clock::local_time() );
			std::cout << "Success s= " << p_comm->Send( msg ) << " porukaS: " << msg << std::endl;
			//std::cout << "Success r= " << p_comm->Receive( msg1, 1000 ) << " porukaP: " << msg1 << std::endl;
			//std::cout << msg1 << std::endl;
			sleep(2);	
	}

	std::cout << "Receiving data:" << std::endl; 
	
		std::cout << "Success = " << p_comm->Receive( msg1, 1000 ) << std::endl;
		std::cout << msg1 << std::endl;

	std::cout << "Press any key  and return to terminate..." << std::endl;
	char c;
	std::cin >> c;

	return 0;
}
