/*!	\file	flipper_test.cpp
	
	Main file for the flipper-testing application.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h>
#include "../Communication/Communication.hpp"
#include "MotorEPOS.hpp"
#include "LinAct.hpp"
//#include "Windows.h"
#include "../Communication/SerialBoost.hpp"
#include "WirelessVIV.hpp"
#include <unistd.h>
using namespace Vatroslav;
int main( int argc, char* argv[] )
{

	ros::init(argc, argv, "mainControl");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	std_msgs::String msg2;
	std::stringstream ss;
	ss << "hello world " << count;
	msg2.data = ss.str();

	chatter_pub.publish(msg2);

		CommPar par( CommPar::UNO,125000,"can1" );

//	std::cout << par << std::endl;	
//	if( CommPtr p_comm( Communication::Create( Communication::BLOCKING, par ) ) ) printf("Hra_test_1\n");
	CommPtr p_comm( Communication::Create( Communication::BLOCKING, par ) );
//	p_comm->Open();
	
	int debug;
	p_comm->Open();
	//if(	p_comm->Open()	) printf("Hra_test_1\n");
	RegParEPOS par2; 
	MotionParEPOS par3(PROFILE_VELOCITY_MODE);
	//std::cout << "Hra_test_2" << std::endl;
	MotorParEPOS par41( 1, par2, par3 );
	MotorParEPOS par42( 2, par2, par3 );
	MotorParEPOS par43( 3, par2, par3 );
	MotorParEPOS par44( 4, par2, par3 );
	//std::cout << "Hra_test_3" << std::endl;
	
	boost::shared_ptr<MotorEPOS> motor1(new MotorEPOS( par41, p_comm ));
	boost::shared_ptr<MotorEPOS> motor2(new MotorEPOS( par42, p_comm ));
	boost::shared_ptr<MotorEPOS> motor3(new MotorEPOS( par43, p_comm ));
	boost::shared_ptr<MotorEPOS> motor4(new MotorEPOS( par44, p_comm ));	
	//std::cout << "Hra_test_4" << std::endl;
	
	boost::shared_ptr<LinAct> lin1(new LinAct(101,p_comm));
	boost::shared_ptr<LinAct> lin2(new LinAct(102,p_comm));
	boost::shared_ptr<LinAct> lin3(new LinAct(103,p_comm));
	boost::shared_ptr<LinAct> lin4(new LinAct(104,p_comm));

	
	//std::cout << "Hra_test_5" << std::endl;
	FlipperPar flip(0.05,0.15);

	boost::shared_ptr<AngleSensor> ang1(new AngleSensor(motor1,2000,1));
	boost::shared_ptr<AngleSensor> ang2(new AngleSensor(motor2,1494,-1));
	
	boost::shared_ptr<AngleSensor> ang3(new AngleSensor(motor3,1452,-1));
	boost::shared_ptr<AngleSensor> ang4(new AngleSensor(motor4,3422,1));


	//std::cout << "Hra_test_6" << std::endl;
	boost::shared_ptr<Flipper> flipper1(new Flipper(motor3,lin2,flip,ang3));
	boost::shared_ptr<Flipper> flipper2(new Flipper(motor4,lin1,flip,ang4));
	boost::shared_ptr<Flipper> flipper3(new Flipper(motor1,lin3,flip,ang1));
	boost::shared_ptr<Flipper> flipper4(new Flipper(motor2,lin4,flip,ang2));
	
	boost::shared_ptr<Kinematics> kin(new Kinematics(flipper2,flipper1,flipper3,flipper4));
	
	//std::cout << "Hra_test_7" << std::endl;
	boost::shared_ptr<GasSensor> gas1(new GasSensor(0,p_comm));
	boost::shared_ptr<GasSensor> gas2(new GasSensor(1,p_comm));
	boost::shared_ptr<GasSensor> gas3(new GasSensor(2,p_comm));

	gas1->Connect();
	gas2->Connect();
	gas3->Connect();

	//std::cout << "Hra_test_8" << std::endl;
	boost::shared_ptr<PowerSensor> pow1(new PowerSensor(0,p_comm));
	boost::shared_ptr<PowerSensor> pow2(new PowerSensor(1,p_comm));
	boost::shared_ptr<PowerSensor> pow3(new PowerSensor(2,p_comm));
	boost::shared_ptr<PowerSensor> pow4(new PowerSensor(3,p_comm));
	boost::shared_ptr<PowerSensor> pow5(new PowerSensor(4,p_comm));
	boost::shared_ptr<PowerSensor> pow6(new PowerSensor(5,p_comm));

	boost::shared_ptr<RotateCamera> rot(new RotateCamera(p_comm));


	//std::cout << "Hra_test_9" << std::endl;
	if (pow1->Connect()) printf("Sensor 1 connected\n");
	if (pow2->Connect()) printf("Sensor 2 connected\n");
	if (pow3->Connect()) printf("Sensor 3 connected\n");
	if (pow4->Connect()) printf("Sensor 4 connected\n");
	if (pow5->Connect()) printf("Sensor 5 connected\n");
	if (pow6->Connect()) printf("Sensor 6 connected\n");

	//std::cout << "Hra_test_10" << std::endl;
	pow1->UpdateRead();
	pow2->UpdateRead();
	pow3->UpdateRead();
	pow4->UpdateRead();
	pow5->UpdateRead();
	
	
	//std::cout << "Hra_test_11" << std::endl;
	printf("Pow1=%d\n",pow1->GetValue(0));
	printf("Pow2=%d\n",pow2->GetValue(0));
	printf("Pow3=%d\n",pow3->GetValue(0));
	printf("Pow4=%d\n",pow4->GetValue(0));
	printf("Pow5=%d\n",pow5->GetValue(0));

	//std::cout << "Hra_test_12" << std::endl;

	boost::shared_ptr<TemperatureSensor> temp1(new TemperatureSensor(300,p_comm));
	boost::shared_ptr<TemperatureSensor> temp2(new TemperatureSensor(402,p_comm));

	//std::cout << "Hra_test_13" << std::endl;

	boost::shared_ptr<PositionSensor> pos1(new PositionSensor(lin1));
	boost::shared_ptr<PositionSensor> pos2(new PositionSensor(lin2));
	
	//std::cout << "Hra_test_14" << std::endl;

	boost::shared_ptr<PositionSensor> pos3(new PositionSensor(lin3));
	boost::shared_ptr<PositionSensor> pos4(new PositionSensor(lin4));

	//std::cout << "Hra_test_15" << std::endl;

	boost::shared_ptr<Sensors> sen(new Sensors);
	temp1->Connect();
	sen->Add(gas1);
	sen->Add(gas2);
	sen->Add(gas3);
	sen->Add(temp1);
	sen->Add(temp2);
	sen->Add(pos1);
	sen->Add(pos2);
	sen->Add(pos3);
	sen->Add(pos4);
	sen->Add(pow1);
	sen->Add(pow2);
	sen->Add(pow3);
	sen->Add(pow4);
	sen->Add(pow5);
	sen->Add(pow6);
	sen->Add(ang1);	
	sen->Add(ang2);
	sen->Add(ang3);
	sen->Add(ang4);
	
	//std::cout << "Hra_test_16" << std::endl;

	if (lin1->Connect()) printf("Linear Actuator 1 connected\n");
	//	lin1->ChangeConstant(LinAct::end_position, 780);
	if (lin2->Connect()) printf("Linear Actuator 2 connected\n");
	//	lin2->ChangeConstant(LinAct::end_position, 720);
	if (lin3->Connect()) printf("Linear Actuator 3 connected\n");
	if (lin4->Connect()) printf("Linear Actuator 4 connected\n");
	/*
lin1->ChangeConstant(LinAct::max_current,1600);
	sleep(1);
lin2->ChangeConstant(LinAct::max_current,1600);
	sleep(1);
lin3->ChangeConstant(LinAct::max_current,1600);
	sleep(1);
lin4->ChangeConstant(LinAct::max_current,1600);
	sleep(1);
*/
/*	lin1->ChangeConstant(LinAct::start_position,190);
	sleep(1);
	lin1->ChangeConstant(LinAct::end_position,470);
	sleep(1);
	lin2->ChangeConstant(LinAct::start_position,90);
	sleep(1);
	lin2->ChangeConstant(LinAct::end_position,400);
	sleep(1);
	lin3->ChangeConstant(LinAct::start_position,528);	
	sleep(1);
	lin3->ChangeConstant(LinAct::end_position,763);
	sleep(1);
	lin4->ChangeConstant(LinAct::start_position,550);
	sleep(1);
	lin4->ChangeConstant(LinAct::end_position,784);
	sleep(1);*/


	lin1->ReadConstants();
	lin2->ReadConstants();
	lin3->ReadConstants();
	lin4->ReadConstants();
	boost::shared_ptr<Power> pow(new Power(p_comm));
	pow->Connect();	
	pow->SetStateDrive(true);
	pow->UpdateWrite();

	//std::cout << "Hra_test_17" << std::endl;

//	motor1->SetImmediateRef(true);
	if (motor1->Connect()) printf("Motor1 connected\n");
	motor1->Params().MotionParams().SetProfileVelocity(500);
	motor1->Params().MotionParams().SetProfileAcceleration(5000);
	motor1->Params().MotionParams().SetProfileDeceleration(5000);
	motor1->UpdateMotionParWrite();
	motor1->UpdateWrite();

//	motor2->SetImmediateRef(true);
	if (motor2->Connect()) printf("Motor2 connected\n");
	motor2->Params().MotionParams().SetProfileVelocity(500);
	motor2->Params().MotionParams().SetProfileAcceleration(5000);
	motor2->Params().MotionParams().SetProfileDeceleration(5000);
	motor2->UpdateMotionParWrite();
	motor2->UpdateWrite();
	
//	motor3->SetImmediateRef(true);
	if (motor3->Connect()) printf("Motor3 connected\n");
	motor3->Params().MotionParams().SetProfileVelocity(500);
	motor3->Params().MotionParams().SetProfileAcceleration(5000);
	motor3->Params().MotionParams().SetProfileDeceleration(5000);
	motor3->UpdateMotionParWrite();
	motor3->UpdateWrite();

//	motor4->SetImmediateRef(true);
	if (motor4->Connect()) printf("Motor4 connected\n");
	motor4->Params().MotionParams().SetProfileVelocity(500);
	motor4->Params().MotionParams().SetProfileAcceleration(5000);
	motor4->Params().MotionParams().SetProfileDeceleration(5000);
	motor4->UpdateMotionParWrite();
	motor4->UpdateWrite();

	WirelessVIV wir(kin,sen,pow,rot);
	kin->Connect();
	wir.Open("/dev/ttyS0");
	int i=0,j,t=0;

	//std::cout << "Hra_test_18" << std::endl;

/*	lin2->UpdateRead();
	lin2->Off();
	lin2->UpdateWrite();*/
//	flipper1->ChangeMode(Flipper::DRIVE);
//	flipper1->UpdateWrite();
//	kin->SetState(Kinematics::run,Kinematics::front_left);
//	kin->UpdateWrite();
	boost::posix_time::ptime time;
	boost::posix_time::ptime now;
		boost::posix_time::time_duration dur;
	char data[] = { 3,2,3,4,5,6,7,8};
	//std::cout << "Hra_test_19" << std::endl;
	
	CommMsg msg( 1, data, 8, boost::posix_time::microsec_clock::local_time() );
	//std::cout << "Hra_test_19.1" << std::endl;	
	
	while (p_comm->Receive(msg,30));//{std::cout << "Hra_test_19.1_zapeo_while" << std::endl;};//ovo je praznjenje buffera
	//std::cout << "Hra_test_19.2" << std::endl;	
	wir.EmptyBuffer();
	int brojac=0;
	//std::cout << "Hra_test_19.3" << std::endl;	
	while (1){
		time=boost::posix_time::microsec_clock::local_time() ;
		//std::cout<<time<<std::endl;
		while (p_comm->Receive(msg,10));
		if (pow->GetStateDrive()){
			kin->UpdateWrite();
			kin->UpdateRead();
		}
		else {
			kin->QuickStop();
		}
//		motor3->UpdateAnalogInputs();
	//	motor4->UpdateAnalogInputs();
//
//		printf("Analogna vrijednost je 1:%d\n",motor1->AnalogInput2());
		//printf("Analogna vrijednost je 2:%d\n",motor2->AnalogInput2());

//		printf("Analogna vrijednost je 3:%d\n",motor3->AnalogInput2());
//		printf("Analogna vrijednost je 4:%d\n",motor4->AnalogInput2());
		//std::cout << "Hra_test_19.4_zapeo_while" << std::endl;
		now=boost::posix_time::microsec_clock::local_time() ;
		dur=now-time;
//		printf("Vrijeme osvjezavanja je %d\n", dur.total_milliseconds());
//		if (lin3->UpdateRead() && lin4->UpdateRead()) printf("Uspjesno\n"); else printf("error\n");
		sen->UpdateRead();

	//std::cout << "Hra_test_20" << std::endl;

		sen->UpdateWrite();
		pow->UpdateWrite();
		rot->UpdateWrite();
/*		if (pow->ReConnectDrives()){
			kin->Connect();
		}*/
//		printf("pos=%d %d\n",lin3->GetPosition(),lin4->GetPosition());
		wir.ReceiveMessage(400);
		wir.SendMessage();
		brojac++;
		if (brojac>10){
			printf("\n\nIzbaceno paketa %d\n",wir.EmptyBuffer());
			brojac=0;			
		}
		now=boost::posix_time::microsec_clock::local_time() ;
		dur=now-time;
		while (dur.total_milliseconds()<20){
			now=boost::posix_time::microsec_clock::local_time() ;
			dur=now-time;
		}

		printf("                                                 kut je mot3:%d mot4:%d mot1:%d mot2:%d\n",ang3->GetValue(1),ang4->GetValue(1),ang1->GetValue(1),ang2->GetValue(1));
		//motor3->VelocityRef(100);
//		motor3->UpdateWrite();
//		motor2->VelocityRef(500);
	//	motor2->UpdateWrite();
	//	motor4->VelocityRef(100);
//		motor4->UpdateWrite();
		
/*		for (i=0;i<10000;i++){
			for (j=0;j<10000;j++) t++;
		}*/

		printf("				Pozicije:\n					 motor3: %d\n					motor4: %d\n					motor1: %d\n					motor2: %d\n",motor3->Position(),motor4->Position(),motor1->Position(),motor2->Position());
	}

	std::cout << "Hra_test_21_kraj" << std::endl;
	return 0;
}