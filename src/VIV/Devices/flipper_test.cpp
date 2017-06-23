
/*!	\file	flipper_test.cpp
	
	Main file for the flipper-testing application.
 */

#include "flipper_test.hpp"


//using namespace std;

using namespace Vatroslav;
namespace pt = boost::posix_time;


ros::Publisher sendToCAN;
ros::Subscriber subCAN;

std::list<Vatroslav::CommMsg> msgList;	// vector for storing data from subscriber

namespace Can_ROS_UNO_2 {
	bool SendV(const CommMsg &por1) {

		vatroslav::CanMsg por2;
		const boost::posix_time::ptime temp(por1.Timestamp());

		ros::Time temp2 = ros::Time::fromBoost(temp);

		//boost::posix_time::ptime pt = t.toBoost();

		//  std::string temp2 = boost::posix_time::to_iso_string(temp);

		por2.id = por1.Id();
		por2.data = std::string(por1.Data(), por1.Size());
		por2.size = por1.Size();
		por2.time = temp2;

		std::cout << "Gotova priprema za slanje flipperTesta na sendToCAN" << std::endl;
		sendToCAN.publish(por2);

		std::cout << "Poslano flipperTesta na sendToCAN" << std::endl;
		ROS_DEBUG("Poslano flipperTesta na sendToCAN");
		return 0;

	}

//-----------------------------------------------------------------------------

/* virtual */
	bool ReceiveV(CommMsg &msg, unsigned short timeout) {

		bool success = false;
		boost::posix_time::time_duration dur;
		pt::ptime t1, t2;

		t1 = boost::posix_time::microsec_clock::local_time();
		std::cout << "Zahtjev za primanjem flipperTest" << std::endl;
		ROS_DEBUG("Zahtjev za primanjem flipperTest");
		while (msgList.empty()) {
			t2 = boost::posix_time::microsec_clock::local_time();
			dur = t2 - t1;

			if (dur.total_milliseconds() > timeout) {
				success = false;
				break;
			}
			if (!msgList.empty()) {
				msg = msgList.front();
				msgList.pop_front();
				std::cout << "primljenjo flipperTest" << std::endl;
				ROS_DEBUG("primljenjo flipperTest");
				success = true;
				break;
			}

		}
		return success;

	}

	void canCallback(const vatroslav::CanMsg &por) {
		char result_data[] = {0, 0, 0, 0, 0, 0, 0, 0};
		result_data[0] = (char) por.data[0];
		result_data[1] = (char) por.data[1];
		result_data[2] = (char) por.data[2];
		result_data[3] = (char) por.data[3];
		result_data[4] = (char) por.data[4];
		result_data[5] = (char) por.data[5];
		result_data[6] = (char) por.data[6];
		result_data[7] = (char) por.data[7];

		Vatroslav::CommMsg result((unsigned short) 1, result_data, (size_t) por.size, (por.time).toBoost());

		msgList.push_back(result);
		ROS_DEBUG("Skunto s topica flipperTest");

	}
}
//-----------------------------------------------------------------------------
#include "flipper_test_CAN.hpp"
using namespace CanROS_UNO;
using namespace Can_ROS_UNO_2;

int main( int argc, char* argv[] )
{



	ros::init(argc, argv, "vatroslav");
	ros::NodeHandle n;	

  	ros::Subscriber sub = n.subscribe("receiveCAN", 1000, canCallback);

	//subCAN = n.subscribe("receiveCAN", 100, canTopicCallback); // TREBA ODKOMENTIRAT ALI NE RADI !!!!!!!!!!!!!!!!!!!!!!
	sendToCAN = n.advertise<vatroslav::CanMsg>("sendCAN", 1000);

	ROS_INFO("krenuo flipperTest");
	std::cout << "krenuo flipperTest" << std::endl;

	CommPar par( CommPar::UNO,125000,"can0" );

	CommPtr p_comm( Communication::Create( Communication::BLOCKING, par ) );


	//samo demo ya git

	
	while (1){
		char data[] = { 1,2,3,4,5,6,7,8};
	
		CommMsg msg( 1, data, 8, boost::posix_time::microsec_clock::local_time() );

		std::cout << "debugg petlja" << p_comm->Send( msg ) << std::endl;
		//p_comm->Send( msg );
		sleep(1);
	}



	RegParEPOS par2; 
	MotionParEPOS par3(PROFILE_VELOCITY_MODE);
	MotorParEPOS par41( 1, par2, par3 );
	MotorParEPOS par42( 2, par2, par3 );
	MotorParEPOS par43( 3, par2, par3 );
	MotorParEPOS par44( 4, par2, par3 );

	
	boost::shared_ptr<MotorEPOS> motor1(new MotorEPOS( par41, p_comm ));
	boost::shared_ptr<MotorEPOS> motor2(new MotorEPOS( par42, p_comm ));
	boost::shared_ptr<MotorEPOS> motor3(new MotorEPOS( par43, p_comm ));
	boost::shared_ptr<MotorEPOS> motor4(new MotorEPOS( par44, p_comm ));

	
	boost::shared_ptr<LinAct> lin1(new LinAct(101,p_comm));//stvoriti svoje nodove ya lin act??????????????
	boost::shared_ptr<LinAct> lin2(new LinAct(102,p_comm));
	boost::shared_ptr<LinAct> lin3(new LinAct(103,p_comm));
	boost::shared_ptr<LinAct> lin4(new LinAct(104,p_comm));

	

	FlipperPar flip(0.05,0.15);

	boost::shared_ptr<AngleSensor> ang1(new AngleSensor(motor1,2000,1));
	boost::shared_ptr<AngleSensor> ang2(new AngleSensor(motor2,1494,-1));
	
	boost::shared_ptr<AngleSensor> ang3(new AngleSensor(motor3,1452,-1));
	boost::shared_ptr<AngleSensor> ang4(new AngleSensor(motor4,3422,1));


	boost::shared_ptr<Flipper> flipper1(new Flipper(motor3,lin2,flip,ang3));
	boost::shared_ptr<Flipper> flipper2(new Flipper(motor4,lin1,flip,ang4));
	boost::shared_ptr<Flipper> flipper3(new Flipper(motor1,lin3,flip,ang1));
	boost::shared_ptr<Flipper> flipper4(new Flipper(motor2,lin4,flip,ang2));
	
	boost::shared_ptr<Kinematics> kin(new Kinematics(flipper2,flipper1,flipper3,flipper4));
	
	boost::shared_ptr<GasSensor> gas1(new GasSensor(0,p_comm));
	boost::shared_ptr<GasSensor> gas2(new GasSensor(1,p_comm));
	boost::shared_ptr<GasSensor> gas3(new GasSensor(2,p_comm));

	gas1->Connect();
	gas2->Connect();
	gas3->Connect();

	boost::shared_ptr<PowerSensor> pow1(new PowerSensor(0,p_comm));
	boost::shared_ptr<PowerSensor> pow2(new PowerSensor(1,p_comm));
	boost::shared_ptr<PowerSensor> pow3(new PowerSensor(2,p_comm));
	boost::shared_ptr<PowerSensor> pow4(new PowerSensor(3,p_comm));
	boost::shared_ptr<PowerSensor> pow5(new PowerSensor(4,p_comm));
	boost::shared_ptr<PowerSensor> pow6(new PowerSensor(5,p_comm));

	boost::shared_ptr<RotateCamera> rot(new RotateCamera(p_comm));


	if (pow1->Connect()) printf("Sensor 1 connected\n");
	if (pow2->Connect()) printf("Sensor 2 connected\n");
	if (pow3->Connect()) printf("Sensor 3 connected\n");
	if (pow4->Connect()) printf("Sensor 4 connected\n");
	if (pow5->Connect()) printf("Sensor 5 connected\n");
	if (pow6->Connect()) printf("Sensor 6 connected\n");

	pow1->UpdateRead();
	pow2->UpdateRead();
	pow3->UpdateRead();
	pow4->UpdateRead();
	pow5->UpdateRead();
	

	printf("Pow1=%d\n",pow1->GetValue(0));
	printf("Pow2=%d\n",pow2->GetValue(0));
	printf("Pow3=%d\n",pow3->GetValue(0));
	printf("Pow4=%d\n",pow4->GetValue(0));
	printf("Pow5=%d\n",pow5->GetValue(0));


	boost::shared_ptr<TemperatureSensor> temp1(new TemperatureSensor(300,p_comm));
	boost::shared_ptr<TemperatureSensor> temp2(new TemperatureSensor(402,p_comm));


	boost::shared_ptr<PositionSensor> pos1(new PositionSensor(lin1));
	boost::shared_ptr<PositionSensor> pos2(new PositionSensor(lin2));


	boost::shared_ptr<PositionSensor> pos3(new PositionSensor(lin3));
	boost::shared_ptr<PositionSensor> pos4(new PositionSensor(lin4));


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
	
	CommMsg msg( 1, data, 8, boost::posix_time::microsec_clock::local_time() );

	
	while (p_comm->Receive(msg,30));//ovo je praznjenje buffera
	
	wir.EmptyBuffer();
	int brojac=0;
	
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
			//printf("\n\nIzbaceno paketa %d\n",wir.EmptyBuffer());
			brojac=0;			
		}
		now=boost::posix_time::microsec_clock::local_time() ;
		dur=now-time;
		while (dur.total_milliseconds()<20){
			now=boost::posix_time::microsec_clock::local_time() ;
			dur=now-time;
		}

		//printf("                                                 kut je mot3:%d mot4:%d mot1:%d mot2:%d\n",ang3->GetValue(1),ang4->GetValue(1),ang1->GetValue(1),ang2->GetValue(1));
		//motor3->VelocityRef(100);
//		motor3->UpdateWrite();
//		motor2->VelocityRef(500);
	//	motor2->UpdateWrite();
	//	motor4->VelocityRef(100);
//		motor4->UpdateWrite();
		
/*		for (i=0;i<10000;i++){
			for (j=0;j<10000;j++) t++;
		}*/

		//printf("				Pozicije:\n					 motor3: %d\n					motor4: %d\n					motor1: %d\n					motor2: %d\n",motor3->Position(),motor4->Position(),motor1->Position(),motor2->Position());
	}

	ros::spinOnce();

	return 0;
}
