#include "Sensor.hpp"

namespace Vatroslav
{
	
namespace pt = boost::posix_time;

//=============================================================================

GasSensor::GasSensor(int SensorNumber, CommPtr pComm) : pComm_( pComm ),
														number( SensorNumber )
{
	open_=false;
}

//-----------------------------------------------------------------------------

GasSensor::~GasSensor()
{

}

//-----------------------------------------------------------------------------

int GasSensor::Status()
{
	long long duration;
	boost::posix_time::ptime now=boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration dur=now-time;
	duration=dur.total_microseconds();
	duration+=this->DelayTime;
	if (status==0) return 0;
	if (duration<this->ALLOWED_DELAY) return 1;
	else if (duration<this->MAXIMUM_DELAY) return 2;
	else return 3;
}

//-----------------------------------------------------------------------------

int GasSensor::GetValue(int no)
{
//	if (status==0) return -1;
	return value;
}

//-----------------------------------------------------------------------------

int GasSensor::Type()
{
	if (status==0) return -1;
	return type;
}

//-----------------------------------------------------------------------------

bool GasSensor::UpdateRead()
{
	if (open_==false) return false;
	int id_=300;
	char data[8];
	data[0]=2+this->number;
	CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
	if (pComm_->Send(msg)){
		if (pComm_->Receive(msg,1000)){
			this->RecievingStatus=msg.Data()[1];
			this->value=(unsigned char) msg.Data()[2]*256+(unsigned char) msg.Data()[3];
			this->DelayTime=(msg.Data()[4]*256+msg.Data()[5])*10;
			return true;
		}
	}
	return false;
}

//-----------------------------------------------------------------------------

bool GasSensor::UpdateWrite(){
	if (open_==false) return false;
	return true;
}

//-----------------------------------------------------------------------------

bool GasSensor::Connect(){
	pComm_->Open();
	int id_=300;
	char data[8];
	data[0]=0;
	CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
	if (pComm_->Send(msg)){
		if (pComm_->Receive( msg, 1000 )){
			this->status=1;
			open_=true;
			return true;
		}
	}	
	this->status=0;
	return false;
}

//-----------------------------------------------------------------------------

bool GasSensor::Disconnect()
{
	return false;
}

//=============================================================================

TemperatureSensor::TemperatureSensor(int ID, CommPtr pComm) 
	: pComm_( pComm ), id_( ID ) 
{
		open_=false;
}

//-----------------------------------------------------------------------------

TemperatureSensor::~TemperatureSensor()
{

}

//-----------------------------------------------------------------------------

int TemperatureSensor::Status()
{
	long long duration;
	boost::posix_time::ptime now=boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration dur=now-time;
	duration=dur.total_microseconds();
	duration+=this->DelayTime;
	if (status==0) return 0;
	if (duration<this->ALLOWED_DELAY) return 1;
	else if (duration<this->MAXIMUM_DELAY) return 2;
	else return 3;
}

//-----------------------------------------------------------------------------

int TemperatureSensor::GetValue(int no)
{
	return value;
}

//-----------------------------------------------------------------------------

bool TemperatureSensor::UpdateRead()
{
	if (open_==false) return false;
	char data[8];
	data[0]=99;
	CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
	if (pComm_->Send(msg)){
		if (pComm_->Receive(msg,1000)){
			this->value=(unsigned char) msg.Data()[1]*256+(unsigned char) msg.Data()[2];
			this->DelayTime=(msg.Data()[3]*256+msg.Data()[4])*10;
			return true;
		}
	}
	return false;
}

//-----------------------------------------------------------------------------

bool TemperatureSensor::UpdateWrite(){
	if (open_==false) return false;
	return true;
}

//-----------------------------------------------------------------------------

bool TemperatureSensor::Connect()
{
	pComm_->Open();
	char data[8];
	data[0]=99;
	CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
	if (pComm_->Send(msg)){
		if (pComm_->Receive( msg, 1000 )){
			open_=true;
			this->status=1;
			return true;
		}
	}	
	this->status=0;
	return false;
}

//-----------------------------------------------------------------------------

bool TemperatureSensor::Disconnect(){
	return true;
}

//=============================================================================

AHRS::AHRS( CommPtr pComm ) : pComm_( pComm ), status( 0 )
{

}

//-----------------------------------------------------------------------------


AHRS::~AHRS(){
}

//-----------------------------------------------------------------------------

int AHRS::Status()
{
	long long duration;
	boost::posix_time::ptime now=boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration dur=now-time;
	duration=dur.total_microseconds();
	duration+=this->DelayTime;
	if (duration<this->ALLOWED_DELAY) return 1;
	else if (duration<this->MAXIMUM_DELAY) return 2;
	else return 3;
}

//-----------------------------------------------------------------------------

float AHRS::GetValue(int no)
{
	switch (no)
	{
		case 1:
			return this->roll;
			break;
		case 2:
			return this->pitch;
			break;
		case 3:
			return this->yaw;
			break;
		case 4:
			return this->ax;
			break;
		case 5:
			return this->ay;
			break;
		case 6:
			return this->az;
			break;
		default:
			return 0;
	}

}

//-----------------------------------------------------------------------------

bool AHRS::UpdateRead()
{
	int id_=300;	
	char data[8];
	data[0]=5;
	int sign1,sign2,sign3;
	CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
	if (status==0) return 0;
	if (pComm_->Send(msg)){
		if (pComm_->Receive(msg,1000)){
			sign1=msg.Data()[0]/abs(msg.Data()[0]);
			sign2=msg.Data()[2]/abs(msg.Data()[2]);
			sign3=msg.Data()[4]/abs(msg.Data()[4]);
			this->roll= (float) msg.Data()[0]*256+sign1*msg.Data()[1];
			this->pitch= (float) msg.Data()[2]*256+sign2*msg.Data()[3];
			this->yaw= (float) msg.Data()[4]*256+sign3*msg.Data()[5];
			this->DelayTime=(msg.Data()[6]*256+msg.Data()[7])*10;
			return true;
		}
	}
	return false;
}

//-----------------------------------------------------------------------------

bool AHRS::UpdateWrite(){
	return true;
}

//-----------------------------------------------------------------------------


bool AHRS::Connect()
{
	int id_=300;
	pComm_->Open();
	char data[8];
	data[0]=0;
	CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
	if (pComm_->Send(msg)){
		if (pComm_->Receive( msg, 1000 )){
			this->status=1;
			return true;
		}
	}	
	this->status=0;
	return false;
}

//-----------------------------------------------------------------------------

bool AHRS::Disconnect()
{
	return true;
}


//=============================================================================

PowerSensor::PowerSensor(int no, CommPtr pComm): pComm_( pComm ),
														number_( no )
{
	open_=false;
}

//-----------------------------------------------------------------------------
bool PowerSensor::Connect()
{
	pComm_->Open();
	int id_=50;
	char data[8];
	data[0]=1;
	CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
	if (pComm_->Send(msg)){
		if (pComm_->Receive( msg, 1000 )){
			this->status=1;
			open_=true;
			return true;
		}
	}	
	this->status=0;
	return false;
	
}

	
//-----------------------------------------------------------------------------
bool PowerSensor::UpdateRead()
{
	if (open_==false) return false;
	int id_=50;
	char data[8];
	if (number_<3)
	data[0]=1;
	else data[0]=2;
	status=0;
	CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
	if (pComm_->Send(msg)){
		if (pComm_->Receive(msg,1000)){	
			status=1;	
			if (number_==0) this->value=((unsigned char)msg.Data()[1]*256+(unsigned char) msg.Data()[2])*(33000+2200)/2200;
			if (number_==1) this->value=(unsigned char)msg.Data()[3]*256+(unsigned char) msg.Data()[4];
			if (number_==2) this->value=((unsigned char)msg.Data()[5]*256+(unsigned char) msg.Data()[6])*(2200+1050)/1050;
			if (number_==3) this->value=((unsigned char)msg.Data()[1]*256+(unsigned char) msg.Data()[2])*(5600+1050)/1050;
			if (number_==4) this->value=(unsigned char)msg.Data()[3]*256+(unsigned char) msg.Data()[4];
			if (number_==5) this->value=((unsigned char)msg.Data()[5]*256+(unsigned char) msg.Data()[6])*(2200+1050)/1050;;
			return true;
		}
	}
	return false;
}
	
//-----------------------------------------------------------------------------

bool PowerSensor::UpdateWrite()
{
	return true;
}	


//=============================================================================

bool AngleSensor::Connect(){
	return mot_->Connect();
}

//-----------------------------------------------------------------------------

bool AngleSensor::Disconnect(){
	return true;
}

//-----------------------------------------------------------------------------

bool AngleSensor::UpdateRead()
{
	bool rez;
	rez=mot_->UpdateAnalogInputs();
	value=mot_->AnalogInput2();
	if (rez) status=1; else status=0;
	return rez;
}

//-----------------------------------------------------------------------------

bool AngleSensor::UpdateWrite(){
	return true;
}

//-----------------------------------------------------------------------------

int AngleSensor::GetNumberOfWords(){
	return 1;
}

//-----------------------------------------------------------------------------

int AngleSensor::GetValue(int no){
	int pom;
	pom=pred_*(value-zero_);
	if (pom<0) pom=pom+4000; 
	return pom*360/4000;
}

//-----------------------------------------------------------------------------

int AngleSensor::Status(){
	return status;
}

}
