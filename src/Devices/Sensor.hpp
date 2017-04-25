#include "../Communication/Communication.hpp"
#include "Device.hpp"
#include "boost/date_time/posix_time/posix_time_types.hpp"
//#include "Solenoid.hpp"
#include "LinAct.hpp"
#include "MotorEPOS.hpp"
namespace Vatroslav
{

//! Interface for sensors
/*!
*/
struct Sensor : public Device
{
public:
	virtual int GetNumberOfWords()=0;
	//! Get value from sensor
	virtual int GetValue(int no)=0;
	//! Get status of sensor
	virtual int Status()=0;

protected:
	int type;
	int value;
	int status;
};
struct GasSensor : public Sensor
{
public:
	//! Gas sensor constructor
	/*!
		Send number of sensor : 1 - CO,
		2 - H2S, 3-17SH 
	*/
	GasSensor( int SensorNumber, CommPtr pComm );

	//! Gas sensor destructor
	~GasSensor();
	//! Get status of gas sensor
	/*!
	0 - no communication with sensor, 1 - normal operation, 2 - acceptable delay, 3 - unacceptable delay
	*/
	int Status();
	//! Get value of gas sensor
	int GetValue(int no);
	//! Get type of gas sensor
	int Type();
	//!  Update value and status of gas sensor
	bool UpdateRead();
	//!  Update settings of sensors
	bool UpdateWrite();
	//! Connect to sensor
	bool Connect();
	//! Disconnect from sensor
	bool Disconnect();
	//! Get number of words
	int GetNumberOfWords(){
		return 1;
	}

private:
	CommPtr pComm_; 
	boost::posix_time::ptime time;
	int RecievingStatus;
	int number;
	int DelayTime;
	int value;
	bool open_;
	static const int ALLOWED_DELAY=10000;
	static const int MAXIMUM_DELAY=30000;
};

//! Class for temperature sensors
struct TemperatureSensor : public Sensor
{
public:
	//! Temperature sensor constructor
	/*!
		ID determines the ID of device to send temperature	
	*/
	TemperatureSensor(int ID, CommPtr pComm);
	//! Temperature sensor destructor
	~TemperatureSensor();
	//! Get status of temperature sensor
	/*!
	0 - no communication with sensor, 1 - normal operation, 2 - acceptable delay, 3 - unacceptable delay
	*/
	int Status();
	//! Get value of temperature sensor
	int GetValue(int no);
	//!  Update value and status of temperature
	bool UpdateRead();
	//!  Update settings of sensors
	bool UpdateWrite();
	//! Conncet to temperature sensor
	bool Connect();
	//! Disconnect from temperature sensor
	bool Disconnect();
	//! Get number of words
	int GetNumberOfWords(){
		return 1;
	}

private:
	bool open_;	
	CommPtr pComm_; 
	boost::posix_time::ptime time;
	int RecievingStatus;
	int value;
	int DelayTime;
	int id_;
	static const int ALLOWED_DELAY=10000;
	static const int MAXIMUM_DELAY=30000;
};

//! Class for AHRS (Attitude and Heading Reference Systems)
struct AHRS:public Device
{
public:	
	//! AHRS constructor
	/*!
	*/
	AHRS( CommPtr pComm );
	//! AHRS destructor
	/*!
	*/
	~AHRS();
	//! Get AHRS status
	/*!
	0 - no communication with sensor, 1 - normal operation, 2 - acceptable delay, 3 - unacceptable delay
	*/
	int Status();
	//! Get value of data no from AHRS
	/*!
	Return value of no data from AHRS. For no: 1 - roll, 2 - pitch, 3 - yaw, others not yet implemented
	*/
	float GetValue(int no);
	//!  Update value from AHRS
	bool UpdateRead();
	//!  Update settings of AHRS
	bool UpdateWrite();
	//! Connect to the AHRS
	bool Connect();
	//! Disconnect from AHRS
	bool Disconnect();
private:
	bool open_;
	CommPtr pComm_; 	
	boost::posix_time::ptime time;
	int DelayTime,status;
	float roll,pitch,yaw,ax,ay,az;
	static const int ALLOWED_DELAY=1000;
	static const int MAXIMUM_DELAY=3000;
};

typedef boost::shared_ptr<LinAct> LinActPtr;
class PositionSensor:public Sensor{
public:
	PositionSensor(LinActPtr lin):lin_(lin)
	{
	}
	int GetNumberOfWords(){
		return 1;
	}
	//! Get value from sensor
	int GetValue(int no){
		return lin_->GetPosition();
	}
	//! Get status of sensor
	int Status(){
		return 1;
	}
	//! Read data from sensor
	bool UpdateRead(){
		return true;
	}
	//!  Write data to sensor
	bool UpdateWrite(){
		return true;
	}
	//! Connect to sensor
	bool Connect(){
		return true;
	}
	//! Disconnect from sensor
	bool Disconnect(){
		return true;
	}
	//! Get number of words
private:
	LinActPtr lin_;

};
class PowerSensor:public Sensor{
public:

	//!Constructor
	PowerSensor(int no, CommPtr pComm);

	//!Destructor
	~PowerSensor()
	{
	}

	//! Conncet to device
	bool Connect();

	//!Disconnect from device
	bool Disconnect(){
		return true;
	}
	//! Get status of sensor
	int Status()
	{
		return status;
	}
	
	//! Read data from sensor
	bool UpdateRead();
	
	//!  Write data to sensor
	bool UpdateWrite();
	
	int GetNumberOfWords(){
		return 1;
	}
	//! Get value from sensor
	int GetValue(int no){
		return value;
	}

private:
	bool open_;
	CommPtr pComm_; 
	int number_;
};
class AngleSensor:public Sensor{
public:
	AngleSensor(boost::shared_ptr<MotorEPOS> mot,int zero,int pred)
	{
		mot_=mot;
		zero_=zero;
		pred_=pred;
	}
	~AngleSensor(){
	}
	bool Connect();
	bool Disconnect();
	bool UpdateRead();
	bool UpdateWrite();
	//! Get number of words
	int GetNumberOfWords();
	//! Get value from sensor
	int GetValue(int no);
	//! Get status of sensor
	int Sign(){
		return pred_;
	}
	int Status();
private:
boost::shared_ptr<MotorEPOS> mot_;
int zero_,pred_;
};
}
