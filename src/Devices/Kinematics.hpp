//#ifndef VATROSLAV_CANDEVICES_HPP
//#define VATROSLAV_CANDEVICES_HPP
#include <vector>
#include "MotorEPOS.hpp"
#include "LinAct.hpp"
#include "Device.hpp"
#include "Flipper.hpp"

#include "../Communication/Communication.hpp"

namespace Vatroslav{
typedef boost::shared_ptr<Flipper> FlipperPtr;

class Kinematics{
public:
	//!Default constructor
	/*!
	*/
	Kinematics();
	//! Conctructor
	/*!
	*/
	Kinematics(FlipperPtr fl,FlipperPtr fr,FlipperPtr bl,FlipperPtr br):fl_(fl),
																fr_(fr),
																bl_(bl),
																br_(br)
	{
	}

	//! Connect to devices
	/*!
	*/
	bool Connect(void);
	enum position {front_left,front_right,back_left,back_right};
	enum mode {flip,run};

	//!Disconnect from devices
	/*!
	*/
	bool Disconnect(void);
	//!Quick stop all drives
	/*!
	*/	
	bool QuickStop(){
		fl_->QuickStop();
		fr_->QuickStop();
		bl_->QuickStop();
		br_->QuickStop();
	}
	//!Set flipper to position
	/*!
	*/
	bool SetFlipper(FlipperPtr f,position pos);
	
	//!Get flipper on position
	/*!
	*/
	FlipperPtr GetFlipper(position pos);

	//! Read state of devices
	/*!
	*/
	bool UpdateRead();

	//!Write to devices
	/*!
	*/
	bool UpdateWrite();

	//! Set speed to forward flippers
	/*!
	*/
	void SetFwdVel(int rotvel);
	//! Set speed to back flippers
	/*!
	*/
	void SetBckVel(int rotvel);
	//! Set speed to left flippers
	/*!
	*/
	void SetLftVel(int rotvel);
	//! Set speed to right flippers
	/*!
	*/
	void SetRgtVel(int rotvel);
	//! Set speed to individual flippers
	/*!
	*/
	void SetVel(int rotvel,position pos);

	//! Set mode of individual flippers
	/*!
	*/
	bool SetState(mode m,position pos);

	//! Return data to send over wireless
	/*!
	*/
	int WirelessData(char* data);
	
	//! Set kinematics mode
	/*!
	*/
	bool SetMode(int mode);
private:
	int mode_,step_;
	FlipperPtr fl_;
	FlipperPtr fr_;
	FlipperPtr bl_;
	FlipperPtr br_;

};
typedef boost::shared_ptr<Sensor> SensorPtr;
class Sensors{
public:
	//! Constructor for CanDevices class
	/*!
	*/
	Sensors(){
		n_=0;
	}

	//! Connect all devices
	/*!
		Connect with all devices
	 */
	
	bool Connect(void);
	
	bool Disconnect(void);
	bool Add(SensorPtr sen);
	bool UpdateRead();
	bool UpdateWrite(){return true;}

	int WirelessData(char* data,int sensor);
private:
	SensorPtr sensors_[20];
	int n_;
	
};
class Power:public Device{
public:
	Power(CommPtr pComm):pComm_( pComm ){
		drivem_=false;
	}
	~Power(){
	}
	bool Disconnect(){
		return true;
	}
	bool Connect();
	bool SetStateDrive(bool drive);
	bool GetStateDrive(){
		return drive_;
	}
	bool SetStateCameras(bool cam);
	bool SetStateWireless(bool wir);
	bool SetStateSensors(bool sen);
	bool SetStateCameraSelect(bool camsel);
	bool ReConnectDrives(){
		return drivem_;
	}
	bool UpdateWrite();
	bool UpdateRead();
private:
	CommPtr pComm_;
	bool open_;
	bool drive_, cam_, wir_, sen_, camsel_,drivem_;
};
class RotateCamera:public Device{
public:
	RotateCamera(CommPtr pComm):pComm_( pComm ){
		ref_=0;
	}
	~RotateCamera(){
	}
	bool Disconnect(){
		return true;
	}
	bool SetRef(int ref){
		ref_=ref;
		return true;
	}
	bool Connect();
	bool UpdateWrite();
	bool UpdateRead(){
		return true;
	}
private:
	CommPtr pComm_;
	bool open_;
	int ref_;
};
}
