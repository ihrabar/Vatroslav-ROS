#include "Kinematics.hpp"
namespace Vatroslav{
	namespace pt = boost::posix_time;
//=============================================================================

	Kinematics::Kinematics(){
		mode_=0;
	}

//-----------------------------------------------------------------------------

	bool Kinematics::Connect(void){
		bool st1=fl_->Connect();
		bool st2=fr_->Connect();
		bool st3=bl_->Connect();
		bool st4=br_->Connect();
		return (st1 && st2 && st3 && st4);
	}

//-----------------------------------------------------------------------------

	bool Kinematics::Disconnect(void){
		return false;
	}

//-----------------------------------------------------------------------------

	bool Kinematics::SetFlipper(FlipperPtr f,position pos){
		if (pos==Kinematics::front_left) fl_=f;
		if (pos==Kinematics::front_right) fr_=f;
		if (pos==Kinematics::back_left) bl_=f;
		if (pos==Kinematics::back_right) br_=f;

		return true;
	}

//-----------------------------------------------------------------------------

	bool Kinematics::UpdateRead(){
		fl_->UpdateRead();
		fr_->UpdateRead();
		bl_->UpdateRead();
		br_->UpdateRead();
		return true;
	}

//-----------------------------------------------------------------------------

	bool Kinematics::UpdateWrite(){
		if (mode_>0 && step_==3){
			if ( bl_->State()==Flipper::OFF && br_->State()==Flipper::OFF){
				mode_=0;
				
			}
		}
		if (mode_>0 && step_==2){
			if (bl_->GetMotor()->Velocity()<100 && br_->GetMotor()->Velocity()<100){
				if (fl_->State()==Flipper::OFF && fr_->State()==Flipper::OFF){
					bl_->ChangeMode(Flipper::DRIVE);
					br_->ChangeMode(Flipper::DRIVE);
					step_=3;
				}
			}
		}
		if (mode_>0 && step_==1){
			if (bl_->State()==Flipper::ON && br_->State()==Flipper::ON){
				if (fl_->GetMotor()->Velocity()<100 && fr_->GetMotor()->Velocity()<100){
					if (mode_==1){ 
						bl_->SetAngle(180);
						br_->SetAngle(180);
					}
					if (mode_==2){
						bl_->SetAngle(180);
						br_->SetAngle(180);

					}
					if (mode_==3){
						bl_->SetAngle(0);
						br_->SetAngle(0);
					}

					fl_->ChangeMode(Flipper::DRIVE);
					fr_->ChangeMode(Flipper::DRIVE);
					step_=2;

				}

			}

			
		}
		if (mode_>0 && step_==0){
			if (fl_->State()==Flipper::ON && fr_->State()==Flipper::ON){
				if (mode_==1){ 
					fl_->SetAngle(45);
					fr_->SetAngle(45);
				}
				if (mode_==2){
					fl_->SetAngle(0);
					fr_->SetAngle(0);

				}
				if (mode_==3){
					fl_->SetAngle(0);
					fr_->SetAngle(0);
				}
				bl_->ChangeMode(Flipper::FLIP);
				br_->ChangeMode(Flipper::FLIP);
				step_=1;

			}
		}
		fl_->UpdateWrite();
		fr_->UpdateWrite();
		bl_->UpdateWrite();
		br_->UpdateWrite();
		return true;
	}

//-----------------------------------------------------------------------------

	void Kinematics::SetFwdVel(int rotvel)
	{
		if (mode_==0){
			fl_->VelRef(-rotvel);
			fr_->VelRef(rotvel);
		}
	}

//-----------------------------------------------------------------------------

	void Kinematics::SetBckVel(int rotvel){
		if (mode_==0){
			bl_->VelRef(-rotvel);
			br_->VelRef(rotvel);
		}
	}

//-----------------------------------------------------------------------------

	void Kinematics::SetLftVel(int rotvel){
		if (mode_==0){
			bl_->VelRef(-rotvel);
			fl_->VelRef(-rotvel);
		}
	}

//-----------------------------------------------------------------------------

	void Kinematics::SetRgtVel(int rotvel){
		if (mode_==0){
			br_->VelRef( rotvel); 
			fr_->VelRef(rotvel);
		}
	}

//-----------------------------------------------------------------------------

	void Kinematics::SetVel(int rotvel,position pos){
		if (mode_==0){
			if (pos==Kinematics::front_left) fl_->VelRef(-rotvel);
			if (pos==Kinematics::front_right)fr_->VelRef(rotvel);
			if (pos==Kinematics::back_left) bl_->VelRef(-rotvel);
			if (pos==Kinematics::back_right) br_->VelRef(rotvel);
		}
	}

//-----------------------------------------------------------------------------

	bool Kinematics::SetState(mode m,position pos){
		if (mode_==0) {
			if (pos==this->front_left && m==this->flip) fl_->ChangeMode(Flipper::FLIP);
			if (pos==this->front_right && m==this->flip) fr_->ChangeMode(Flipper::FLIP);
			if (pos==this->back_left && m==this->flip) bl_->ChangeMode(Flipper::FLIP);
			if (pos==this->back_right && m==this->flip) br_->ChangeMode(Flipper::FLIP);
		
			if (pos==this->front_left && m==this->run) fl_->ChangeMode(Flipper::DRIVE);
			if (pos==this->front_right && m==this->run) fr_->ChangeMode(Flipper::DRIVE); 
			if (pos==this->back_left && m==this->run) bl_->ChangeMode(Flipper::DRIVE);
			if (pos==this->back_right && m==this->run) br_->ChangeMode(Flipper::DRIVE);
		}
		return true;
	}

//-----------------------------------------------------------------------------

	int Kinematics::WirelessData(char* data){
		int suma=0;
		suma=fl_->WirelessData(data);
		suma+=fr_->WirelessData(data+suma);
		suma+=bl_->WirelessData(data+suma);
		suma+=br_->WirelessData(data+suma);
		return suma;
	}

//-----------------------------------------------------------------------------
	
	bool Kinematics::SetMode(int mode){
		if (mode!=mode_){
			mode_=mode;
			step_=0;
			if (mode_>0){
				fl_->ChangeMode(Flipper::FLIP);
				fr_->ChangeMode(Flipper::FLIP);
			}
		}
	}
//=============================================================================

bool Sensors::Connect(void){
	int i;
	for (i=0;i<n_;i++){
		sensors_[i]->Connect();
	}
	return true;
}
	
//-----------------------------------------------------------------------------

bool Sensors::Disconnect(void){
	return true;
}

//-----------------------------------------------------------------------------

bool Sensors::Add(SensorPtr sen){
	sensors_[n_]=sen;

	n_++;
	return true;
}

//-----------------------------------------------------------------------------

bool Sensors::UpdateRead()
{
	int i;
	int sum=0,rez;
	for (i=0;i<n_;i++){
		rez=sensors_[i]->UpdateRead();
		sum=sum ||rez ;
	}
	return sum;
}

//-----------------------------------------------------------------------------

int Sensors::WirelessData(char* data,int sensor)
{
	data[0]=sensor;
	int i,br,k=0,pod;
	for (i=0;i<n_;i++){
		br=sensors_[i]->GetNumberOfWords();
		if (k+br>sensor){
			pod=sensors_[i]->GetValue(sensor-k);
			data[0]+=sensors_[i]->Status()*32;
			data[1]=(unsigned char)(pod/256);
			data[2]=(unsigned char)(pod%256);
			break;
		}
		else k=k+br;
	}
	return 3;
}
	
//=============================================================================


bool Power::Connect(){
	pComm_->Open();
	int id_=50;
	char data[8];
	data[0]=1;
	CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
	if (pComm_->Send(msg)){
		if (pComm_->Receive( msg, 1000 )){
			open_=true;
			return true;
		}
	}	
	return false;
}

//-----------------------------------------------------------------------------
	bool Power::SetStateDrive(bool drive){
		drivem_=false;
		if (drive_==false && drive==true){
			drivem_=true;
		}
		drive_=drive;
		return true;
	}
//-----------------------------------------------------------------------------

	bool Power::SetStateCameras(bool cam){
		cam_=cam;
		return true;
	}
//-----------------------------------------------------------------------------

	bool Power::SetStateWireless(bool wir){
		wir_=wir;
		return true;
	}
//-----------------------------------------------------------------------------

	bool Power::SetStateSensors(bool sen){
		sen_=sen;
		return true;
	}
//-----------------------------------------------------------------------------

	bool Power::SetStateCameraSelect(bool camsel){
		camsel_=camsel;
		return true;
	}


//-----------------------------------------------------------------------------

bool Power::UpdateWrite(){
	pComm_->Open();
	int id_=50;
	char data[8];
	data[0]=3;
	data[1]=drive_ | 1;
	data[2]=cam_;
	data[3]=wir_;
	data[4]=sen_;
	data[5]=camsel_;
	CommMsg msg( id_, data, 6, pt::microsec_clock::local_time() );
	if (pComm_->Send(msg)){
		if (pComm_->Receive( msg, 1000 )){
			open_=true;
			return true;
		}
	}	
	return false;
	
}

//-----------------------------------------------------------------------------

bool Power::UpdateRead(){
	return true;
}

//=============================================================================

	bool RotateCamera::Connect(){
		pComm_->Open();
		int id_=501;
		char data[8];
		data[0]=1;
		data[1]=0;
		CommMsg msg( id_, data, 2, pt::microsec_clock::local_time() );
		if (pComm_->Send(msg)){
			if (pComm_->Receive( msg, 1000 )){
				open_=true;
				return true;
			}
		}	
		return false;

	}

//-----------------------------------------------------------------------------

	bool RotateCamera::UpdateWrite(){
		pComm_->Open();
		int id_=501;
		char data[8];
		data[0]=1;
		data[1]=ref_;
		ref_=0;
		CommMsg msg( id_, data, 2, pt::microsec_clock::local_time() );
		if (pComm_->Send(msg)){
			if (pComm_->Receive( msg, 20 )){
				open_=true;
				return true;
			}
		}	
		return false;
		
	}

}
