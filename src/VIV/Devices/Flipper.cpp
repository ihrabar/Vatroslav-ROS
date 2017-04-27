/*!	\file	Flipper.cpp
	\brief	Flipper member definitions.

 */

#include <cmath>
#include <cstdio>
#include <iostream>

#include "Flipper.hpp"

namespace Vatroslav
{

//=============================================================================

/* virtual */
bool Flipper::Connect( void )
{
	bool result =  p_motor_->Connect( );
	bool resultl = p_linact_->Connect( );
	if (result) printf("Motor connected\n");
	if (resultl) printf("Linear connected\n");

	p_linact_->ReadConstants();
	p_motor_->Params().MotionParams().SetProfileVelocity(1000);
	p_motor_->Params().MotionParams().SetProfileAcceleration(5000);
	p_motor_->VelocityRef(0);
	p_motor_->UpdateMotionParWrite();
	p_motor_->UpdateWrite();

	// Do an update to sync the software with the actual device.
	this->UpdateRead( );

	return (result && resultl);
}

//-----------------------------------------------------------------------------

/* virtual */
bool Flipper::Disconnect( void )
{
	v_ref_ = 0;
	/*!
		Do an update to stop the flipper before disconnecting.
		\todo Depending on how the flipper is actually going to work,
		this might become less trivial.
     */
	this->UpdateWrite( );
	bool result = ( p_motor_->Disconnect( ) && p_linact_->Disconnect( ) );
	
	return result;
}

//-----------------------------------------------------------------------------

/* virtual */
bool Flipper::UpdateRead( void )
{
	p_linact_->UpdateRead();
	if (p_linact_->Status()==LinAct::on) state_=this->ON;
	if (p_linact_->Status()==LinAct::off) state_=this->OFF;
	if (p_linact_->Status()==LinAct::changing_state_on) state_=this->CHANGING;
	if (p_linact_->Status()==LinAct::changing_state_off) state_=this->CHANGING;
	
	
	p_motor_->UpdateRead();
	p_motor_->UpdateAnalogInputs();
	return true;
}

//-----------------------------------------------------------------------------

bool Flipper::UpdateWrite( void )
{
	boost::posix_time::ptime now=boost::posix_time::microsec_clock::local_time();
	boost::posix_time::time_duration dur;
	int duration;
	if (flag_==1){
		if (p_linact_->Status()==LinAct::on){
			if (m_==FLIP){
				p_motor_->VelocityRef(0);
				flag_=0;
				time_=now;
			}
		}
		if (p_linact_->Status()==LinAct::off){
			if (m_==DRIVE){
				p_motor_->VelocityRef(0);
				flag_=0;
				time_=now;
			}
		}
		if (flag_==1){
/*			if (m_==FLIP){
				if (p_linact_->GetConstant(LinAct::end_position)-p_linact_->GetPosition()<40){
					p_linact_->OnSlow();
				}
			}
			if (m_==DRIVE){
				if (p_linact_->GetPosition()-p_linact_->GetConstant(LinAct::start_position)<40){
					p_linact_->OffSlow();
				}
			}*/
			p_motor_->VelocityRef(50);
			if (p_motor_->Current()>3000) p_motor_->VelocityRef(-20);
			dur=now-time_;
			duration=dur.total_seconds();
			if (duration>4){
				p_motor_->VelocityRef(-70);
			}
			if (duration>8) {
				p_motor_->VelocityRef(70);
			}
			if (duration>12){
				if (m_==FLIP){
					this->ChangeMode(DRIVE);
				}
				else if (m_==DRIVE)
				{
					this->ChangeMode(FLIP);
				}
			}
		}
	}
	p_linact_->UpdateWrite();
	if (motormod_>0){
		this->SetAngle(angle_);
	}
	p_motor_->UpdateWrite();
	if (motormod_==0) {
		p_motor_->VelocityRef(0); 
	}
	return true;
}

//=============================================================================

void Flipper::VelRef( int v )
{
	if (motormod_==1){
		motormod_=0;
		p_motor_->Params().MotionParams().SetMotorOperationMode(PROFILE_VELOCITY_MODE);
		p_motor_->UpdateMotionParWrite();
	}
	v_ref_=v;
	
	if (p_linact_->Status()==LinAct::on) {
		if (flag_==0) p_motor_->VelocityRef(v*MAX_SPEED_FLIP/100);
	}
	if (p_linact_->Status()==LinAct::off) {
		if (flag_==0) p_motor_->VelocityRef(v*MAX_SPEED_DRIVE/100);
	}
}

//-----------------------------------------------------------------------------

void Flipper::ChangeMode(Mode m)
{
	m_=m;
	printf("%d %d\n",(int) m, (int)(p_linact_->Status()));
	if (m==DRIVE) printf("Drive "); else if (m==this->FLIP) printf("Flip ");
	if (p_linact_->Status()==LinAct::off) printf ("off");
	else if (p_linact_->Status()==LinAct::on) printf ("on");
	else if (p_linact_->Status()==LinAct::changing_state_on) printf ("ch on");
	else if (p_linact_->Status()==LinAct::changing_state_off) printf ("ch off");
	else printf("other");
	printf("\n");
	if (m==this->DRIVE  && p_linact_->Status()!=LinAct::off) 
	{	
		flag_=1;
		time_=boost::posix_time::microsec_clock::local_time();
		p_motor_->VelocityRef(70);
		p_linact_->Off();
	}
	if (m==this->FLIP && (p_linact_->Status())!=(LinAct::on)) {
		flag_=1;
		time_=boost::posix_time::microsec_clock::local_time();

		p_motor_->VelocityRef(70);
		p_linact_->On();
	}
}

//-----------------------------------------------------------------------------


int Flipper::WirelessData(char* data)
{
	data[0]=0;
	data[1]=1;
	int pos=(p_linact_->GetPosition()-p_linact_->GetConstant(LinAct::start_position))*100/(p_linact_->GetConstant(LinAct::end_position)-p_linact_->GetConstant(LinAct::start_position)+1);
	if (p_linact_->Status()==LinAct::on) {data[0]=0;data[1]=(unsigned char) (p_angle_->GetValue(1)/2);}
	if (p_linact_->Status()==LinAct::off) {data[0]=1;data[1]=(char) (p_motor_->Velocity()*100/MAX_SPEED_DRIVE);}
	if (p_linact_->Status()==LinAct::changing_state_on) {data[0]=2;data[1]=(char) (pos);}
	if (p_linact_->Status()==LinAct::changing_state_off) {data[0]=3;data[1]=(char) (100-pos); }
	data[0]<<=5;
//	if (p_motor_->Status().Id()==Motor::ERROR) data[0]+=1;
	return 2;	
}

//-----------------------------------------------------------------------------

bool Flipper::SetAngle(int angle){
		if (motormod_==0){
			p_motor_->Params().MotionParams().SetMotorOperationMode(PROFILE_POSITION_MODE);
			p_motor_->UpdateMotionParWrite();

		} 
		motormod_=1;
		angle_=angle;
		int kut,raz;
		kut=p_angle_->GetValue(0);
		raz=angle-kut;
		if (raz<-180)  raz=raz+360;
		if (raz>180) raz-=360;
		if (kut<270 && raz>0 && kut+raz>270) raz-=360;
		if (kut>270 && raz<0 && kut+raz<270) raz+=360;
		if (kut<90 && raz<0 && kut+raz<-90) raz+=360; 
		
		if (abs(raz)>5)
		p_motor_->PositionRef(1350*raz*p_angle_->Sign());
		else
	 	p_motor_->PositionRef(0);
// 		p_motor_->UpdateWrite();
	return true;
}
//=============================================================================

}
