#include "../Communication/CommPrint.hpp"
#include "LinAct.hpp"
#include "../Communication/CommMsg.hpp"
#include <vatroslav/CanMsg.h>

namespace Vatroslav{
	namespace pt = boost::posix_time;

//=============================================================================

LinAct::LinAct(int id, CommPtr pComm) : status_( 4 ), 
											id_( id ), 
											pComm_( pComm )
{
	open_=false;
	set_state=0;
	position_=0;
	current_=0;
	slow_=0;
}

//=============================================================================

	bool LinAct::UpdateWrite(){
		if (open_==false) {
			slow_=0;
			return false;
		}
		char data[] = { 3,2,3,4,5,6,7,8};
	
		data[0]=1;
		data[1]=slow_;
		CommMsg msg( id_, data, 2, pt::microsec_clock::local_time() );
		if (set_state==1){
//			std::cout<<msg<<std::endl;
			Send(msg);
		} else if (set_state==2){
			data[0]=2;
			data[1]=slow_;
			msg.Data(data,2);
//			std::cout<<msg<<std::endl;
			Send(msg);
		} else if (set_state==3){
			data[0]=3;
			msg.Data(data,1);
//			std::cout<<msg<<std::endl;

			Send(msg);
		}
		if (set_state==21){
			data[0]=21;
			data[1]=1;
			data[2]=value_/256;
			data[3]=value_%256;
			msg.Data(data,4);
			Send(msg);

		}
		if (set_state==22){
			data[0]=21;
			data[1]=2;
			data[2]=value_/256;
			data[3]=value_%256;
			msg.Data(data,4);
			Send(msg);
		}
		if (set_state==23){
			data[0]=21;
			data[1]=5;
			data[2]=value_/256;
			data[3]=value_%256;
			msg.Data(data,4);
			Send(msg);
		}
		set_state=0;
		slow_=0;
		return true;
	}

//=============================================================================

	bool LinAct::UpdateRead(){
		if (open_==false) return false;
		char data[] = { 3};
	
		data[0]=4;
		CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
		if (Send(msg)){
			if (Receive( msg, 1000 )){
				current_=(unsigned char) msg.Data()[1]*256+(unsigned char)msg.Data()[2];
				position_=(unsigned char) msg.Data()[3]*256+(unsigned char)msg.Data()[4];
				status_=msg.Data()[5];
				std::cout<<"motorId: " << id_ << "	LinAct position: " << position_  << "	status: " << status_ << std::endl;


				return true;
			}
		}
		return false;
	}

//=============================================================================
	bool LinAct::Connect(){
		//pComm_->Open();
		char data[] = { 4};
		data[0]=4;
		CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
		if (Send(msg)){
			if (Receive( msg, 1000 )){
				if (msg.Size()>0) {
					status_=msg.Data()[0];
					open_=true;
					this->ReadConstants();
					return true;	
				}
			}
		}	
		open_=false;
		return false;
	}
//=============================================================================
	bool LinAct::Disconnect(){
		return true;
	}
	
//=============================================================================

	bool LinAct::Off(){
		set_state=1;
		slow_=0;
//		char data[] = { 1};
//		CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
		return true;//pComm_->Send(msg);
	}

//=============================================================================

	bool LinAct::OffSlow(){
		set_state=1;
		slow_=1;
//		char data[] = {2};
//		CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
		return true;//pComm_->Send(msg);
	}

//=============================================================================
	bool LinAct::On(){
		set_state=2;
		slow_=0;
//		char data[] = {2};
//		CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
		return true;//pComm_->Send(msg);
	}

//=============================================================================

	bool LinAct::OnSlow(){
		set_state=2;
		slow_=1;
//		char data[] = {2};
//		CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
		return true;//pComm_->Send(msg);
	}

//=============================================================================

	bool LinAct::MotorOff(){
		set_state=3;
		return true;
	}

//=============================================================================

	bool LinAct::ReadConstants(){
		if (open_==false) return false;
		set_state=20;
		char data[] = {20};
		CommMsg msg( id_, data, 1, pt::microsec_clock::local_time() );
		if (Send(msg)){
			if (Receive( msg, 1000 )){
				if (msg.Size()>6) {
					if (msg.Data()[0]==20){
//						std::cout<<msg<<std::endl;
						start_pos_=(unsigned char) msg.Data()[1]*256+ (unsigned char)msg.Data()[2];
						end_pos_=(unsigned char) msg.Data()[3]*256+ (unsigned char)msg.Data()[4];
						max_curr_=(unsigned char) msg.Data()[5]*256+ (unsigned char)msg.Data()[6];
						std::cout << "Konstante lin act, motorId" << id_ << ": star, end , max" << start_pos_  << " " << end_pos_ << " " << max_curr_ << std::endl;
					}
					return true;
				}
			}
		}
		return false;
	}


//=============================================================================

	bool LinAct::ChangeConstant(LinAct::constant con, int value){
		if (open_==false ) return false;
		if (con==start_position){
			value_=value;
			set_state=21;			
		}
		if (con==end_position){
			value_=value;
			set_state=22;			
		}
		if (con==max_current){
			value_=value;
			set_state=23;			
		}
		return true;
	}

//=============================================================================

	LinAct::status_enum LinAct::Status(){
		if (this->status_==1) return changing_state_off; else
		if (this->status_==2) return changing_state_on; else
		if (this->status_==3) return on; else 
		if (this->status_==4) return off; else
		if (this->status_==5) return motor_off; else 
		return other;
	}

//=============================================================================

	int LinAct::GetPosition()
	{
		return position_;
	}

//=============================================================================

	int LinAct::GetCurrent()
	{
		return current_;
	}

//=============================================================================


	int LinAct::GetConstant(constant con){
		if (con==start_position){
			return start_pos_;
		}
		if (con==end_position){
			return end_pos_;
		}
		if (con==max_current){
			return max_curr_;
		}
}

}
