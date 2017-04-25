/*! \file	SerialBoost.cpp

 */

#include "WirelessVIV.hpp"

namespace Vatroslav{
//=============================================================================


WirelessVIV::~WirelessVIV()
{
	if ( open_ )
	{

		this->Close();
	}
}

//=============================================================================

bool WirelessVIV::Open( std::string port )
{	
	 serial = new TimeoutSerial(port,9600);
	 serial->setTimeout(boost::posix_time::milliseconds(50));
	 open_=true;
		
	return open_;
}

//=============================================================================

bool WirelessVIV::Close( void )
{
	if(open_)
	{
	}
	return 0;
}

//-----------------------------------------------------------------------------

unsigned char WirelessVIV::UtilCalcCRC8(const unsigned char* pbyBuffer, unsigned int nLength)
{
  static const unsigned char CRC8_TABLE[16] =
  {
    0x00, 0x1C, 0x38, 0x24,
    0x70, 0x6C, 0x48, 0x54,
    0xE0, 0xFC, 0xD8, 0xC4,
    0x90, 0x8C, 0xA8, 0xB4
  };

  static const unsigned char x0F = 0x0F;

  unsigned char byCrc = 0;
  unsigned int nIndex;

  for(nIndex = 0; nIndex < nLength; nIndex++, pbyBuffer++)
  {
    // Compute low nibble.
    byCrc = ((byCrc >> 4) & x0F) ^ CRC8_TABLE[byCrc & x0F] ^ CRC8_TABLE[*pbyBuffer & x0F];

    // Compute high nibble.
    byCrc = ((byCrc >> 4) & x0F) ^ CRC8_TABLE[byCrc & x0F] ^ CRC8_TABLE[(*pbyBuffer >> 4) & x0F];  
  }

  return byCrc;
}

//-----------------------------------------------------------------------------

bool WirelessVIV::SendMessage( )
{
	static int br=0;
	char OutputBuff[20]={0};
	int flag = 48;
	int suma=0;
	if(open_==true)
	{
		if (br>20) br=0;
		suma+=sen_->WirelessData(OutputBuff,br);
		br++;

		suma+=kin_->WirelessData(OutputBuff+suma);
		OutputBuff[suma]=UtilCalcCRC8((const unsigned char*)OutputBuff, suma);
		suma++;
		OutputBuff[suma]=(unsigned char)255;
		suma++;
		OutputBuff[suma]=(unsigned char)'\0';
		printf("Velicina poruke je %d\n",suma);

/*		OutputBuff[0] = (unsigned char)48;
//		OutputBuff[1] = (unsigned char)lSpeed;
//		OutputBuff[2] = (unsigned char)rSpeed;
		OutputBuff[3] = UtilCalcCRC8((const BYTE*)OutputBuff, 3);
		OutputBuff[4]=  (unsigned char)255;
		OutputBuff[5]=  '\0';
		*/
		printf("%d ",OutputBuff[0]%32);
		int j,t;
		for(int i=0;i<suma;i++)
		{
			serial->write(&OutputBuff[i],1);
			
			if (i>0) printf("%d ",OutputBuff[i]);
			for (j=0;j<1000;j++){
				t++;
			}
		
		}
		printf("\n");
		return true;
	}
	else return false;
	
}

//-----------------------------------------------------------------------------

bool WirelessVIV::Parser()
{
	int i;
	//ako je primljena struktura veca od 12


	if(InBuff.size()>=4)
	{
		//check CRC
		char message[15];
		for(i=InBuff.size()-5;i<InBuff.size()-1;i++) message[i-(InBuff.size()-5)] = InBuff.at(i);

//		getchar();
		message[4]='\0';
		BYTE CRC = UtilCalcCRC8((const BYTE*)message,3);
		if(CRC == (BYTE)message[3])
		{
			printf("Primio uspjesno... %d %d %d\r\n",message[0],message[1],message[2]);
			pow_->SetStateDrive(((unsigned char) message[0]/128)==0);
			pow_->SetStateCameras(((unsigned char) message[0]%128/64)==0);
			pow_->SetStateWireless(((unsigned char) message[0]%64/32)==0);
			pow_->SetStateSensors(((unsigned char) message[0]%32/16)==0);
			message[0]=(unsigned char) message[0]%16;
			switch (message[0]){
				case 0:
					kin_->SetLftVel(message[1]);
					kin_->SetRgtVel(message[2]);
					break;
				case 1:
					kin_->SetVel(message[1],Kinematics::front_left);
					kin_->SetVel(message[2],Kinematics::front_right);
					break;
				case 2:
					kin_->SetVel(message[1],Kinematics::back_left);
					kin_->SetVel(message[2],Kinematics::back_right);
					break;
				case 3:
					if (message[1]==0) {
						kin_->SetState(Kinematics::run,Kinematics::front_left);
						kin_->SetState(Kinematics::run,Kinematics::front_right);
					}
					if (message[1]==1) {
						kin_->SetState(Kinematics::run,Kinematics::front_left);
						kin_->SetState(Kinematics::flip,Kinematics::front_right);
					}
					if (message[1]==2) {
						kin_->SetState(Kinematics::flip,Kinematics::front_left);
//						kin_->SetState(Kinematics::run,Kinematics::front_right);
					}
					if (message[1]==3) {
						kin_->SetState(Kinematics::flip,Kinematics::front_left);
						kin_->SetState(Kinematics::flip,Kinematics::front_right);
					}
					if (message[2]==0) {
						kin_->SetState(Kinematics::run,Kinematics::back_left);
						kin_->SetState(Kinematics::run,Kinematics::back_right);
					}
					if (message[2]==1) {
						kin_->SetState(Kinematics::run,Kinematics::back_left);
						kin_->SetState(Kinematics::flip,Kinematics::back_right);
					}
					if (message[2]==2) {
						kin_->SetState(Kinematics::flip,Kinematics::back_left);
						kin_->SetState(Kinematics::run,Kinematics::back_right);
					}
					if (message[2]==3) {
						kin_->SetState(Kinematics::flip,Kinematics::back_left);
						kin_->SetState(Kinematics::flip,Kinematics::back_right);
					}
					break;
				case 4:
					kin_->Connect();
				break;
				case 5:
					kin_->SetMode(message[1]);
				break;
				case 13:
					rot_->SetRef(message[1]);
				break;
				case 14:
					pow_->SetStateCameraSelect(message[1]==1);
				break;
				default:
					break;
			}
			for(int i=0;i<=InBuff.size();i++) InBuff.erase(InBuff.begin());
			return true;
		}

	}
	if (InBuff.size()>=4){
		while (InBuff.size()>4){
			InBuff.erase(InBuff.begin());
		}
	}
	return false;
	
}

//-----------------------------------------------------------------------------

bool WirelessVIV::ReceiveMessage(int timeout){
	boost::posix_time::ptime beg=boost::posix_time::microsec_clock::local_time();
	boost::posix_time::ptime now;
	boost::posix_time::time_duration dur;
	serial->setTimeout(boost::posix_time::milliseconds(timeout));
	int milli;
	while (this->Receive()){
		if (this->Parser()) {
			return true;
		}
		now=boost::posix_time::microsec_clock::local_time();
		dur=now-beg;
		milli=timeout-dur.total_milliseconds();
		serial->setTimeout(boost::posix_time::milliseconds(milli));
	}
	printf("ERROR\n");
	return false;
}

//-----------------------------------------------------------------------------

bool WirelessVIV::Receive()
{
	std::string limiter;
	limiter.push_back((unsigned char)255);
	try
	{
		std::string inputData;
		inputData = serial->readStringUntil(limiter);
		if(inputData.size()>0)
		{
			for(int i=0;i<inputData.size();i++)
			{
				InBuff.push_back(inputData.at(i));
			}
			InBuff.push_back((unsigned char)255);
		}
		return true;

	}
	catch(boost::system::system_error& e)
    {
        return false;
    }
	return false;

}

//-----------------------------------------------------------------------------

int WirelessVIV::EmptyBuffer(){
	int brojac=0;
	serial->setTimeout(boost::posix_time::milliseconds(30));
	while(this->Receive()) brojac++;
	return brojac;
}

//=============================================================================


int WirelessMessage::GetMessage(unsigned char ** data){
	*data=data_;
	return n_;
}

//-----------------------------------------------------------------------------

void WirelessMessage::SetMessage(unsigned char *data,int n){
	int i;
	for (i=0;i<n;i++){
		data_[i]=data[i];
	}
	n_=n;
}


//=============================================================================



//=============================================================================

//=============================================================================

}
