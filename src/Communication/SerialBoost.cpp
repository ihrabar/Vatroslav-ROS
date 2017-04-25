/*! \file	SerialBoost.cpp

 */

#include "SerialBoost.hpp"

namespace Vatroslav
{

//=============================================================================

SerialBoost::SerialBoost() : open_(false), par_( CommPar( CommPar::NONE ) )
{
}

//=============================================================================

SerialBoost::~SerialBoost()
{
	if ( open_ )
	{
		this->Close();
	}
}

//=============================================================================

/* virtual */
bool SerialBoost::Open( const CommPar& par )
{
	typedef boost::asio::serial_port_base spb;

	bool success = false;

	if (!open_)
	{
		try
		{
			pio_.reset( new boost::asio::io_service() );
			pserial_.reset( new boost::asio::serial_port( *pio_, par.Port()) );

			pserial_->set_option(spb::baud_rate(par.Baudrate()));
			pserial_->set_option(spb::character_size(par.DataBits()));

			switch (par.StopBits())
			{
			case 0:
				pserial_->set_option(spb::stop_bits(spb::stop_bits::one));
				break;
			case 1:
				pserial_->set_option(spb::stop_bits(spb::stop_bits::onepointfive));
				break;
			case 2:
				pserial_->set_option(spb::stop_bits(spb::stop_bits::two));
				break;
			}

			switch (par.Parity())
			{
			case 0:
				pserial_->set_option(spb::parity(spb::parity::none));
				break;
			case 1:
				pserial_->set_option(spb::parity(spb::parity::odd));
				break;
			case 2:
				pserial_->set_option(spb::parity(spb::parity::even));
				break;
			}
		
			//! Set flow control to none
			pserial_->set_option(spb::flow_control(spb::flow_control::none));

			//! 
		}
		catch (boost::system::system_error& ErrorCode_)
		{
				return success;
		}

		open_ = true;
		par_ = par;
		success = true;
		
	}
	else
	{
		// The communication channel is already open, check parameters
		if ( par_ == par )
		{
			success = true;
		}
	}				

	return success;
}

//=============================================================================

/* virtual */
bool SerialBoost::Close( void )
{
	if (open_)
	{
		pserial_->close();
		open_ = false;
	}
	return !open_;
}


//=============================================================================

/* virtual */
bool SerialBoost::Send( const CommMsg& msg )
{
	bool success = false;

	unsigned short Index = 0;
	unsigned short Sub_Index = 0;
	unsigned short NodeID = 0;
	unsigned long dwData = 0;

	//! Clear all error codes and buffer variable
	errorCode_ = 0;
	errorCodeCommunication_ = 0;
	read_ = 0;
	
	//! Decode needed variables from sent message 
	Index = (msg.Data()[1] & 0xFF) | ((msg.Data()[0] & 0xFF) << 8);
	Sub_Index = msg.Data()[2] & 0xFF;
	NodeID = msg.Data()[3] & 0xFF;
	dwData = ((((msg.Data()[7] & 0xFF) | (((msg.Data()[6] & 0xFF) << 8) & 0xFF00))
		| (((msg.Data()[5] & 0xFF) << 16)) & 0xFF0000) | (((msg.Data()[4] & 0xFF) << 24) & 0xFF000000));

	if (open_)
	{
		if (msg.Data()[8] > 0) // write to EPOS device
		{
			errorCode_ = WriteObject(Index, Sub_Index, NodeID, dwData);
			errorCodeCommunication_ = errorCode_;
		}
		else // read from EPOS device
		{
			read_ = ReadObject(Index, Sub_Index, NodeID, &errorCode_, &errorCodeCommunication_);
		}
	}

	success = ((errorCode_) == 0) || ((errorCodeCommunication_) == 0);
	
	return success;
}

//=============================================================================

/* virtual */
bool SerialBoost::Receive( CommMsg& msg, unsigned short timeout )
{
	char Result[8]={0};
	
	if (open_)
	{
		Result[0] = (read_ & 0xFF);
		Result[1] = ((read_ >> 8) & 0xFF);
		Result[2] = ((read_ >> 16) & 0xFF);
		Result[3] = ((read_ >> 24) & 0xFF);
		Result[4] = (errorCodeCommunication_ & 0xFF);
		Result[5] = ((errorCodeCommunication_ >> 8) & 0xFF);
		Result[6] = ((errorCodeCommunication_ >> 16) & 0xFF);
		Result[7] = ((errorCodeCommunication_ >> 24) & 0xFF);
		msg.Data(Result, 8);
		msg.Timestamp( boost::posix_time::microsec_clock::local_time() );
		return true;
	}
	return false;
}

//=============================================================================
unsigned short SerialBoost::CalcFieldCRC(const unsigned short* pDataArray, unsigned short numberOfWords)
{
   unsigned short shifter, c;
   unsigned short carry;
   unsigned short CRC = 0;

   //Calculate pDataArray Word by Word

   while(numberOfWords--)
   {
        shifter = 0x8000;
        c = *pDataArray++;
        do
        {
           carry = CRC & 0x8000; 
           CRC <<= 1;
           if(c & shifter) CRC++;
           if(carry) CRC ^= 0x1021;
           shifter >>= 1;
        } while(shifter);
   }
  return CRC;
}

//=============================================================================

unsigned long SerialBoost::WriteObject(unsigned short Index, unsigned short Sub_Index, unsigned short NodeID, unsigned long dwData)
{
	unsigned long Result=0;

	if (Result = WriteObject_SendData(Index, Sub_Index, NodeID, dwData))
		return Result;

	if (Result = WriteObject_ReceiveData())
		return Result;

	return Result;
}

//=============================================================================

unsigned long SerialBoost::WriteObject_SendData(unsigned short Index, unsigned short Sub_Index, unsigned short NodeID, unsigned long dwData)
{
	unsigned char OpCode = WRITEOBJECT_OPCODE;
	unsigned char Len_1 = WRITEOBJECT_LEN_1;
	unsigned char readyAck=ACK_FALSE, endAck=ACK_FALSE;
	unsigned short *WordsToWrite=NULL;
	unsigned short WordsCRC[6] = {0};
	unsigned short wCRC=0;

	if (!WriteToSerial( &OpCode, 1)) return ERROR_WRITING_BYTE;
	readyAck = WaitToReadByteFromSerial();
	if (readyAck == ACK_FALSE) return FAIL_WRITE_RDY_ACK;
	if (!WriteToSerial( &Len_1, 1)) return ERROR_WRITING_BYTE;
	WordsToWrite = new unsigned short[4];  // 4 Words to write
	memset(WordsToWrite, 0, 8);

	WordsToWrite[0] = Index;
	WordsToWrite[1] = Sub_Index | (NodeID << 8);
	WordsToWrite[2] = ((unsigned short) (dwData & 0x0000FFFF));
	WordsToWrite[3] = ((unsigned short) (dwData >> 16));

	if (!WriteWordsToSerial(WordsToWrite, 4)) return ERROR_WRITING_WORDS;


	WordsCRC[0] = WRITEOBJECT_LEN_1 | (((unsigned short) WRITEOBJECT_OPCODE) << 8);
	WordsCRC[1] = WordsToWrite[0];
	WordsCRC[2] = WordsToWrite[1];
	WordsCRC[3] = WordsToWrite[2];
	WordsCRC[4] = WordsToWrite[3];
	WordsCRC[5] = 0x0;
	
	wCRC = CalcFieldCRC(WordsCRC, 6);
	if (!WriteWordsToSerial(&wCRC, 1)) return ERROR_WRITING_WORDS;    // CRC size = 1 word

	endAck = WaitToReadByteFromSerial();
	if (endAck == ACK_FALSE) return FAIL_WRITE_END_ACK;

	delete [] WordsToWrite;

	return 0;
}

//=============================================================================

unsigned long SerialBoost::WriteObject_ReceiveData(void)
{
	unsigned char OpCode=WRITEOBJECT_OPCODE_RESPONSE;
	unsigned char readyAck=ACK_OK;
	unsigned char endAck=ACK_OK;
	unsigned char numberOfWords=1;
	unsigned short wCRC=0, wCRC_Calc=0;
	unsigned short *WordsCRC=NULL;
	unsigned short *WordsReaded=NULL;
	unsigned long ResultErrorCode=0;
	int i;

	OpCode = WaitToReadByteFromSerial();
	if (OpCode != WRITEOBJECT_OPCODE_RESPONSE) return WRONG_OPCODE_WRITE;
	if (!WriteToSerial( &readyAck, 1)) return ERROR_WRITING_BYTE;
	numberOfWords = WaitToReadByteFromSerial();
	WordsReaded = new unsigned short[numberOfWords+1];
	memset(WordsReaded, 0, (numberOfWords+1)*2);
	WaitToReadWordsFromSerial(WordsReaded, (unsigned short) (numberOfWords+1) );

	WaitToReadWordsFromSerial(&wCRC, 1);  // CRC size = 1 word

	WordsCRC = new unsigned short[numberOfWords+3];
	WordsCRC[0] = numberOfWords;
	for (i=0; i<= numberOfWords+1; i++)
	{
		WordsCRC[i+1]=WordsReaded[i];
	}
	WordsCRC[numberOfWords+2] = 0x0;

	wCRC_Calc = CalcFieldCRC(WordsCRC, numberOfWords+3);
	if (!(wCRC_Calc-wCRC) )
		endAck = ACK_OK;
	else
		endAck = ACK_FALSE;
	
	if (!WriteToSerial( &endAck, 1)) return ERROR_WRITING_BYTE;

	if (numberOfWords == WRITEOBJECT_LEN_1_RESPONSE) //Len_1 = 1
		ResultErrorCode = WordsReaded[0] | (0xFFFF0000 & (WordsReaded[1] << 16));
	else
		ResultErrorCode = WordsReaded[0];

	delete [] WordsCRC;
	delete [] WordsReaded;

	return ResultErrorCode;
}

//=============================================================================

bool SerialBoost::WriteToSerial(const unsigned char* Buf, std::streamsize N)
{
	int Result;
	
	Result = boost::asio::write(*pserial_, boost::asio::buffer(Buf, N));

	if (Result == N)
		return true;
	else
		return false;
}

//=============================================================================

bool SerialBoost::WriteWordsToSerial(const unsigned short *pWords, unsigned short numberOfWords)
{
	unsigned char* buffer = new unsigned char[numberOfWords*2];
	int i = 0, j = -1;

	for (i=0; i<=(numberOfWords-1); i++)
	{	
		buffer[++j] = (char) (pWords[i] & 0xFF);	
		buffer[++j] = (char) ((pWords[i] >> 8) & 0xFF);	
	}
	if (WriteToSerial( buffer, numberOfWords*2))
	{
		delete [] buffer;
		return true;
	}
	else
	{
		delete [] buffer;
		return false;
	}

	return true;
}

//=============================================================================

unsigned char SerialBoost::ReadByteFromSerial(void)
{
	char ReadedData = 0;
	int Result;

	Result = boost::asio::read(*pserial_, boost::asio::buffer(&ReadedData, 1));
	
	return ReadedData;
}

//=============================================================================

unsigned char SerialBoost::WaitToReadByteFromSerial(void)
{
	unsigned char ByteReaded = 255;
	const unsigned long TIMEOUT = 50;
	boost::posix_time::ptime t_start_, t_stop_;
	boost::posix_time::time_duration dur;
	int pass = 0;

	t_start_ = boost::posix_time::microsec_clock::local_time();

	for(;;)
	{
		ByteReaded = ReadByteFromSerial();
		
		t_stop_ = boost::posix_time::microsec_clock::local_time();
		dur = t_stop_ - t_start_;
		pass++;
		
		if ( ((dur.total_milliseconds() >= TIMEOUT) || (ByteReaded != 0)) || (pass != 0) ) break;
	}

	return ByteReaded;
}

//=============================================================================

void SerialBoost::WaitToReadWordsFromSerial(unsigned short *pReadedWords, unsigned short numberOfWords)
{
	unsigned short ByteReaded1 = 0, ByteReaded2 = 0;
	int i;
	
	for (i=0; i<=(numberOfWords-1); ++i)
	{
		ByteReaded2 = WaitToReadByteFromSerial();
		ByteReaded1 = WaitToReadByteFromSerial();

		pReadedWords[i] = ByteReaded2 | (ByteReaded1 << 8);
	}
	return;
}


//=============================================================================

unsigned long SerialBoost::ReadObject_SendData(unsigned short Index, unsigned short Sub_Index, unsigned short NodeID)
{
	unsigned char OpCode = READOBJECT_OPCODE;
	unsigned char Len_1 = READOBJECT_LEN_1;
	unsigned char readyAck, endAck;
	unsigned short *WordsToWrite;
	unsigned short WordsCRC[4] = {0};
	unsigned short wCRC;

	if (!WriteToSerial( &OpCode, 1)) return ERROR_WRITING_BYTE;
	readyAck = WaitToReadByteFromSerial();
	if (readyAck == ACK_FALSE) return FAIL_READ_RDY_ACK;
	if (!WriteToSerial( &Len_1, 1)) return ERROR_WRITING_BYTE;
	WordsToWrite = new unsigned short[2];

	WordsToWrite[0] = Index;
	WordsToWrite[1] = Sub_Index | (NodeID << 8);

	if (!WriteWordsToSerial(WordsToWrite, 2)) return ERROR_WRITING_WORDS;

	WordsCRC[0] = READOBJECT_LEN_1 | (((unsigned short) READOBJECT_OPCODE) << 8);
	WordsCRC[1] = Index;
	WordsCRC[2] = Sub_Index | (NodeID << 8);
	WordsCRC[3] = 0x0000;
	
	wCRC = CalcFieldCRC(WordsCRC, 4);
	if (!WriteWordsToSerial(&wCRC, 1)) return ERROR_WRITING_WORDS;    // CRC size = 1 word

	endAck = WaitToReadByteFromSerial();
	if (endAck == ACK_FALSE) return FAIL_READ_END_ACK;

	delete [] WordsToWrite;

	return 0;
}

//=============================================================================

unsigned long SerialBoost::ReadObject_ReceiveData(unsigned short **pData, unsigned char *pnumberOfWords)
{
	unsigned char OpCode=READOBJECT_OPCODE_RESPONSE;
	unsigned char readyAck=ACK_OK;
	unsigned char endAck;
	unsigned short wCRC, wCRC_Calc;
	unsigned short *WordsCRC;
	unsigned short *WordsReaded=NULL;
	int i;

	OpCode = WaitToReadByteFromSerial();
	if (OpCode != READOBJECT_OPCODE_RESPONSE) return WRONG_OPCODE_READ;

	if (!WriteToSerial( &readyAck, 1)) return ERROR_WRITING_BYTE;
	*pnumberOfWords = WaitToReadByteFromSerial();  // Len_1
	WordsReaded = new unsigned short[*pnumberOfWords+1];
	memset(WordsReaded, 0, (*pnumberOfWords+1)*2);
	WaitToReadWordsFromSerial(WordsReaded, (unsigned short) (*pnumberOfWords+1) );
	*pData = WordsReaded;

	WaitToReadWordsFromSerial(&wCRC, 1);  // CRC size = 1 word

	WordsCRC = new unsigned short[*pnumberOfWords+3];
	WordsCRC[0] = *pnumberOfWords;
	for (i=0; i<= *pnumberOfWords+1; i++)
	{
		WordsCRC[i+1]=WordsReaded[i];
	}
	WordsCRC[*pnumberOfWords+2] = 0x0;

	wCRC_Calc = CalcFieldCRC(WordsCRC, *pnumberOfWords+3);
	if (!(wCRC_Calc-wCRC) )
		endAck = ACK_OK;
	else
		endAck = ACK_FALSE;
	
	if (!WriteToSerial( &endAck, 1)) return ERROR_WRITING_BYTE;

	delete [] WordsCRC;

	return 0;
}

//=============================================================================

unsigned long SerialBoost::ReadObject(unsigned short Index, unsigned short Sub_Index, unsigned short NodeID, unsigned long *pdwErrorCode, unsigned long *pdwErrorCodeCommunication)

{
	unsigned short *WordsReaded=NULL;
	unsigned char numberOfWords;
	unsigned long dwFlag;
	unsigned long dwReaded=0;

	*pdwErrorCode = 0x0;
	*pdwErrorCodeCommunication = 0x0;

	*pdwErrorCode = ReadObject_SendData(Index, Sub_Index, NodeID);
	if (*pdwErrorCode != 0) return 0;
	*pdwErrorCode = ReadObject_ReceiveData((unsigned short**) &WordsReaded, &numberOfWords);

	dwFlag = (unsigned long) WordsReaded[1];
	*pdwErrorCodeCommunication = (unsigned long) WordsReaded[0] | (dwFlag << 16);

	dwFlag = (unsigned long) WordsReaded[3];
	dwReaded = (unsigned long) WordsReaded[2] | (dwFlag << 16);

	delete [] WordsReaded;

	return dwReaded;
}

//=============================================================================

}
