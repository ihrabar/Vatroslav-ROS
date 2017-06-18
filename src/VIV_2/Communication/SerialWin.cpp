/*! \file	SerialWin.cpp

 */

#ifdef VATROSLAV_SERIAL_WIN

#include "SerialWin.hpp"

namespace Vatroslav
{

//=============================================================================

SerialWin::SerialWin() : open_(false), par_( CommPar( CommPar::NONE ) ),
						hPort_( NULL ), dwErrorCode_(0), dwErrorCodeCommunication_(0)
{
}

//=============================================================================

SerialWin::~SerialWin()
{
	if ( open_ )
	{
		this->Close();
	}
}

//=============================================================================

std::wstring SerialWin::S2WS(const std::string& s)
{
	int len;
	int slength = (int)s.length() + 1;

	len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
	wchar_t* buf = new wchar_t[len];
	MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
	std::wstring r(buf);
	delete[] buf;
	
	return r;
}

//=============================================================================

/* virtual */
bool SerialWin::Open( const CommPar& par )
{
	bool success = false;
	
	if (!open_)
	{
		std::string s = par.Port();
		LPCWSTR lpszPortName = TEXT("");

#ifdef UNICODE
			std::wstring stemp = S2WS(s); // Temporary buffer is required
			lpszPortName = stemp.c_str();
#else
			lpszPortName = s.c_str();
#endif

		hPort_ = CreateFile ( lpszPortName, // Pointer to the name of the port
		  GENERIC_READ | GENERIC_WRITE,
						// Access (read-write) mode
		  0,            // Share mode
		  NULL,         // Pointer to the security attribute
		  OPEN_EXISTING,// How to open the serial port
		  0,            // Port attributes
		  NULL);        // Handle to port with attribute
						// to copy
		
		if ( hPort_ != NULL )
		{
			open_ = true;
			par_ = par;

			success = Configure( par );
			if ( success )
			{
				success = ConfigureTimeout( );
			}

		}
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

bool SerialWin::Configure( const CommPar& par )
{
	DCB PortDCB;
	DWORD Baudrate;
	BYTE Parity, StopBits;

	//! Determine Baudrate constant for Windows
	switch (par.Baudrate())
	{
	case 110:
		Baudrate = CBR_110;
		break;
	case 300:
		Baudrate = CBR_300;
		break;
	case 600:
		Baudrate = CBR_600;
		break;
	case 1200:
		Baudrate = CBR_1200;
		break;
	case 2400:
		Baudrate = CBR_2400;
		break;
	case 4800:
		Baudrate = CBR_4800;
		break;
	case 9600:
		Baudrate = CBR_9600;
		break;
	case 14400:
		Baudrate = CBR_14400;
		break;
	case 19200:
		Baudrate = CBR_19200;
		break;
	case 38400:
		Baudrate = CBR_38400;
		break;
	case 56000:
		Baudrate = CBR_56000;
		break;
	case 57600:
		Baudrate = CBR_57600;
		break;
	case 115200:
		Baudrate = CBR_115200;
		break;
	case 128000:
		Baudrate = CBR_128000;
		break;
	case 256000:
		Baudrate = CBR_256000;
		break;
	}
	
	//! Determine parity constant for Windows
	switch(par.Parity())
	{
	case 0:
		Parity = NOPARITY;
		break;
	case 1:
		Parity = ODDPARITY;
		break;
	case 2:
		Parity = EVENPARITY;
		break;
	case 3:
		Parity = MARKPARITY;
		break;
	case 4:
		Parity = SPACEPARITY;
		break;
	}

	//! Determine constant for no. stop bits for Windows
	switch(par.StopBits())
	{
	case 0:
		StopBits = ONESTOPBIT;
		break;
	case 1:
		StopBits = ONE5STOPBITS;
		break;
	case 2:
		StopBits = TWOSTOPBITS;
		break;
	}
	
	if (open_) 
	{

		// Initialize the DCBlength member. 
		PortDCB.DCBlength = sizeof (DCB); 

		// Get the default port setting information.
		GetCommState (hPort_, &PortDCB);

		// Change the DCB structure settings.
		PortDCB.BaudRate = Baudrate;         // Current baud 
		PortDCB.fBinary = TRUE;               // Binary mode; no EOF check 
		PortDCB.fParity = TRUE;               // Enable parity checking 
		PortDCB.fOutxCtsFlow = FALSE;         // No CTS output flow control 
		PortDCB.fOutxDsrFlow = FALSE;         // No DSR output flow control 
		PortDCB.fDtrControl = DTR_CONTROL_ENABLE; 
											  // DTR flow control type 
		PortDCB.fDsrSensitivity = FALSE;      // DSR sensitivity 
		PortDCB.fTXContinueOnXoff = TRUE;     // XOFF continues Tx 
		PortDCB.fOutX = FALSE;                // No XON/XOFF out flow control 
		PortDCB.fInX = FALSE;                 // No XON/XOFF in flow control 
		PortDCB.fErrorChar = FALSE;           // Disable error replacement 
		PortDCB.fNull = FALSE;                // Disable null stripping 
		PortDCB.fRtsControl = RTS_CONTROL_ENABLE; 
											  // RTS flow control 
		PortDCB.fAbortOnError = FALSE;        // Do not abort reads/writes on 
											  // error
		PortDCB.ByteSize = (BYTE) par.DataBits();          // Number of bits/byte, 4-8 
		PortDCB.Parity = Parity;            // 0-4=no,odd,even,mark,space 
		PortDCB.StopBits = StopBits;        // 0,1,2 = 1, 1.5, 2 

		// Configure the port according to the specifications of the DCB 
		// structure.
		if (!SetCommState (hPort_, &PortDCB))
		{
		  // Could not configure the serial port.
		  return false;
		}
		
		return true;
	}
	return false;
}

//=============================================================================

bool SerialWin::ConfigureTimeout( void )
{
	if (open_)
	{
		// Retrieve the timeout parameters for all read and write operations
		// on the port. 
		COMMTIMEOUTS CommTimeouts;
		
		if (hPort_ == NULL) return false;
		
		GetCommTimeouts (hPort_, &CommTimeouts);

		// Change the COMMTIMEOUTS structure settings.
		CommTimeouts.ReadIntervalTimeout = MAXDWORD;  
		CommTimeouts.ReadTotalTimeoutMultiplier = 0;  
		CommTimeouts.ReadTotalTimeoutConstant = 0;    
		CommTimeouts.WriteTotalTimeoutMultiplier = 10;  
		CommTimeouts.WriteTotalTimeoutConstant = 1000;    

		// Set the timeout parameters for all read and write operations
		// on the port. 
		if (!SetCommTimeouts (hPort_, &CommTimeouts))
		{
		  // Could not set the timeout parameters.
		  return false;
		}
		return true;
	}
	return false;
}

//=============================================================================

/* virtual */
bool SerialWin::Close( void )
{
	if (open_)
	{
		CloseHandle(hPort_);
		open_ = false;
	}
	return !open_;
}

//=============================================================================

/* virtual */
bool SerialWin::Send( const CommMsg& msg )
{
	bool success = false;

	WORD Index = 0;
	WORD Sub_Index = 0;
	WORD NodeID = 0;
	DWORD dwData = 0;

	//! Clear all error codes and buffer variable
	dwErrorCode_ = 0;
	dwErrorCodeCommunication_ = 0;
	dwRead_ = 0;

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
			dwErrorCode_ = WriteObject(Index, Sub_Index, NodeID, dwData);
			dwErrorCodeCommunication_ = dwErrorCode_;
		}
		else // read from EPOS device
		{
			dwRead_ = ReadObject(Index, Sub_Index, NodeID, &dwErrorCode_, &dwErrorCodeCommunication_);
		}
	}

	success = ((dwErrorCode_) == 0) || ((dwErrorCodeCommunication_) == 0);

	return success;
}

//=============================================================================

/* virtual */
bool SerialWin::Receive(CommMsg& msg, unsigned short timeout)
{
	char Result[8]={0};

	if (open_)
	{
		Result[0] = (dwRead_ & 0xFF);
		Result[1] = ((dwRead_ >> 8) & 0xFF);
		Result[2] = ((dwRead_ >> 16) & 0xFF);
		Result[3] = ((dwRead_ >> 24) & 0xFF);
		Result[4] = (dwErrorCodeCommunication_ & 0xFF);
		Result[5] = ((dwErrorCodeCommunication_ >> 8) & 0xFF);
		Result[6] = ((dwErrorCodeCommunication_ >> 16) & 0xFF);
		Result[7] = ((dwErrorCodeCommunication_ >> 24) & 0xFF);
		msg.Data(Result, 8);
		msg.Timestamp( boost::posix_time::microsec_clock::local_time() );
		return true;
	}

	return false;
}

//=============================================================================

WORD SerialWin::CalcFieldCRC(const WORD* pDataArray, WORD numberOfWords)
{
   WORD shifter, c;
   WORD carry;
   WORD CRC = 0;

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

DWORD SerialWin::WriteObject(WORD Index, WORD Sub_Index, WORD NodeID, DWORD dwData)
{
	DWORD Result=0;

	if (Result = WriteObject_SendData(Index, Sub_Index, NodeID, dwData))
		return Result;

	if (Result = WriteObject_ReceiveData())
		return Result;

	return Result;
}

//=============================================================================

DWORD SerialWin::WriteObject_SendData(WORD Index, WORD Sub_Index, WORD NodeID, DWORD dwData)
{
	BYTE OpCode = WRITEOBJECT_OPCODE;
	BYTE Len_1 = WRITEOBJECT_LEN_1;
	BYTE readyAck=ACK_FALSE, endAck=ACK_FALSE;
	WORD *WordsToWrite=NULL;
	WORD WordsCRC[6] = {0};
	WORD wCRC=0;

	if (!WriteByteToSerial(OpCode)) return ERROR_WRITING_BYTE;
	readyAck = WaitToReadByteFromSerial();
	if (readyAck == ACK_FALSE) return FAIL_WRITE_RDY_ACK;
	if (!WriteByteToSerial(Len_1)) return ERROR_WRITING_BYTE;
	WordsToWrite = new WORD[4];  // 4 Words to write
	memset(WordsToWrite, 0, 8);

	WordsToWrite[0] = Index;
	WordsToWrite[1] = Sub_Index | (NodeID << 8);
	WordsToWrite[2] = ((WORD) (dwData & 0x0000FFFF));
	WordsToWrite[3] = ((WORD) (dwData >> 16));

	if (!WriteWordsToSerial(WordsToWrite, 4)) return ERROR_WRITING_WORDS;


	WordsCRC[0] = WRITEOBJECT_LEN_1 | (((WORD) WRITEOBJECT_OPCODE) << 8);
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

DWORD SerialWin::WriteObject_ReceiveData(void)
{
	BYTE OpCode=WRITEOBJECT_OPCODE_RESPONSE;
	BYTE readyAck=ACK_OK;
	BYTE endAck=ACK_OK;
	BYTE numberOfWords=1;
	WORD wCRC=0, wCRC_Calc=0;
	WORD *WordsCRC=NULL;
	WORD *WordsReaded=NULL;
	DWORD ResultErrorCode=0;
	int i;

	OpCode = WaitToReadByteFromSerial();
	if (OpCode != WRITEOBJECT_OPCODE_RESPONSE) return WRONG_OPCODE_WRITE;
	if (!WriteByteToSerial(readyAck)) return ERROR_WRITING_BYTE;
	numberOfWords = WaitToReadByteFromSerial();
	WordsReaded = new WORD[numberOfWords+1];
	memset(WordsReaded, 0, (numberOfWords+1)*2);
	WaitToReadWordsFromSerial(WordsReaded, (WORD) (numberOfWords+1) );

	WaitToReadWordsFromSerial(&wCRC, 1);  // CRC size = 1 word

	WordsCRC = new WORD[numberOfWords+3];
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
	
	if (!WriteByteToSerial(endAck)) return ERROR_WRITING_BYTE;

	if (numberOfWords == WRITEOBJECT_LEN_1_RESPONSE) //Len_1 = 1
		ResultErrorCode = WordsReaded[0] | (0xFFFF0000 & (WordsReaded[1] << 16));
	else
		ResultErrorCode = WordsReaded[0];

	delete [] WordsCRC;
	delete [] WordsReaded;

	return ResultErrorCode;
}

//=============================================================================

BOOL SerialWin::WriteByteToSerial(BYTE ByteToWrite)
{
	BOOL WriteResult=0;
	DWORD dwNumBytesWritten=0;

	WriteResult = WriteFile (hPort_,  // Port handle
	   &ByteToWrite,        // Pointer to the data to write 
	   sizeof(ByteToWrite), // Number of bytes to write
	   &dwNumBytesWritten, // Pointer to the number of bytes 
						   // written
	   NULL                // Must be NULL for Windows CE
	);

	return WriteResult;
}

//=============================================================================

BOOL SerialWin::WriteWordsToSerial(const WORD *pWords, WORD numberOfWords)
{
	BOOL WriteResult=0;
	DWORD dwNumBytesWritten=0;

	WriteResult = WriteFile (hPort_,              // Port handle
	   pWords,        // Pointer to the data to write 
	   sizeof(WORD)*numberOfWords, // Number of bytes to write
	   &dwNumBytesWritten, // Pointer to the number of bytes 
						   // written
	   NULL                // Must be NULL for Windows CE
	);
	return WriteResult;
}

//=============================================================================

BYTE SerialWin::ReadByteFromSerial(void)
{
	BYTE ByteReaded=0;
	DWORD dwBytesTransferred=0;

	ReadFile (hPort_,                // Port handle
			  &ByteReaded,          // Pointer to data to read
			  1,                    // Number of bytes to read
			  &dwBytesTransferred,  // Pointer to number of bytes
									// read
			  NULL                  // Must be NULL for Windows CE
	);
	return ByteReaded;
}

//=============================================================================

BYTE SerialWin::WaitToReadByteFromSerial(void)
{
	DWORD dwCommModemStatus;
	BYTE ByteReaded;
	DWORD dwBytesTransferred=0;
	struct timeb tp;
	time_t t1, t2;

	// Specify a set of events to be monitored for the port.
	SetCommMask (hPort_, EV_RXCHAR | EV_CTS | EV_DSR | EV_RLSD | EV_RING);

	ftime(&tp);
	t1 = tp.millitm + (tp.time & 0xFFFFF) * 1000;

	while (hPort_ != INVALID_HANDLE_VALUE) 
	{
	  // Wait for an event to occur for the port.
	  WaitCommEvent (hPort_, &dwCommModemStatus, 0);

	  // Re-specify the set of events to be monitored for the port.
	  SetCommMask (hPort_, EV_RXCHAR | EV_CTS | EV_DSR | EV_RING);

	  if (dwCommModemStatus & EV_RXCHAR) 
	  {
		 
		// Loop for waiting for the data.
		do 
		{
		  // Read the data from the serial port.
		  ByteReaded = ReadByteFromSerial();

		  if (dwBytesTransferred == 1)
		  {
			dwBytesTransferred = 0;
		  }

		  ftime(&tp);
	      t2 = tp.millitm + (tp.time & 0xFFFFF) * 1000;
		  if ((abs((long)(t1-t2))) > LOOPTIMEOUT_MS) break;

		} while (dwBytesTransferred == 1);
		break;
	  }
	}
	
	return ByteReaded;
}

//=============================================================================

VOID SerialWin::WaitToReadWordsFromSerial(WORD *pReadedWords, WORD numberOfWords)
{
	DWORD dwCommModemStatus;
	DWORD dwBytesTransferred=0;
	struct timeb tp;
	time_t t1, t2;

	// Specify a set of events to be monitored for the port.
	SetCommMask (hPort_, EV_RXCHAR | EV_CTS | EV_DSR | EV_RLSD | EV_RING);

	ftime(&tp);
	t1 = tp.millitm + (tp.time & 0xFFFFF) * 1000;

	while (hPort_ != INVALID_HANDLE_VALUE) 
	{
	  // Wait for an event to occur for the port.
	  WaitCommEvent (hPort_, &dwCommModemStatus, 0);

	  // Re-specify the set of events to be monitored for the port.
	  SetCommMask (hPort_, EV_RXCHAR | EV_CTS | EV_DSR | EV_RING);

	  if (dwCommModemStatus & EV_RXCHAR) 
	  {
		// Loop for waiting for the data.
		Sleep(5); // Wait for 5 [ms]
		do 
		{
		  // Read the data from the serial port.
			ReadFile (hPort_,                // Port handle
					  pReadedWords,          // Pointer to data to read
					  numberOfWords*2,                    // Number of bytes to read
					  &dwBytesTransferred,  // Pointer to number of bytes
											// read
					  NULL                  // Must be NULL for Windows CE
			);

		  ftime(&tp);
	      t2 = tp.millitm + (tp.time & 0xFFFFF) * 1000;
		  if ((abs((long)(t1-t2))) > LOOPTIMEOUT_MS) break;

		} while (dwBytesTransferred != numberOfWords*2);
		break;
	  }
	}
	return;
}


//=============================================================================

DWORD SerialWin::ReadObject_SendData(WORD Index, WORD Sub_Index, WORD NodeID)
{
	BYTE OpCode = READOBJECT_OPCODE;
	BYTE Len_1 = READOBJECT_LEN_1;
	BYTE readyAck, endAck;
	WORD *WordsToWrite;
	WORD WordsCRC[4] = {0};
	WORD wCRC;

	if (!WriteByteToSerial(OpCode)) return ERROR_WRITING_BYTE;
	readyAck = WaitToReadByteFromSerial();
	if (readyAck == ACK_FALSE) return FAIL_READ_RDY_ACK;
	if (!WriteByteToSerial(Len_1)) return ERROR_WRITING_BYTE;
	WordsToWrite = new WORD[2];

	WordsToWrite[0] = Index;
	WordsToWrite[1] = Sub_Index | (NodeID << 8);

	if (!WriteWordsToSerial(WordsToWrite, 2)) return ERROR_WRITING_WORDS;

	WordsCRC[0] = READOBJECT_LEN_1 | (((WORD) READOBJECT_OPCODE) << 8);
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

DWORD SerialWin::ReadObject_ReceiveData(WORD **pData, BYTE *pnumberOfWords)
{
	BYTE OpCode=READOBJECT_OPCODE_RESPONSE;
	BYTE readyAck=ACK_OK;
	BYTE endAck;
	WORD wCRC, wCRC_Calc;
	WORD *WordsCRC;
	WORD *WordsReaded=NULL;
	int i;

	OpCode = WaitToReadByteFromSerial();
	if (OpCode != READOBJECT_OPCODE_RESPONSE) return WRONG_OPCODE_READ;

	if (!WriteByteToSerial(readyAck)) return ERROR_WRITING_BYTE;
	*pnumberOfWords = WaitToReadByteFromSerial();  // Len_1
	WordsReaded = new WORD[*pnumberOfWords+1];
	memset(WordsReaded, 0, (*pnumberOfWords+1)*2);
	WaitToReadWordsFromSerial(WordsReaded, (WORD) (*pnumberOfWords+1) );
	*pData = WordsReaded;

	WaitToReadWordsFromSerial(&wCRC, 1);  // CRC size = 1 word

	WordsCRC = new WORD[*pnumberOfWords+3];
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
	
	if (!WriteByteToSerial(endAck)) return ERROR_WRITING_BYTE;

	delete [] WordsCRC;

	return 0;
}

//=============================================================================

DWORD SerialWin::ReadObject(WORD Index, WORD Sub_Index, WORD NodeID, DWORD *pdwErrorCode, DWORD *pdwErrorCodeCommunication)

{
	WORD *WordsReaded=NULL;
	BYTE numberOfWords;
	DWORD dwFlag;
	DWORD dwReaded=0;

	*pdwErrorCode = 0x0;
	*pdwErrorCodeCommunication = 0x0;

	*pdwErrorCode = ReadObject_SendData(Index, Sub_Index, NodeID);
	if (*pdwErrorCode != 0) return 0;
	*pdwErrorCode = ReadObject_ReceiveData((WORD**) &WordsReaded, &numberOfWords);

	dwFlag = (DWORD) WordsReaded[1];
	*pdwErrorCodeCommunication = (DWORD) WordsReaded[0] | (dwFlag << 16);

	dwFlag = (DWORD) WordsReaded[3];
	dwReaded = (DWORD) WordsReaded[2] | (dwFlag << 16);

	delete [] WordsReaded;

	return dwReaded;
}

//=============================================================================

}

#endif //VATROSLAV_SERIAL_WIN