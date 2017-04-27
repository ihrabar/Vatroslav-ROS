/*! \file	SerialWin.hpp
	\brief	Serial RS232 communication implementation on Windows based platform.
	
	USE THIS FILE WITH WINDOWS FUNCTIONS FOR SERIAL COMMUNICATION ON YOUR RISK. ON
	SOME COMPUTERS THEY MAY NOT WORK PROPERLY. 

 */

#ifdef VATROSLAV_SERIAL_WIN

#ifndef VATROSLAV_SERIAL_WIN_HPP
#define VATROSLAV_SERIAL_WIN_HPP

#include <cassert>
#include <iostream>
#include <sys/timeb.h>

#include "Windows.h"
#include "Winbase.h"

#include "SerialTypeDecs.hpp"

#define LOKI_CLASS_LEVEL_THREADING	// Loki's threading policy
#include <loki/Singleton.h>

#include "CommImpl.hpp"

namespace Vatroslav
{

//! Serial communication implementation for Windows OS
/*!

*/
class SerialWin : public CommImpl 
{
 public:

	//! Open function...
	/*!
		This function opens serial communication port defined
		with string name and sets its handle.
		
		@param par	Communication parameters. Currently just a placeholder.
	 */
	virtual bool Open( const CommPar& par );

	//! Close function...
	/*!

	 */
	virtual bool Close( void );

	//! Send a message...
	/*!

	 */
	virtual bool Send( const CommMsg& msg );

	//! Receive a message...
	/*!

	 */
	virtual bool Receive( CommMsg& msg, unsigned short timeout );

	//! Get communication parameters.
	/*!

	 */
	virtual const CommPar& Params( void )
	{
		return par_;
	}

 private:

	//! Constructor
	/*!
		All the construction facilities are made private, because the class is
		ment to be used only as a Singleton !
	 */
	SerialWin( );

	//! Copy constructor
	SerialWin( const SerialWin& original );

	//! Destructor
	~SerialWin( );

	//! Assignment operator
	SerialWin& operator=( const SerialWin& rhs );

	//! Address-of operator
	SerialWin* operator&( void );

	//! Configure serial port with DCB structure
	/*!
		!!!The most critical phase in serial communications programming
		is configuring the port settings with the DCB structure!!! :-(
	*/
	bool Configure( const CommPar& par );

	//! Configure serial timeouts for opened port
	/*!
		If this structure is not configured, the port uses default timeouts
		supplied by the driver, or timeouts from a previous communication application. 

		By assuming specific timeout settings when the settings are actually different, an
		application can have read/write operations that never complete or complete too often.
	*/
	bool ConfigureTimeout( void );

	//! Calculate message CRC
	/*!
		Calculate message CRC (Cyclic Redundancy Check) in process of sending
		messages or only check if received message is valid. Following parameters are 
		used:

		@param pDataArray - pointer to data field (message in general)
		@param numberOfWords - number of words (length of the message)
	*/
	WORD CalcFieldCRC(const WORD* pDataArray, WORD numberOfWords);

	//! Converts standard string to wstring data type
	/*!
		Converts standard string to wstring type and cares about
		unicode characters. This function is important for defining
		port name.

		@param s - standard string
	*/
	std::wstring SerialWin::S2WS(const std::string& s);

	//! Writes data to controller using serial communication
	/*!
		Function is used to write data to motor controller using serial
		communication. This process is structured of sending and receiving
		data to controller. The result is error code.

		@param Index - message identifier
		@param Sub_Index - message identifier in group of messages with similar type
		@param NodeID - controller No.
		@param dwData - data to write to controller
	*/

	DWORD WriteObject(WORD Index, WORD Sub_Index, WORD NodeID, DWORD dwData);

	//! Sends data to controller in writing process
	/*!
		This function sends data to controller in writing process. After this, receiving follows.
		The result is error code. 

		@param Index - message identifier
		@param Sub_Index - message identifier in group of messages with similar type
		@param NodeID - controller No.
		@param dwData - data to write to controller
	*/
	DWORD WriteObject_SendData(WORD Index, WORD Sub_Index, WORD NodeID, DWORD dwData);

	//! Receives data from controller in writing process
	/*!
		This function receives data from controller in writing process. This is useless function
		for user, but important in defined serial communication structure. The result is error code.
	*/
	DWORD WriteObject_ReceiveData(void);

	//! Writes one byte to controller using serial communication
	/*!
		Writes one byte using serial communication to controller. The result of this operation
		is success. 

		@param ByteToWrite - byte for writing
	*/
	BOOL WriteByteToSerial(BYTE ByteToWrite);

	//! Writes words to controller using serial communication
	/*!
		Function is used to write one or more words (16-bit data) to motor controller using
		serial communication. The result is success of this operation.

		@param pWords - pointer to words for writing
		@param numberOfWords - number of words for writing
	*/
	BOOL WriteWordsToSerial(const WORD *pWords, WORD numberOfWords);

	//! Reads one byte from controller using serial communication
	/*!
		Reads one byte using serial communication from controller. The result is
		success of this operation.
	*/
	BYTE ReadByteFromSerial(void);

	//! Waits for and reads one byte from controller using serial communication
	/*!
		Function waits for and reads one byte using serial communication. It calls
		"ReadByteFromSerial" function and returns readed byte. It waits for an
		event to occur for the port.
	*/
	BYTE WaitToReadByteFromSerial(void);

	//! Waits for and reads words from controller using serial communication
	/*!
		Function waits for and reads one or more words using serial communication.
		It waits for an event to occur for the port.

		@param pReadedWords - pointer to words for writing
		@param numberOfWords - number of words for writing
	*/
	VOID WaitToReadWordsFromSerial(WORD *pReadedWords, WORD numberOfWords);

	//! Sends data to controller in reading process
	/*!
		This function sends data to controller in reading process. After this, receiving follows.
		The result is error code.

		@param Index - message identifier
		@param Sub_Index - message identifier in group of messages with similar type
		@param NodeID - controller No.
	*/
	DWORD ReadObject_SendData(WORD Index, WORD Sub_Index, WORD NodeID);

	//! Receives data from controller in reading process
	/*!
		This function receives data from controller in reading process. The result is error code.

		@param pData - double pointer to readed data
		@param pnumberOfWords - pointer to variable with number of words data 
	*/
	DWORD ReadObject_ReceiveData(WORD **pData, BYTE *pnumberOfWords);

	//! Reads data from controller using serial communication
	/*!
		Function is used to read data from motor controller using serial
		communication. This process is structured of sending and receiving
		data to controller. The result is error code.

		@param Index - message identifier
		@param Sub_Index - message identifier in group of messages with similar type
		@param NodeID - controller No.
		@param pdwErrorCode - pointer to error code caused during this communication process
		@param pdwErrorCodeCommunication - pointer to error code caused in, and reported from controller
	*/
	DWORD ReadObject(WORD Index, WORD Sub_Index, WORD NodeID, DWORD *pdwErrorCode, DWORD *pdwErrorCodeCommunication);


	//! Whether the channel is open or not.
	bool open_;
	//! Communication parameters.
	CommPar par_;
	//! Handle of the object/port
	HANDLE hPort_;
	//! Error code variable during communication process
	DWORD dwErrorCode_;
	//! Error code returned from EPOS servo drive
	DWORD dwErrorCodeCommunication_;
	//! Variable which stores readed value
	DWORD dwRead_;

	//! Define some constants for serial communication
	static const int LOOPTIMEOUT_MS					= 1000;

	//! Define some OPCODES, used as a header part of the message frame structure
	static const int READOBJECT_OPCODE				= 0x10;
	static const int READOBJECT_OPCODE_RESPONSE		= 0x00;
	static const int WRITEOBJECT_OPCODE				= 0x11;
	static const int WRITEOBJECT_OPCODE_RESPONSE	= 0x00;

	//! Length of messages (len-1) during writing to or reading from EPOS in 
	//! acknowledge or response contest
	static const int READOBJECT_LEN_1				= 0x01;
	static const int READOBJECT_LEN_1_RESPONSE		= 0x03;
	static const int WRITEOBJECT_LEN_1				= 0x03;
	static const int WRITEOBJECT_LEN_1_RESPONSE		= 0x01;

	//! Acknowledge message codes for determining success of serial communication
	static const int ACK_OK							= 0x4F;
	static const int ACK_FALSE						= 0x46;

	//! Error codes during communication process
	static const int ERROR_WRITING_BYTE				= 0x01;
	static const int ERROR_WRITING_WORDS			= 0x02;
	static const int FAIL_READ_RDY_ACK				= 0x03;
	static const int FAIL_READ_END_ACK				= 0x04;
	static const int FAIL_WRITE_RDY_ACK				= 0x05;
	static const int FAIL_WRITE_END_ACK				= 0x06;
	static const int WRONG_OPCODE_READ				= 0x07;
	static const int WRONG_OPCODE_WRITE				= 0x08;

	// We need to grant friend access, in order for 
	// Loki::SingletonHolder to work.
	friend struct Loki::CreateUsingNew<SerialWin>;
};

//! Define the SerialWin singleton for public use.
typedef Loki::SingletonHolder<SerialWin, Loki::CreateUsingNew,
							  Loki::PhoenixSingleton> SerialWinSingleton;

}

#endif //VATROSLAV_SERIAL_WIN_HPP

#endif //VATROSLAV_SERIAL_WIN