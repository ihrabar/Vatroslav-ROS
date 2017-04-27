/*! \file	SerialBoost.hpp
	\brief	Serial RS232 communication implementation on all platforms.

 */

#ifndef VATROSLAV_SERIAL_BOOST_HPP
#define VATROSLAV_SERIAL_BOOST_HPP


#include <cassert>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/local_time/local_time.hpp"
#include "boost/asio.hpp"
#include "boost/shared_ptr.hpp"

#define LOKI_CLASS_LEVEL_THREADING	// Loki's threading policy
#include <loki/Singleton.h>

#include "CommImpl.hpp"

namespace Vatroslav
{

//! Serial communication implementation for all OS platforms.
/*!

*/
class SerialBoost : public CommImpl 
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
	//! Writes one byte to controller using serial communication
	/*!
		Writes one byte using serial communication to controller. The result of this operation
		is success. 

		@param ByteToWrite - byte for writing
	*/
	bool WriteToSerial(const unsigned char* Buf, std::streamsize N);

	//! Writes words to controller using serial communication
	/*!
		Function is used to write one or more words (16-bit data) to motor controller using
		serial communication. The result is success of this operation.

		@param pWords - pointer to words for writing
		@param numberOfWords - number of words for writing
	*/
	bool WriteWordsToSerial(const unsigned short *pWords, unsigned short numberOfWords);

	//! Reads one byte from controller using serial communication
	/*!
		Reads one byte using serial communication from controller. The result is
		success of this operation.
	*/
	unsigned char ReadByteFromSerial(void);

	//! Waits for and reads one byte from controller using serial communication
	/*!
		Function waits for and reads one byte using serial communication. It calls
		"ReadByteFromSerial" function and returns readed byte. It waits for an
		event to occur for the port.
	*/
	unsigned char WaitToReadByteFromSerial(void);

	//! Waits for and reads words from controller using serial communication
	/*!
		Function waits for and reads one or more words using serial communication.
		It waits for an event to occur for the port.

		@param pReadedWords - pointer to words for writing
		@param numberOfWords - number of words for writing
	*/
	void WaitToReadWordsFromSerial(unsigned short *pReadedWords, unsigned short numberOfWords);
	//! Constructor
	/*!
		All the construction facilities are made private, because the class is
		ment to be used only as a Singleton !
	 */
	SerialBoost( );

	//! Copy constructor
	SerialBoost( const SerialBoost& original );

	//! Destructor
	~SerialBoost( );

 private:


	//! Assignment operator
	SerialBoost& operator=( const SerialBoost& rhs );

	//! Address-of operator
	SerialBoost* operator&( void );

	//! Calculate message CRC
	/*!
		Calculate message CRC (Cyclic Redundancy Check) in process of sending
		messages or only check if received message is valid. Following parameters are 
		used:

		@param pDataArray - pointer to data field (message in general)
		@param numberOfWords - number of words (length of the message)
	*/
	unsigned short CalcFieldCRC(const unsigned short* pDataArray, unsigned short numberOfWords);

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

	unsigned long WriteObject(unsigned short Index, unsigned short Sub_Index, unsigned short NodeID, unsigned long dwData);

	//! Sends data to controller in writing process
	/*!
		This function sends data to controller in writing process. After this, receiving follows.
		The result is error code. 

		@param Index - message identifier
		@param Sub_Index - message identifier in group of messages with similar type
		@param NodeID - controller No.
		@param dwData - data to write to controller
	*/
	unsigned long WriteObject_SendData(unsigned short Index, unsigned short Sub_Index, unsigned short NodeID, unsigned long dwData);

	//! Receives data from controller in writing process
	/*!
		This function receives data from controller in writing process. This is useless function
		for user, but important in defined serial communication structure. The result is error code.
	*/
	unsigned long WriteObject_ReceiveData(void);


	//! Sends data to controller in reading process
	/*!
		This function sends data to controller in reading process. After this, receiving follows.
		The result is error code.

		@param Index - message identifier
		@param Sub_Index - message identifier in group of messages with similar type
		@param NodeID - controller No.
	*/
	unsigned long ReadObject_SendData(unsigned short Index, unsigned short Sub_Index, unsigned short NodeID);

	//! Receives data from controller in reading process
	/*!
		This function receives data from controller in reading process. The result is error code.

		@param pData - double pointer to readed data
		@param pnumberOfWords - pointer to variable with number of words data 
	*/
	unsigned long ReadObject_ReceiveData(unsigned short **pData, unsigned char *pnumberOfWords);

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
	unsigned long ReadObject(unsigned short Index, unsigned short Sub_Index, unsigned short NodeID, unsigned long *pdwErrorCode, unsigned long *pdwErrorCodeCommunication);


	//! Whether the channel is open or not.
	bool open_;
	//! Communication parameters.
	CommPar par_;

	//! Boost variables needed for serial communication
	std::auto_ptr<boost::asio::io_service> pio_;
	std::auto_ptr<boost::asio::serial_port> pserial_;

	unsigned long errorCode_;
	//! Error code returned from EPOS servo drive
	unsigned long errorCodeCommunication_;
	//! Variable which stores readed value
	unsigned long read_;

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
	friend struct Loki::CreateUsingNew<SerialBoost>;
};

//! Define the SerialBoost singleton for public use.
typedef Loki::SingletonHolder<SerialBoost, Loki::CreateUsingNew,
							  Loki::PhoenixSingleton> SerialBoostSingleton;

}

#endif //VATROSLAV_SERIAL_BOOST_HPP
