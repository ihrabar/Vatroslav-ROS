/*!	\file	CommMsg.hpp
	\brief	Communication message class.

 */

#ifndef VATROSLAV_COMM_MSG_HPP
#define VATROSLAV_COMM_MSG_HPP

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <string.h>
namespace Vatroslav
{

//! Communication message.
/*! 
	This is the basic message that can be sent and received over the
	communication channel. Each message has the following three properties:

	Id	 -	the message Id 
		 -	on the can bus, an integer betwen 0 and 65536 (0xFFFF)
		 -	in serial communication ???
	data -	message data
	size -	message size, in bytes
	timestamp -	message arrival time

 */
class CommMsg
{
 public:
	
	//! Constructor.
	/*!
		Creates a message from the provided Id and data.
	 */
	CommMsg( unsigned short id = 0xFFFF, 
			 const char* data = NULL, 
			 size_t size = 0,
			 boost::posix_time::ptime timestamp = boost::posix_time::not_a_date_time)
			 : id_(id), data_(NULL), size_(size), timestamp_(timestamp)
	{
		if ( size_ != 0 )
		{
			data_ = new char[size_];
			memcpy( data_, data, size_ );
		}
	}

	//! Copy constructor.
	/*!
		
	 */
	CommMsg( const CommMsg& original )
		: id_(original.id_), 
		  data_(NULL), size_(original.size_), 
		  timestamp_(original.timestamp_)
	{
		if ( size_ != 0 )
		{
			data_ = new char[size_];
			memcpy( data_, original.data_, size_ );
		}
	}

	//! Destructor
	/*!

	 */
	~CommMsg()
	{
		if ( data_ != NULL)
		{
			delete [] data_;
		}
	}

	//! Assignment operator
	/*!

	 */
	CommMsg& operator=( const CommMsg& rhs )
	{
		if ( this != &rhs ) // checking for self-assignment
		{
			if ( data_ != NULL )
			{
				delete [] data_;
			}
			id_ = rhs.id_;
			timestamp_ = rhs.timestamp_;
			size_ = rhs.size_;
			if ( size_ != 0 )
			{
				data_ = new char[size_];
				memcpy( data_, rhs.data_, size_ );
			}
			else
			{
				data_ = NULL;
			}
		}
		return *this;
	}

	//! Get message Id.
	/*!

	 */
	unsigned short Id( void ) const { return id_; }
	
	//! Set message Id.
	void Id( unsigned short id ) { id_ = id; }

	//! Get pointer to data buffer
	/*!
		 
	 */
	const char* Data( void ) const { return data_; }

	//! Get data size.
	size_t Size( void ) const { return size_; }

	//! Reset message data
	void Data( const char* data, size_t size )
	{
		if ( data != data_ ) 
		{
			if ( data_ != NULL )
			{
				delete [] data_;
				data_ = NULL;
			}
			size_ = size;
			if ( size_ != 0 )
			{
				data_ = new char[size_];
				memcpy( data_, data, size_ );
			}
		}
	}

	//! Get the timestamp
	const boost::posix_time::ptime Timestamp( void ) const 
	{ 
		return timestamp_; 
	}

	//! Set the timestamp
	void Timestamp( const boost::posix_time::ptime& timestamp )
	{
		timestamp_ = timestamp;
	}

 private:
	
	// Message Id on the CAN bus, betwen 0 and 65536 (0xFFFF)
	unsigned short id_;
	// The data buffer.
	char* data_;
	// Data size (in bytes i.e. in sizeof(char) units)
	size_t size_;
	// Data timestamp
	boost::posix_time::ptime timestamp_;
};

}

#endif
