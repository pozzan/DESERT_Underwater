//
// Copyright (c) 2012 Regents of the SIGNET lab, University of Padova.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Padova (SIGNET lab) nor the 
//    names of its contributors may be used to endorse or promote products 
//    derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/**
 * @file mfsk_whoi_mm/mserial.cc
 * @author Riccardo Masiero, Matteo Petrani, Ivano Calabrese
 * \version 2.0.0
 * \brief  Implementation of the Mserial class. 
 */

#include "mserial.h"
#include <uwmdriver.h>

Mserial::Mserial(UWMdriver* pmDriver_, std::string pathToDevice_) : UWMconnector(pmDriver_, pathToDevice_)
{
	// Variable initialization
	fd = 0;
	rc = 0;

	if (debug_)
	{
		cout << this << ": in constructor of Mserial which points to driver: " << pmDriver << "\n";
	}
}

Mserial::~Mserial()
{
}

int Mserial::openConnection()
{
	if (debug_)
	{
		cout << this << ": in openConnection() method of Mserial. Note: ID = " << pmDriver -> getID() << "\n";
	}
	// Open modem device for reading and writing and not as controlling tty because we don't want to get killed if linenoise sends CTRL-C.
	// Opening.
	fd = open(pathToDevice.c_str(), O_RDWR | O_NOCTTY);
	// Check for opening errors.
	if (fd < 0)
	{
		perror(pathToDevice.c_str());
		exit(1);
	}
	// Save current serial port settings.
	tcgetattr(fd, &oldtio);
	// Set the new port settings.
	new_port_settings(&newtio);
	// Clean the modem line.
	tcflush(fd, TCIFLUSH);
	// Activate the new settings for the port.
	tcsetattr(fd, TCSANOW, &newtio);

	// Create the parallel process to read from the serial.
	rc = pthread_create(&thread_id, NULL, read_process_mserial, (void *) this);
	//rc = pthread_create(&thread_id, NULL, read_process_mserial, NULL);
	// Check the correct starting of the reading process
	if (rc)
	{
		printf("ERROR; return code from pthread_create() is %d\n", rc);
		exit(-1);
	}

	return _MODEM_OK;
}

void Mserial::closeConnection()
{

	// Restore the old port settings before closing. 
	tcsetattr(fd, TCSANOW, &oldtio);
}

int Mserial::writeToModem(std::string str)
{

	if (debug_)
	{
		cout << this << ": in Mserial::writeToModem(string). Note: ID = " << pmDriver -> getID() << ", TX msg = " << str << "\n";
	}

	// Copy the input string in Mserial::msg_tx
	strcpy(msg_tx, str.c_str());
	// Length of the message to be transmitted
	int msg_ssz = strlen(msg_tx);

	// Append to the message that we want to send the windows-complaint string terminator (WHOI Micro-Modem requirements) 
	msg_tx[msg_ssz++] = '\r';
	msg_tx[msg_ssz++] = '\n';
	msg_tx[msg_ssz] = '\0';

	// Send message host to modem via serial
	int w_bytes = write(fd, msg_tx, msg_ssz);

	// Return number of written bytes
	return w_bytes;
}

void Mserial::new_port_settings(struct termios *tio_p)
{

	// Clear struct for new port settings. 
	memset(tio_p, 0, sizeof (*tio_p));

	/* BAUDRATE : Set bps rate. You could also use cfsetispeed and cfsetospeed.
	 * CRTSCTS : output hardware flow control (only used if the cable has all necessary lines. See sect. 7 of Serial-HOWTO) not used for WHOI micromodems since they want all the flow controls disabled (comment by RM)
	 * CS8 : 8n1 (8bit,no parity,1 stopbit)
	 * CLOCAL  : local connection, no modem contol
	 * CREAD   : enable receiving characters
	 */
	(*tio_p).c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;

	/* IGNPAR  : ignore bytes with parity errors
	 * ICRNL   : map CR to NL (otherwise a CR input on the other computer will not terminate input) - NOTE (by RM) if enable caused the messages from WHOI to be received in two sentences, the actual message and a blank one made only of a CR (or NL) caracter ...  
	 */
	(*tio_p).c_iflag = IGNPAR;

	// Raw output.
	(*tio_p).c_oflag = 0;

	/* ICANON : enable canonical input disable all echo functionality, and don't send signals to calling program
	 */
	(*tio_p).c_lflag = ICANON;

	// Initialize all control characters default values can be found in /usr/include/termios.h, and are given in the comments, but we don't need them here
	(*tio_p).c_cc[VINTR] = 0; // Ctrl-c
	(*tio_p).c_cc[VQUIT] = 0; // Ctrl-'\'
	(*tio_p).c_cc[VERASE] = 0; // del
	(*tio_p).c_cc[VKILL] = 0; // @
	(*tio_p).c_cc[VEOF] = 4; // Ctrl-d
	(*tio_p).c_cc[VTIME] = 0; // inter-character timer unused
	(*tio_p).c_cc[VMIN] = 1; // blocking read until 1 character arrives
	(*tio_p).c_cc[VSWTC] = 0; // '\0'
	(*tio_p).c_cc[VSTART] = 0; // Ctrl-q
	(*tio_p).c_cc[VSTOP] = 0; // Ctrl-s 
	(*tio_p).c_cc[VSUSP] = 0; // Ctrl-z 
	(*tio_p).c_cc[VEOL] = 0; // '\0' 
	(*tio_p).c_cc[VREPRINT] = 0; // Ctrl-r 
	(*tio_p).c_cc[VDISCARD] = 0; // Ctrl-u 
	(*tio_p).c_cc[VWERASE] = 0; // Ctrl-w 
	(*tio_p).c_cc[VLNEXT] = 0; // Ctrl-v 
	(*tio_p).c_cc[VEOL2] = 0; // '\0' 
}

void *read_process_mserial(void *pMserial_me_)
{
	// Array to store the received message
	char msg_rx[_MAX_MSG_LENGTH];
	// Structure to queue the received message
	msgModem tmp_;
	// Output stream to write to pMserial_me->getReadingBuff()
	std::ofstream out;

	// Cast to the Mserial pointer passed as argument
	Mserial* pMserial_me = (Mserial*) pMserial_me_;

	if (pMserial_me->getDebug())
	{
		cout << "         : in the  read_process() method (parallel thread). Note: ReadingBuff = " << pMserial_me->getReadingBuff() << "\n";
	}


	while (1)
	{
		// Read from the serial
		tmp_.msg_length = read(pMserial_me->getSerial(), msg_rx, _MAX_MSG_LENGTH);
		// Set end of string
		msg_rx[tmp_.msg_length] = '\0';

		// Check the queue length
		if (pMserial_me->queueMsg.size() > _MAX_QUEUE_LENGTH)
		{
			cout << "WARNING in mserial.cc::*read_process_mserial(void *pMserial_me_): the buffering queue is full." << endl;
			pMserial_me->queueMsg.pop();
		}

		tmp_.msg_rx.assign(msg_rx, tmp_.msg_length);
		pMserial_me->queueMsg.push(tmp_);

		if (pMserial_me->getDebug())
		{
			cout << "[WRITE]: <-- " << tmp_.msg_rx << endl;
			cout << "[WRITE]: in the queue there are now: " << pMserial_me->queueMsg.size() << " entries " << endl;
		}

		// Write received message into the disk-file readingBuff.
//		out.open(((pMserial_me->getReadingBuff()).c_str()), ios::app);
//		out << tmp_.msg_rx;
//		out.flush();
//		out.close();

		if (pMserial_me->getDebug())
		{
			cout << "         : in the  read_process() method (parallel thread). Note:  ReadingBuff = " << pMserial_me->getReadingBuff() << ", RX message = " << tmp_.msg_rx << "\n";
		}

	}
	// Exit the parallel reading thread
	pthread_exit(NULL);

}
