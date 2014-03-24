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
 * @file mdriverFSK_WHOI_MM.cc
 * @author Riccardo Masiero, Matteo Petrani
 * \version 1.0.0
 * \brief Implementation of the MdriverFSK_WHOI_MM class. 
 */

#include "mdriverFSK_WHOI_MM.h"
#include <uwmphy_modem.h>

void MdriverFSK_WHOI_MM::emptyModemQueue()
{
    cout << "emptyModemQueue do nothing! " << endl;
}

MdriverFSK_WHOI_MM::MdriverFSK_WHOI_MM(UWMPhy_modem* pmModem_) : UWMdriver(pmModem_), mInterpreter(this), mConnector(this, pmModem_->getPathToDevice())
{
    // Member initialization
    m_status_tx = _IDLE;
    ongoing_tx = false;
    m_status_rx = _IDLE;

    if (!pmModem_->getBin()) {
        umessage = _UMP;
    } else {
        umessage = _UBIN;
    }
    
    // Linkage of the base class pointers
    setConnections(&mInterpreter, &mConnector);

    if (debug_) {
        cout << this << ": in constructor of MdriverFSK_WHOI_MM. The driver points to the modem: " << pmModem << "\n";
    }
}

MdriverFSK_WHOI_MM::~MdriverFSK_WHOI_MM()
{
}


void MdriverFSK_WHOI_MM::start()
{

    if (debug_) {
        cout << this << ": in start() method of MdriverFSK_WHOI_MM. Note: ID = " << ID << "\n";
    }
    // Create the reading buffer of the serial
    //mConnector.create_readingBuff();
    // Open the serial connession
    mConnector.openConnection();
    // Set the modem ID
    status = _CFG;
    m_status_tx = _SETID;
    modemTxManager();
}

void MdriverFSK_WHOI_MM::stop()
{
    // Close the serial connection
    mConnector.closeConnection();
}

void MdriverFSK_WHOI_MM::modemTx()
{
    //Set the right modem status
    status = _TX;
    //Set the right FSK transmission status
    if (umessage == _UBIN) {
        m_status_tx = _CINIT;
    } else if (umessage == _UMP) {
        m_status_tx = _MP;
    }

    if (debug_) {
        cout << this << ": in modemTx() method of MdriverFSK_WHOI_MM. Note: ID = " << ID << ", status = " << status << ", m_status_tx = " << m_status_tx << "\n";
    }
    // Manage the packet transmission
    modemTxManager();
}

int MdriverFSK_WHOI_MM::updateStatus()
{
    if (debug_) {
        cout << this << ": at the begin of the updateStatus() method of MdriverFSK_WHOI_MM. Note: ID = " << ID << ", status = " << status << ", m_status_tx = " << m_status_tx << ", m_status_rx = " << m_status_rx << "\n";
    }
    // variable to store the possible received message
    std::string rx_msg;
    // Variable to continue to read from the modem
    bool cread = true;
    // Variable to assure the exit from the while loop (it can be removed if everything works as expected)
    //int max_readings = 5;

    while (cread) {
        // Read the possible received messages
        rx_msg = mConnector.readFromModem();

        if (rx_msg != "") {
//            if (getShow() == 2) {
//                cout << this << ": IN DRIVER, RX MESSAGE: " << rx_msg << std::endl;
//            }

            // parsing for a recognized string
            // sequence of messages in transmission:
            if (rx_msg.find("$CAMUC") != string::npos) {
                queue_tx.push(rx_msg);
            } else if (rx_msg.find("$CATXP") != string::npos) {
                queue_tx.push(rx_msg);
            } else if (rx_msg.find("$CATXF") != string::npos) {
                queue_tx.push(rx_msg);
            } else if (rx_msg.find("$CAXST") != string::npos) {
                queue_tx.push(rx_msg);
            } else if (rx_msg.find("$CACFG") != string::npos) {
                queue_tx.push(rx_msg);
            } else if (rx_msg.find("$CACYC") != string::npos) {
                if (ongoing_tx){
                    queue_tx.push(rx_msg);
                } else {
                    queue_rx.push(rx_msg);
                }
            } else if (rx_msg.find("$CADRQ") != string::npos) {
                queue_tx.push(rx_msg);
            } else if (rx_msg.find("$CATXD") != string::npos) {
                queue_tx.push(rx_msg);
                
            }// sequence of messages in reception
            else if (rx_msg.find("$CADQF") != string::npos) {
                queue_rx.push(rx_msg);
            } else if (rx_msg.find("$CAMUA") != string::npos) {
                queue_rx.push(rx_msg);
            } else if (rx_msg.find("$CACST") != string::npos) {
                queue_rx.push(rx_msg);
            } else if (rx_msg.find("$CARXP") != string::npos) {
                queue_rx.push(rx_msg);
            } else if (rx_msg.find("$CARXD") != string::npos) {
                queue_rx.push(rx_msg);
                
            } else if (rx_msg.find("$CARXA") != string::npos) {
                queue_rx.push(rx_msg);
            }
            
            else if (rx_msg.find("$CAERR") != string::npos) {
                //queue_rx.push(rx_msg);
                printf("PACKET in\033[0;31m WARNING: unknown packet in the queue: %s\033[0m.", rx_msg.c_str()); 
                cout << "CAERR" << endl;
            } // others
            else if (rx_msg.find("$CAREV") != string::npos) {
                printf("PACKET in: %s\n", rx_msg.c_str());
            } else {
                printf("PACKET in\033[0;31m WARNING: unknown packet in the queue: %s\033[0m.", rx_msg.c_str());
                //continue;
            }
        } else {
            // no new message
            cread = false;
        }
    }

    cread = true;

    if (status == _TX || status == _CFG) {
        // read from queue_tx
        while (cread) {
            if (!queue_tx.empty()) {
                rx_msg = queue_tx.front();
                queue_tx.pop();

                if (rx_msg.find("CAMUC") != string::npos) {
                    if (m_status_tx == _MPS) {
                        m_status_tx++;
                    } else {
                        printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CAMUC \033[0m\n", m_status_tx);
                    }

                } else if (rx_msg.find("CATXP") != string::npos) {
                    if (!pmModem->getBin()) {
                        if (m_status_tx == _MPR) {
                            m_status_tx++;
                        } else {
                            printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CATXP (MP)\033[0m\n", m_status_tx);
                        }
                    } else {
                        if (m_status_tx == _BINSR) {
                            m_status_tx++;
                        } else if (m_status_tx == _CINITR) {
                            m_status_tx++;
                        } else {
                            printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CATXP (BIN)\033[0m\n", m_status_tx);
                        }
                    }

                } else if (rx_msg.find("CATXF") != string::npos) {
                    if (!pmModem->getBin()) {
                        if (m_status_tx == _MPST) {
                            m_status_tx++;
                        } else {
                            printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CATXF (MP)\033[0m\n", m_status_tx);
                        }
                    } else {
                        if (m_status_tx == _BINST) {
                            m_status_tx++;
                        } else if (m_status_tx == _CINITST) {
                            m_status_tx++;
                        } else {
                            printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CATXF (BIN)\033[0m\n", m_status_tx);
                        }
                    }

                } else if (rx_msg.find("CAXST") != string::npos) {
                    if (m_status_tx == _MPET) {
                        m_status_tx = _IDLE;
                        status = _IDLE;
                        ongoing_tx = false;
                    } else if (m_status_tx == _BINET) {
                        m_status_tx = _IDLE;
                        status = _IDLE;
                        ongoing_tx = false;
                    } else if (m_status_tx == _CINITET) {
                        m_status_tx++;
                    } else {
                        printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CAXST\033[0m\n", m_status_tx);
                    }

                    cread = false;
                } else if (rx_msg.find("CACFG") != string::npos) {
                    if (m_status_tx == _SETIDS) {
                        m_status_tx = _IDLE;
                        status = _IDLE;
                        cread = false;
                } else {
                        printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CACFG\033[0m\n", m_status_tx);
                    }
                } else if (rx_msg.find("CACYC") != string::npos) {
                    mInterpreter.parse_cacyc(rx_msg);
                    if (m_status_tx == _CINITS) {
                        m_status_tx++;
                    } else {
                        printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CACYC\033[0m\n", m_status_tx);
                    }
                } else if (rx_msg.find("CADRQ") != string::npos) {
                    //cout << "CADRQ received !! " << endl;
                    mInterpreter.parse_cadrq(rx_msg);
                    if (m_status_tx == _CINITSS) {
                        m_status_tx = _BIN;
                        modemTxManager();
                    } else {
                        printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CADRQ\033[0m\n", m_status_tx);
                    }

                } else if (rx_msg.find("CATXD") != string::npos) {
                    if (m_status_tx == _BINS) {
                        mInterpreter.parse_catxd(rx_msg);
                        m_status_tx++;
                    } else {
                        printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CATXD\033[0m\n", m_status_tx);
                    }

                }/*else if (rx_msg.find("CATXF") != string::npos) {
                    if (m_status_tx == _BINST) {
                        m_status_tx++;
                    } else {
                        printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CATXF\n", m_status_tx);
                    }

                } */ else {
                    // incorrect packet in queue_tx 
                    cread = false;
                }
            } else {
                // queue_tx empty
                cread = false;
            }
        }
    } else { // status = _RX, read from queue_rx

        while (cread) {
            if (!queue_rx.empty()) {
                rx_msg = queue_rx.front();
                queue_rx.pop();

                if (rx_msg.find("CADQF") != string::npos) {
                    if (m_status_rx == _IDLE) {
                        m_status_rx = _RXS;
                        status = _RX;
                        cread = false;
                    } else if (m_status_rx == _RXCINITST) {
                        m_status_rx = _RXS;
                    } else {
                        printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CADQF\033[0m\n", m_status_rx);
                    }
                } else if (rx_msg.find("CAMUA") != string::npos) {
                    if (m_status_rx == _RXS) {
                        m_status_rx = _RXMP;
                        status = _IDLE_RX;
                        mInterpreter.parse_camua(rx_msg);
                        cread = false;
                    } else {
                        printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CAMUA\033[0m\n", m_status_rx);
                    }
                } else if (rx_msg.find("CACST") != string::npos) {
                    if (m_status_rx == _RXMP || m_status_rx == _RXBIN) {
                        m_status_rx = _IDLE;
                        status = _IDLE;
                        cread = false;
                    } else if (m_status_rx == _RXCINIT) {
                        m_status_rx--;
                    } else {
                        printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CACST\033[0m\n", m_status_rx);
                    }
                } else if (rx_msg.find("CACYC") != string::npos) {
                    if (m_status_rx == _RXS) {
                        mInterpreter.parse_cacyc(rx_msg);
                        m_status_rx--;
                        //cread = false;
                    } else {
                        printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CACYC\033[0m\n", m_status_rx);
                    }
                } else if (rx_msg.find("CARXD") != string::npos) {
                    if (m_status_rx == _RXS) {
                        m_status_rx = _RXBIN;
                        status = _IDLE_RX;
                        mInterpreter.parse_carxd(rx_msg);
                        cread = false;
                    } else {
                        printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CARXD\033[0m\n", m_status_rx);
                    }
                } else if (rx_msg.find("CARXA") != string::npos) {
                     if (m_status_rx == _RXS) {
                        //m_status_rx = _RXBIN;
                        //status = _IDLE_RX;
                         cout << "Received a CARXA " << rx_msg << endl;
                        mInterpreter.parse_carxa(rx_msg);
                        cread = false;
                    } else {
                        printf("PACKET in\033[0;31m WARNING: Unknown transition, current status: %d - received a CARXA\033[0m\n", m_status_rx);
                    }
                }
                
                else {

                    // incorrect packet in queue_rx. Exit queue
                    cread = false;
                }
            } else {
                cread = false;
            }
        }
    }

    return status;
}

void MdriverFSK_WHOI_MM::modemTxManager() {

    if (debug_) {
        cout << this << ": in modemTxManager() method of MdriverFSK_WHOI_MM. Note: ID = " << ID << ", m_status_tx = " << m_status_tx << "\n";
    }

    // Variable to store the message to be sent to the modem
    std::string tx_msg;

    if (status == _TX || status == _CFG /*|| status == _TX_RX*/) {
        switch (m_status_tx) {
            case _SETID:
                // Build the configuration message
                tx_msg = mInterpreter.build_cccfg("SRC", ID);
                break;
            case _CINIT:
                // Build the cycle init-message
                //tx_msg = mInterpreter.build_cccyc(ID, dest, 0, 1);
                //tx_msg = mInterpreter.build_cccyc(ID, 255, 0, 1);
                tx_msg = mInterpreter.build_cccyc(ID, 2, 0, 1);
                break;
            case _BIN:
                // Build the binary data message 
                //tx_msg = mInterpreter.build_cctxd(ID, dest, 0, payload_tx);
                tx_msg = mInterpreter.build_cctxd(ID, 2, 0, payload_tx);
                //tx_msg = mInterpreter.build_cctxa(ID, dest, 0, payload_tx);
                break;
            case _MP:
                // Build the binary data message 
                cout << "Build CCMUC! " << endl;
                tx_msg = mInterpreter.build_ccmuc(ID, dest, payload_tx);
                break;
            default:
                cout << "WARNING in MdriverFSK_WHOI_MM::modemTxManager(): unexpected call to this method. Status: " << status << "; m_status_tx = " << m_status_tx << " . Note: ID = " << ID << "\n";
        } // End switch
        // Send messages if at the beginning of the transmission
        if (m_status_tx == _CINIT || m_status_tx == _BIN || m_status_tx == _MP || m_status_tx == _SETID) {
            // Set to true the flag of the ongoing tx
            if (m_status_tx != _SETID) ongoing_tx = true;
            // Send the message to the modem
            mConnector.writeToModem(tx_msg);
            // Update the FSK tx status
            m_status_tx++;
        }
    } else {
        cout << "ERROR in mdriverFSK_WHOI_MM.cc::modemTxManager():  modem status flag unexpectedly not set to _TX = 1 nor to _CFG = 4 but to " << status << ". Note: ID = " << ID << "\n";
    }
} 