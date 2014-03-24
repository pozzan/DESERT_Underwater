//
// Copyright (c) 2013 Regents of the SIGNET lab, University of Padova.
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
 * @file packer-uwudp.cpp
 * @author Paolo Casari
 * \version 1.0.0
 * \brief  Implementation of the class responsible to map the NS-Miracle packet of uw-msun into a bit stream, and vice-versa.
 */

#include "packer-uwmsun.h"
#include "packer-common.h"


/**
 * Class to create the Otcl shadow object for an object of the class packer.
 */
static class PackerUWMSUNClass : public TclClass
{
public:
	PackerUWMSUNClass() : TclClass("UW/MSUN/Packer") {}
	TclObject* create(int, const char*const*) {
		return (new packerUWMSUN());
	}
} class_module_packerUWMSUN;


packerUWMSUN::packerUWMSUN():packer(false) {
    PType_Bits                   = 8*sizeof(u_int8_t);
    List_of_hops_length_Bits     = 8*sizeof(u_int8_t);
    Pointer_to_list_of_hops_Bits = 8*sizeof(u_int8_t);
    List_of_hops_Entry_Bits      = 8*sizeof(u_int8_t); 
    Quality_Bits                 = 8*sizeof(u_int8_t);               
    SAddr_Bits                   = 8*sizeof(u_int8_t);
    Uid_Bits                     = 8*sizeof(u_int8_t);
    TTL_Bits                     = 8*sizeof(u_int8_t);
    
            
    bind( "PType_Bits"                   , (int*) &PType_Bits);
    bind( "List_of_hops_length_Bits"     , (int*) &List_of_hops_length_Bits);
    bind( "Pointer_to_list_of_hops_Bits" , (int*) &Pointer_to_list_of_hops_Bits);
    bind( "List_of_hops_Entry_Bits"      , (int*) &List_of_hops_Entry_Bits);
    bind( "Quality_Bits"                 , (int*) &Quality_Bits);
    bind( "SAddr_Bits"                   , (int*) &SAddr_Bits);
    bind( "Uid_Bits"                     , (int*) &Uid_Bits);
    bind( "TTL_Bits"                     , (int*) &TTL_Bits);
    
    this->init();
    // std::vector<size_t> nbits_(7); // Set in packer_uwmsun.h
}

packerUWMSUN::~packerUWMSUN(){    
}

void packerUWMSUN::init() {
    if ( debug_ )
        cout << "Initialization of n_bits for the UWMSUN packer " << std::endl;
    
    n_bits.clear();
    n_bits.assign(8,0); // Seven zeroes as below
    
    // Field sizes in bits used by PATH EST packets
    n_bits[P_EST_TYPE]              = PType_Bits;
    n_bits[QUALITY]                 = Quality_Bits;
    // Field sizes in bits for PATH EST and DATA packets
    n_bits[LIST_OF_HOPS_LENGTH]     = List_of_hops_length_Bits;
    n_bits[POINTER_TO_LIST_OF_HOPS] = Pointer_to_list_of_hops_Bits;
    n_bits[LIST_OF_HOPS_ENTRY]      = List_of_hops_Entry_Bits;
    // Field sizes in bits for ACK packets
    n_bits[SADDR]                   = SAddr_Bits;
    n_bits[UID]                     = Uid_Bits;
    // Field size for the TTL in the broadcast case
    n_bits[TTL]                     = TTL_Bits;
}

size_t packerUWMSUN::packMyHdr(Packet* p, unsigned char* buf, size_t offset) {
	hdr_cmn* ch = HDR_CMN(p);
        
        if ( ch->ptype() == PT_MSUN_ACK ) {
            // MSUN ACK
            hdr_msun_ack* hMsunAck = HDR_MSUN_ACK(p);
            offset += put( buf , offset , &(hMsunAck->saddr_) , n_bits[SADDR] );
            offset += put( buf , offset , &(hMsunAck->uid_)   , n_bits[UID]   );
        } else if ( ch->ptype() == PT_MSUN_PATH_EST ) {
            // MSUN PATH EST
            hdr_msun_path_est* hMsunPEst = HDR_MSUN_PATH_EST(p);
            
            if ( hMsunPEst->ptype() == msun::PATH_SEARCH || hMsunPEst->ptype() == msun::PATH_ANSWER || hMsunPEst->ptype() == msun::PATH_ERROR  ) { // Redundant, only for double-check
                offset += put( buf , offset , &(hMsunPEst->ptype_)                   , n_bits[P_EST_TYPE]              );
                offset += put( buf , offset , &(hMsunPEst->list_of_hops_length_)     , n_bits[LIST_OF_HOPS_LENGTH]     );
                offset += put( buf , offset , &(hMsunPEst->pointer_to_list_of_hops_) , n_bits[POINTER_TO_LIST_OF_HOPS] );
                for ( int it_hops = 0 ; it_hops < hMsunPEst->list_of_hops_length() ; it_hops++ )
                    offset += put( buf , offset , &(hMsunPEst->list_of_hops_[it_hops])   , n_bits[LIST_OF_HOPS_ENTRY]      );
                if ( hMsunPEst->ptype() == msun::PATH_SEARCH || hMsunPEst->ptype_ == msun::PATH_ANSWER ) // Pack quality field only if not PATH ERROR
                            offset += put( buf , offset , &(hMsunPEst->quality_)                 , n_bits[QUALITY]                 );                
            } else {
                cout << "\033[0;0;31m ERROR: \033[0m " << "In packerUWMSUN::packMyHdr: undefined Path Establishment packet type, or header packing not implemented." 
                        "        Path Est. type requested:" << hMsunPEst->ptype() << std::endl;
            }
        // Unrecognized type    
        } else {
//        } else if ( ch->ptype() == PT_UWCBR ) {
            // MSUN DATA
            hdr_uwip* hip = HDR_UWIP(p);
            
/*            cout << "\033[0;0;31m ERROR: \033[0m " << "In packerUWMSUN::packMyHdr: undefined packet type, or header packing not implemented.\n" 
                 << "        Packet type requested: " << ch->ptype() << std::endl;
*/            
            if ( hip->daddr() == UWIP_BROADCAST ) {
                // Manage broadcast packet
                hdr_msun_broadcastdata* hMsubBCastData = HDR_MSUN_BROADCASTDATA(p);
                offset += put( buf , offset , &(hMsubBCastData->ttl_)     , n_bits[TTL] );
            } else {
                // Manage normal source-routed data packet 
                hdr_msun_data* hMsunData = HDR_MSUN_DATA(p);
                offset += put( buf , offset , &(hMsunData->list_of_hops_length_)     , n_bits[LIST_OF_HOPS_LENGTH]     );
                offset += put( buf , offset , &(hMsunData->pointer_to_list_of_hops_) , n_bits[POINTER_TO_LIST_OF_HOPS] );
                for ( int it_hops = 0 ; it_hops < hMsunData->list_of_hops_length() ; it_hops++ )
                    offset += put( buf , offset , &(hMsunData->list_of_hops_[it_hops])   , n_bits[LIST_OF_HOPS_ENTRY]      );
            }
        }
//        else {
//            std::cout << "PT_MSUN_ACK:" << PT_MSUN_ACK << std::endl;
//            std::cout << "PT_MSUN_DATA:" << PT_MSUN_DATA << std::endl;
//            std::cout << "PT_MSUN_BROADCASTDATA:" << PT_MSUN_BROADCASTDATA << std::endl;
//            std::cout << "PT_MSUN_PATH_EST:" << PT_MSUN_PATH_EST << std::endl;
//            cout << "\033[0;0;31m ERROR: \033[0m " << "In packerUWMSUN::packMyHdr: undefined packet type, or header packing not implemented.\n" 
//                 << "        Packet type requested: " << ch->ptype() << std::endl;
//            exit(-1);
//        }

	if (debug_) {
            if ( ch->ptype() == PT_MSUN_ACK ) {
                cout << "\033[1;37;45m TX MSUN ACK packer hdr \033[0m"      << std::endl;
                printMyHdrFields(p);
            } else if ( ch->ptype() == PT_MSUN_PATH_EST ) {
                cout << "\033[1;37;45m TX MSUN PATH EST packer hdr \033[0m" << std::endl;
                printMyHdrFields(p);
//            } else if ( ch->ptype() == PT_UWCBR ) {
            } else  {
                cout << "\033[1;37;45m TX MSUN DATA packer hdr \033[0m"     << std::endl;
                printMyHdrFields(p);
//            } else {
//                cout << "\033[0;0;31m ERROR: \033[0m " << "In packerUWMSUN::packMyHdr: undefined packet type for header printing in debug block.\n" 
//                     << "        Packet type requested: " << ch->ptype() << std::endl;
            }
	}
    return offset;
}


size_t packerUWMSUN::unpackMyHdr(unsigned char* buf, size_t offset, Packet* p) {
        hdr_cmn* ch = HDR_CMN(p);
        	
	// Assumes that the CH packet type has been already unpacked!!!
        if ( ch->ptype() == PT_MSUN_ACK ) {
            hdr_msun_ack* hMsunAck = HDR_MSUN_ACK(p);

            memset( &( hMsunAck->saddr_ ) , 0 , sizeof( hMsunAck->saddr_ ) );
            offset += get( buf , offset , &(hMsunAck->saddr_) , n_bits[SADDR] );
            memset( &( hMsunAck->uid_ )   , 0 , sizeof( hMsunAck->uid_ )   );
            offset += get( buf , offset , &(hMsunAck->uid_)   , n_bits[UID]   );
            if (debug_) {
                cout << "\033[1;37;45m RX MSUN ACK packer hdr \033[0m"     << std::endl;
                printMyHdrFields(p);
            }
            
        } else if ( ch->ptype() == PT_MSUN_PATH_EST ) {
            hdr_msun_path_est* hMsunPEst = HDR_MSUN_PATH_EST(p);

            memset( &( hMsunPEst->ptype_ ) , 0 , sizeof( hMsunPEst->ptype_ ) );
            offset += get( buf , offset , &(hMsunPEst->ptype_) , n_bits[P_EST_TYPE] );
            if ( hMsunPEst->ptype() == msun::PATH_SEARCH || hMsunPEst->ptype() == msun::PATH_ANSWER || hMsunPEst->ptype() == msun::PATH_ERROR ) { // Redundant, only to double-check 
                    memset( &( hMsunPEst->list_of_hops_length_ ) , 0 , sizeof( hMsunPEst->list_of_hops_length_ ) );
                    offset += get( buf , offset , &(hMsunPEst->list_of_hops_length_) , n_bits[LIST_OF_HOPS_LENGTH] );
                    memset( &( hMsunPEst->pointer_to_list_of_hops_ ) , 0 , sizeof( hMsunPEst->pointer_to_list_of_hops_ ) );
                    offset += get( buf , offset , &(hMsunPEst->pointer_to_list_of_hops_) , n_bits[POINTER_TO_LIST_OF_HOPS] );
                    hMsunPEst->pointer_to_list_of_hops_ = restoreSignedValue(hMsunPEst->pointer_to_list_of_hops_, n_bits[POINTER_TO_LIST_OF_HOPS]);
                    
                    int it_hops;
                    for ( it_hops = 0 ; it_hops < hMsunPEst->list_of_hops_length() ; it_hops++ ) {
                        memset( &( hMsunPEst->list_of_hops_[it_hops] ) , 0 , sizeof( hMsunPEst->list_of_hops_[it_hops] ) );
                        offset += get( buf , offset , &(hMsunPEst->list_of_hops_[it_hops]) , n_bits[LIST_OF_HOPS_ENTRY] );
                    }
                    for ( ; it_hops < msun::MAX_HOP_NUMBER ; it_hops++ )
                        memset( &( hMsunPEst->list_of_hops_[it_hops] ) , 0 , sizeof( hMsunPEst->list_of_hops_[it_hops] ) );
                    if ( hMsunPEst->ptype() == msun::PATH_SEARCH || hMsunPEst->ptype() == msun::PATH_ANSWER ) {
                        memset( &( hMsunPEst->quality_ ) , 0 , sizeof( hMsunPEst->quality_ ) );
                        offset += get( buf , offset , &(hMsunPEst->quality_) , n_bits[QUALITY] );
                    }
            } else {
                cout << "\033[0;0;31m ERROR: \033[0m " << "In packerUWMSUN::unpackMyHdr: undefined Path Establishment type, or header packing not implemented.\n" 
                     << "        Path Est type requested: " << hMsunPEst->ptype() << std::endl;
                exit(-1);
            }
            if (debug_) {
                cout << "\033[1;37;45m RX MSUN PATH ESTABLISHMENT packer hdr \033[0m"     << std::endl;
                printMyHdrFields(p);
            }
//        } else if ( ch->ptype() == PT_UWCBR ) {
        } else {
            hdr_uwip* hip = HDR_UWIP(p);
            
            if ( hip->daddr() == UWIP_BROADCAST ) {
                // Manage broadcast packet
                hdr_msun_broadcastdata* hMsunBCastData = HDR_MSUN_BROADCASTDATA(p);

                memset( &( hMsunBCastData->ttl_ ) , 0 , sizeof( hMsunBCastData->ttl_ ) );
                offset += get( buf , offset , &(hMsunBCastData->ttl_) , n_bits[TTL] );
            } else {
                // Manage normal source-routed packet
                hdr_msun_data* hMsunData = HDR_MSUN_DATA(p);
                
                memset( &( hMsunData->list_of_hops_length_ ) , 0 , sizeof( hMsunData->list_of_hops_length_ ) );
                offset += get( buf , offset , &(hMsunData->list_of_hops_length_) , n_bits[LIST_OF_HOPS_LENGTH] );

                memset( &( hMsunData->pointer_to_list_of_hops_ ) , 0 , sizeof( hMsunData->pointer_to_list_of_hops_ ) );
                offset += get( buf , offset , &(hMsunData->pointer_to_list_of_hops_) , n_bits[POINTER_TO_LIST_OF_HOPS] );
                hMsunData->pointer_to_list_of_hops_ = restoreSignedValue(hMsunData->pointer_to_list_of_hops_, n_bits[POINTER_TO_LIST_OF_HOPS]);

                int it_hops;
                for ( it_hops = 0 ; it_hops < hMsunData->list_of_hops_length() ; it_hops++ ) {
                    memset( &( hMsunData->list_of_hops_[it_hops] ) , 0 , sizeof( hMsunData->list_of_hops_[it_hops] ) );
                    offset += get( buf , offset , &(hMsunData->list_of_hops_[it_hops]) , n_bits[LIST_OF_HOPS_ENTRY] );
                }
                for ( ; it_hops < msun::MAX_HOP_NUMBER ; it_hops++ )
                    memset( &( hMsunData->list_of_hops_[it_hops] ) , 0 , sizeof( hMsunData->list_of_hops_[it_hops] ) );
            }
            if (debug_) {
                cout << "\033[1;37;45m RX MSUN DATA packer hdr \033[0m"     << std::endl;
                printMyHdrFields(p);
            }
        }
//        else {
//            cout << "\033[0;0;31m ERROR: \033[0m " << "In packerUWMSUN::unpackMyHdr: undefined packet type, or header packing not implemented"
//                 << "        Packet type requested: " << ch->ptype() << std::endl;
//            exit(-1);
//        }

        return offset;
}


void packerUWMSUN::printMyHdrMap() {
   cout << "\033[1;37;45m Packer Name \033[0m: UWMSUN \n";
   cout << "** Path Establishment fields:\n" ;
   cout << "\033[1;37;45m Field: PathEst packet type \033[0m:"        << n_bits[P_EST_TYPE]              << " bits\n" ;
   cout << "\033[1;37;45m Field: Quality \033[0m : "                  << n_bits[QUALITY]                 << " bits \n" ;
   cout << "** Path Establishment and Data fields:\n" ;
   cout << "\033[1;37;45m Field: List of hops length \033[0m : "      << n_bits[LIST_OF_HOPS_LENGTH]     << " bits \n" ;
   cout << "\033[1;37;45m Field: Pointer to list of hops \033[0m : "  << n_bits[POINTER_TO_LIST_OF_HOPS] << " bits \n" ;
   cout << "\033[1;37;45m Field: List of hops entry \033[0m : "       << n_bits[LIST_OF_HOPS_ENTRY]      << " bits \n" ;
   cout << "\033[1;37;45m Field: TTL \033[0m : "                      << n_bits[TTL]                     << " bits \n" ;
   cout << "** ACK fields:\n" ;
   cout << "\033[1;37;45m Field: Source address: \033[0m : "          << n_bits[SADDR]                   << " bits \n" ;
   cout << "\033[1;37;45m Field: UID: \033[0m : "                     << n_bits[UID]                     << " bits"    ;
   
   cout << std::endl ; // Only at the end do we actually flush the buffer and print
   
}


void packerUWMSUN::printMyHdrFields(Packet* p) {
    hdr_cmn* ch = HDR_CMN(p);
    
    if ( ch->ptype() == PT_MSUN_ACK ) {
            hdr_msun_ack* hMsunAck = HDR_MSUN_ACK(p);
            cout << "\033[1;37;45m 1st field \033[0m, saddr: " << (unsigned int) hMsunAck->saddr() << std::endl
                 << "\033[1;37;45m 2nd field \033[0m, uid: "   << (unsigned int) hMsunAck->uid()   << std::endl ;
    }  else if ( ch->ptype() == PT_MSUN_PATH_EST ) {
            hdr_msun_path_est* hMsunPEst = HDR_MSUN_PATH_EST(p);
            
            if ( hMsunPEst -> ptype() == msun::PATH_SEARCH || hMsunPEst -> ptype() == msun::PATH_ANSWER || hMsunPEst -> ptype() == msun::PATH_ERROR ) { // Redundant, only to double-check
                cout << "\033[1;37;45m 1st field \033[0m, ptype: "                               << (unsigned int) hMsunPEst->ptype_                   << std::endl
                     << "\033[1;37;45m 2nd field \033[0m, list_of_hops_length: "                 << (unsigned int) hMsunPEst->list_of_hops_length_     << std::endl
                     << "\033[1;37;45m 3rd field \033[0m, pointer_to_list_of_hops: "             << (int) hMsunPEst->pointer_to_list_of_hops_ << std::endl ;
                for ( int it_hops = 0 ; it_hops < hMsunPEst->list_of_hops_length() ; it_hops++ ) 
                    cout << "\033[1;37;45m 4th field \033[0m, list_of_hops[" << it_hops << "]: " << (unsigned int) hMsunPEst->list_of_hops_[it_hops]   << std::endl ;
            } else {
                cout << "\033[0;0;31m ERROR: \033[0m " << "In packerUWMSUN::printMyHdrFields: undefined Path Establishment packet type, or header packing not implemented." << std::endl;
                exit(-1);
            }
            if ( hMsunPEst -> ptype() == msun::PATH_SEARCH || hMsunPEst -> ptype() == msun::PATH_ANSWER )
                        cout << "\033[1;37;45m 5th field \033[0m, quality:"                      << (int) hMsunPEst->quality_                 << std::endl ;
//    } else if ( ch->ptype() == PT_UWCBR ) {
    } else {
        hdr_uwip* hip = HDR_UWIP(p);
        
        if ( hip->daddr() == UWIP_BROADCAST ) {
            // Manage broadcast packet
            hdr_msun_broadcastdata* hMsunBCastData = HDR_MSUN_BROADCASTDATA(p);
            
            cout << "\033[1;37;45m 1nd field \033[0m, TTL: "             << (unsigned int) hMsunBCastData->ttl_     << std::endl;
        } else {
            // Manage normal source-routed packet
            hdr_msun_data* hMsunData = HDR_MSUN_DATA(p);
            
            cout << "\033[1;37;45m 1nd field \033[0m, list_of_hops_length: "             << (unsigned int) hMsunData->list_of_hops_length_     << std::endl
                 << "\033[1;37;45m 2rd field \033[0m, pointer_to_list_of_hops: "          << (int) hMsunData->pointer_to_list_of_hops_ << std::endl ;
            for ( int it_hops = 0 ; it_hops < hMsunData->list_of_hops_length() ; it_hops++ ) 
                cout << "\033[1;37;45m 3th field \033[0m, list_of_hops[" << it_hops << "]: " << (unsigned int) hMsunData->list_of_hops_[it_hops]   << std::endl ;
        }

    }
//    else {
//        cout << "\033[0;0;31m ERROR: \033[0m " << "In packerUWMSUN::printMyHdrFields: undefined packet type, or header packing not implemented\n" 
//             << "        Packet type requested: " << ch->ptype() << std::endl;
//        exit(-1);
//    }
}

void packerUWMSUN::printMyHdrField(Packet* p, int field){    
    cout << "\033[0;0;31m ERROR: \033[0m " 
         << "In packerUWMSUN::printMyHdrField: requested printing of field " << field << "but this function should not get called!" << std::endl;
    exit(-1);
}
