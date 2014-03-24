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
 * @file   msun-common.h
 * @author Giovanni Toso
 * @version 1.2.0
 * 
 * \brief Common structures and definitions used by MSUN.
 * 
 * Common structures and definitions used by MSUN
 */

#ifndef __MSUN_COMMON_STRUCTURES_H_
#define __MSUN_COMMON_STRUCTURES_H_

#define SMALL_ADDR                                      /**< Addressed in MSUN from int32 to in8. */

// From ip-routing.h
#define DROP_DEST_UNREACHABLE_REASON "DUR"              /**< Reason for a drop in a <i>UWMSUN</i> module. */

#define DROP_PATH_ALREADY_PROCESSED "PAP"               /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_MAX_NUMBER_OF_HOP_IN_LIST_REACHED "MHR"    /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_WRONG_NODE_PATH_EST "WNP"                  /**< Reason for a drop in a <i>UWMSUN</i> module. */

// Drop reasons for Data packets.
#define DROP_DATA_HOPS_LENGTH_EQUALS_ZERO "DHZ"         /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define TTL_EQUALS_TO_ZERO   "TEZ"                      /**< Reason for a drop in a <i>UWFLOODING</i> module. */

// Drop reasons for Path Establishment Search packets
#define DROP_PATH_ESTABLISHMENT_SEARCH_PACKET_HOP_LIST_FULL "PSF"               /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_PATH_ESTABLISHMENT_SEARCH_PACKET_ALREADY_PROCESSED "PSA"           /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_PATH_ESTABLISHMENT_SEARCH_PACKET_WRONG_CHECKSUM "PWC"              /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_PATH_ESTABLISHMENT_SEARCH_PACKET_NO_ROUTE_TO_DESTINATION "PNR"     /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_PATH_ESTABLISHMENT_MIN_SINR_NOT_REACHED "PMS"                      /**< Reason for a drop in a <i>UWMSUN</i> module. */

// Drop reasons for Path Establishment answer packets
#define DROP_PATH_ESTABLISHMENT_ANSWER_PACKET_GARBAGE "PAG"             /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_PATH_ESTABLISHMENT_ANSWER_NODE_ROUTING_TABLE_FULL "PAF"    /**< Reason for a drop in a <i>UWMSUN</i> module. */

// Drop reasons for Sink Node
#define DROP_SINK_PACKET_TYPE_UNSOPPORTED "SPU"         /**< Reason for a drop in a <i>UWMSUN</i> module. */

// Others
#define DROP_PACKET_NOT_FOR_ME "PNM"                    /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_PACKET_WITHOUT_DIRECTION "PWD"             /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_BROKEN_LINK_DROP "BLD"                     /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_BUFFER_IS_FULL   "BIF"                     /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_DUPLICATED_DATA_PKT "DDP"                  /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_WRONG_MSUN_DST_TYPE "WTP"                  /**< Reason for a drop in a <i>UWMSUN</i> module. */

#define DROP_UNKNWOWN_TYPE_OF_DESTINATION "UTD"         /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_ERROR_BCAST_PACKET "EBP"                   /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_BCAST_DST_OUTDATED "BDO"                   /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_MAX_RETX "MTX"                             /**< Reason for a drop in a <i>UWMSUN</i> module. */
#define DROP_I_AM_NOT_A_SINK "NAS"                      /**< Reason for a drop in a <i>UWMSUN</i> module. */

namespace msun {

    enum MetricType { /* NOTE: enum class MetricType only with gcc c++11 */
        HOPCOUNT = 1,
        SNR
    };  /**< Different values of packet type for a Path Establishment packet. */

    enum DestinationType { /* NOTE: enum DestinationType : std::uint8_t { ... } only experimental in gcc c++11. */
        MSUN_UNICAST = 1,
        MSUN_BROADCAST,
        MSUN_ANYCAST,
        MSUN_UNKNOWN
    };

    enum PathEsType {
        PATH_SEARCH = 1,
        PATH_ANSWER,
        PATH_ERROR
    };

    static const bool   STACK_TRACE     = 0;        /**< Used to keep track of methods call. */
    static const int    MAX_HOP_NUMBER  = 5;        /**< Maximum number of hops contained in a <i>MSUN Path Establishment</i> packet.*/
    extern int   uid_;                              /**< Unique identifier for <i>UWMSUN</i> packets. */
    static const double MIN_SNR         = -999;     /**< Reference value for the minumun SNR measured. */
    static const double THRESHOLD_RETX  = 0.25;     /**< Value used in MSun::getPlusNumRetx() function. */
    static const double PROC_INT_BUFF   = 1/100;      /**< The buffer that contains packets generated by the current node is processed at least with this minimun probability. */
//    static const double MIN_WAIT_TIME   = 0.1;      /**< Value used to resched event in the buffer in case that there is nothing to wait for. Used to avoid busy waiting. */
} // namespace msun

#endif // __MSUN_COMMON_STRUCTURES_H_
