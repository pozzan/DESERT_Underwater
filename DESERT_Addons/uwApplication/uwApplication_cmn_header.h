/** 
 * File:   uwAPPLICATION_cmh_header.h
 * Author: pc_loris87
 *
 * Created on 15 dicembre 2013, 15.48
 */

#ifndef UWAPPLICATION_CMH_HEADER_H
#define	UWAPPLICATION_CMH_HEADER_H

#include <module.h>
#include <packet.h>
#include <pthread.h>

#define MAX_LENGTH_PAYLOAD 4096
#define HDR_DATA_APPLICATION(p) (hdr_DATA_APPLICATION::access(p))     /**< alias defined to access the TRIGGER HEADER */


//pthread_mutex_t mutex_udp = PTHREAD_MUTEX_INITIALIZER;

extern packet_t PT_DATA_APPLICATION; /**< DATA packet type */

/**
 * Content header of TRIGGER packet
 */
typedef struct hdr_DATA_APPLICATION {
    uint16_t sn_;       /**< Serial number of the packet. */
    int rftt_;        /**< Forward Trip Time of the packet. */
    bool rftt_valid_;   /**< Flag used to set the validity of the fft field. */
    uint8_t priority_;     /**< Priority flag: 1 means high priority, 0 normal priority. */
    char payload_msg[MAX_LENGTH_PAYLOAD]; /**< Message payload*/
    
    static int offset_; /**< Required by the PacketHeaderManager. */

    /**
     * Reference to the sn_ variable.
     * 
     * @return uint16_t sn_ 
     */
    inline uint16_t& sn() {
        return sn_;
    }
    
   /**
     *  Reference to the rftt_ variable.
     *
     * @return float rftt_    
     */
    inline int& rftt() {
        return (rftt_);
    }
       
    /**
     *  Reference to the rftt_valid_ variable.
     * 
     * @return bool rftt_valid_
     */
    inline bool& rftt_valid() {
        return rftt_valid_;
    }
    
   /**
     *  Reference to the priority variable.
     * 
     * @return char priority_
     */
    inline uint8_t& priority() {
        return priority_;
    }
    
    /**
     * Reference to the offset variable 
     */
    inline static int& offset() {
        return offset_;
    }

    inline static hdr_DATA_APPLICATION * access(const Packet * p) {
        return (hdr_DATA_APPLICATION*) p->access(offset_);
    }
} hdr_DATA_APPLICATION;


#endif	/* UWAPPLICATION_CMH_HEADER_H */

