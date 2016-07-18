//  -*- mode: c++; c-basic-offset: 4; -*-

#ifndef UWCBR_MULTIHOP_PACKET_H
#define UWCBR_MULTIHOP_PACKET_H

#include <packet.h>

#define HDR_UWCBR_MH(p) (hdr_uwcbr_mh::access(p))

extern packet_t PT_UWCBR_MH;

/** \brief Address for the multihop CBR application */
struct uwcbr_mh_addr {
    nsaddr_t ipaddr;
    uint16_t port;
};

/** \brief NS packet header used by the multihop CBR to store the
 * application level hops
 */
struct hdr_uwcbr_mh {
    static int offset_; /**< Required by the PacketHeaderManager. */
    static int &offset() { return offset_; }
    static hdr_uwcbr_mh *access(const Packet *p) { return (hdr_uwcbr_mh*) p->access(offset_); }

    static const int MAX_PATH_LENGTH = 128; 

    /** \brief Position in the path field of the next hop 
     * 
     * If this is equal to forward_begin_end the field is empty
     */
    int &forward_path_begin() { return forward_path_begin_; }

    /** Position in the path field of the last hop */
    int &forward_path_end() { return forward_path_end_; }

    /** \brief Reference to the list of hops 
     *
     * Reference to an array that holds the hops to reach the
     * destination at positions [forward_path_begin, forward_path_end)
     * and the addresses of the traversed relays in [0,
     * forward_path_begin)
     */
    uwcbr_mh_addr (&path())[MAX_PATH_LENGTH] { return path_; }
    
private:
    uwcbr_mh_addr path_[MAX_PATH_LENGTH];
    int forward_path_begin_;
    int forward_path_end_;
};

/** \brief Function to copy all the items in a container into the path
 * field and initialize the begin and end positions
 */
template<class Iter>
void hdr_uwcbr_mh_assign_path(hdr_uwcbr_mh *h, Iter begin, Iter end) {
    int i = 0;
    for (; begin != end; begin++) {
	assert(i < hdr_uwcbr_mh::MAX_PATH_LENGTH);
	h->path()[i] = *begin;
	i++;
    }
    h->forward_path_begin() = 0;
    h->forward_path_end() = i;
}

/** \brief Function to replace the destination address for the last
 * hop with the source address of the last relay
 */
inline void hdr_uwcbr_mh_update_path(hdr_uwcbr_mh *h, const uwcbr_mh_addr &last_hop) {
    int &begin = h->forward_path_begin();
    assert(begin != h->forward_path_end());
    h->path()[begin] = last_hop;
    begin++;
}

inline void hdr_uwcbr_mh_update_path(hdr_uwcbr_mh *h, const nsaddr_t &last_addr, const uint16_t &last_port) {
    uwcbr_mh_addr addr;
    addr.ipaddr = last_addr;
    addr.port = last_port;
    hdr_uwcbr_mh_update_path(h, addr);
}

#endif
