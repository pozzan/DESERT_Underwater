#ifndef UWTDMA_H
#define UWTDMA_H

#include <mmac.h>
#include <queue>

#define UW_TDMA_STATUS_MY_SLOT 1 /**< Status slot active, whether TDMA modality is on >**/
#define UW_TDMA_STATUS_NOT_MY_SLOT 2 /**< Status slot not active, whether TDMA modality is on >**/

#define UW_CHANNEL_IDLE 1 // status channel idle
#define UW_CHANNEL_BUSY 2 // status channel busy


using namespace std;

class UwTDMA;

class UwTDMATimer : public TimerHandler {
public:

    UwTDMATimer(UwTDMA *m) : TimerHandler() {
        module = m;
    }
protected:
    virtual void expire(Event *e);
    UwTDMA* module;
};

class BufferTimer : public TimerHandler {
public:

    BufferTimer(UwTDMA *m) : TimerHandler() {
        module = m;
    }
protected:
    virtual void expire(Event *e);
    UwTDMA* module;
};

class UwTDMA: public MMac {
public:
	UwTDMA();

	virtual ~UwTDMA();

	virtual void change_tdma_status();
    virtual void start();
    virtual int command(int argc, const char*const* argv);
    virtual void txData();
    virtual void stateTxData();


protected:

    int slot_status; //active or not
    int channel_status;
    int debug_;
	double num_hosts;
    double host_id;
    double frame_time; // frame duration
    double guard_time; // guard time between slots
    double slot_duration; // slot duration
    UwTDMATimer tdma_timer; // tdma handler
    BufferTimer buffer_timer; // buffer handler
    std::queue<Packet*> buffer;
	virtual void recvFromUpperLayers(Packet* p);
    virtual void Phy2MacEndRx(Packet* p);
    virtual void Phy2MacStartRx(const Packet* p);
    virtual void Mac2PhyStartTx(Packet* p);


};

#endif 
