#ifndef _RASPPI_COMMS_H_
#define _RASPPI_COMMS_H_
#include "api.h"

class RasppiComms {
    public:

    RasppiComms(int comm_ways);

    ~RasppiComms();

    bool listen();

    void read_256_from_buff(char * array_out);

    int append_coords(std::vector<double>& cood_array, char * str_array, int& idx);

    void stopListen();

    void startModel();

    void lowerModel();

    private:

    // static pthread_mutex_t __mtx;
    pros::Mutex* __mtx;
    pthread_t __listen1;
    bool __listen_active;
    char * __buffer;
    int __buffer_end; // index last written to in the buffer
    const char __tag [4] = "IVR";
    const char __ack [4] = "ACK";
    const int __tag_size = 3;
    const int __ack_size = 3;

    int __ready;
    int __lastOrder;
    char __prev_mes_ack[8];
    char __mes_ack[8];
    int __comm_ways;
    int __startModelFlag;

    static void __listen_2way(void *context);

    static void __listen_1way(void *context);

    void __establishConnection();

    int __verify_buffer();

    int __verify_ack();

    int __compare_tag(char *cmp);

    int __scpy(char * src);

    // void __stopListen();

};

#endif // __RASPPI_COMMS_H_