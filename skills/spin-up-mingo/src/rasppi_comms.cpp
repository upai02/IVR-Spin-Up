#include "../include/rasppi_comms.h"
#include "pros/rtos.h"
#include <stdio.h>
#include <iostream>
#include <string.h>
// #include "../include/api.h"
// #include "../include/pros/apix.h"

using namespace std;

// pthread_mutex_t RasppiComms::__mtx = PTHREAD_MUTEX_INITIALIZER;
// pros::Mutex RasppiComms::__mtx; // mutex lock for altering __buffer and __ready 

/*
Description: Creates an instance of RasspiComms object
Inputs: None
Ouputs: None
Returns: None
Effects: Initializes private buffer and thread for listening from serial port
*/
RasppiComms::RasppiComms(int comm_ways)  {
    __comm_ways = comm_ways;
    __listen_active = false;
    __buffer = new char[256];
    __buffer_end = 0;
    // cout << "Instantiated object" << endl;
    __ready = 0;
    __lastOrder = 0;
    __startModelFlag = 0;

    // __mtx = pros::mutex_t
    __mtx = (pros::Mutex*)malloc(sizeof(pros::Mutex));
    __mtx = new pros::Mutex();
    char init_prev[8] = "IVR_ACK";
    char init_ack[8] = "IVR0ACK";
    for (int i = 0; i < 8; i++) {
        __prev_mes_ack[i] = init_prev[i];
        __mes_ack[i] = init_ack[i];
    }
    listen();  // start thread which listens from the serial bus
    // __establishConnection();  // Note that this is a blocking function so RaspPi MUST send acknowledge message
};

/*
Description: Destroys instance
Inputs: None
Outputs: None
Returns: None
Effects: Ends thread responsible for listening from serial port
*/
RasppiComms::~RasppiComms() {
    stopListen();
    // delete[] __tag;
    // delete[] __buffer;
};

/*
Description: Creates a thread (in prosv5 called a task) for listening to serial port
Inputs: None
Outputs: None
Returns: None
Effects: Creates a thread
*/
bool RasppiComms::listen() {
    __listen_active = true;
    // return (pthread_create(&__listen1, NULL, __listen, this) == 0);

    // if(__comm_ways == 1) {
        pros::Task my_task(__listen_1way, (void *)this, "ListenTask");
    // }
    // else if (__comm_ways == 2) {
    //     pros::Task my_task(__listen_2way, (void *)this, "ListenTask");
    // }
    
    return 1;
};

/*
Description: Read from private buffer
Inputs: None
Outputs:
    char* array_out -- array to write into from private buffer
Returns: None
Effects: Takes mtx lock for a period of time
*/
void RasppiComms::read_256_from_buff(char * array_out) {
    // pthread_mutex_lock(&__mtx);

    bool take;
    while (1) {
        take = __mtx->take();
        
        if (!take)
            continue;
                
        if (__ready)
            strncpy(array_out, __buffer, __buffer_end + 1);
        else {
            __mtx->give();
            continue;
        }
        __ready = 0;
        __buffer_end = 0;
        __mtx->give();
        break;
    }

    return;
}

int RasppiComms::append_coords(std::vector<double>& cood_array, char * str_array, int& idx) {
    int size = cood_array.size();
    int f = 0;

    int str_start = 0;
		for (int i = 1; i < 256; i++) {
            if (str_array[i-1] == 'F') {
                f = -1;
                break;
            }

			if (str_array[i-1] == '(')
				str_start = i;

			if (str_array[i] == ' ')
				str_start++;

			if (str_array[i] == ',') {
				str_array[i] = '\n';
				cood_array[idx] = stod(&str_array[str_start]);
				str_start = i + 1;
				idx = (idx + 1) % size;
			}


			if (str_array[i] == ')') {
				str_array[i] = '\n';
				cood_array[idx] = stod(&str_array[str_start]);
				idx = (idx + 1) % size;
				break;
			}
		}


    return f; // return updated position of idx

}

/*
Description: Stops the thread
Inputs: None
Outputs: None
Returns: None
Effects: lowers private __listen_active flag
*/
void RasppiComms::stopListen() {
    __listen_active = false;
    // __stopListen();
};

/* 
Note that since this reads the buffer, it should only be invoked when the lock is held.
Returns the position in the buffer to start reading from which is just the tag size
*/
int RasppiComms::__verify_buffer() {
    int match = 0;
    for (int i = 0; i <= __tag_size; i++) {
        if (i == __tag_size)
            match = 1;
        else if (__buffer[i] != __tag[i])
            break;
    }
    if (match)
        return __tag_size;
    else
        return -1;
}

int RasppiComms::__verify_ack() {
    int match = 0;
    for (int i = 0; i <= __ack_size; i++) {
        if (i == __ack_size)
            match = 1;
        else if (__buffer[__tag_size + i] != __ack[i])
            break;
    }
    if (match)
        return __ack_size;
    else    
        return -1;
}

/*
Description: Thread responsible for writing to private buffer from serial port
Inputs:
    void* context -- reference to RasppiComms object
Outputs: None
Returns: None
Effects: Repeatedly takes mtx lock
*/
void RasppiComms::__listen_2way(void *context) {
    char temp[256]; // temp that constantly reads from micro usb
    char prev_mes_ack[9] = "IVR_ACK\n"; // the previous ack message to send if rasppi did not receive it initialliy
    char mes_ack[9] = "IVR0ACK\n"; // acknowledge signal to send to rasppi if message was received (otherwise it will keep spamming the message)
    RasppiComms* obj_inst = (RasppiComms*)context; // cast context to object reference
    int take;
    bool same;

    /* main work that does not end until __listen_active is lowered */
    while (obj_inst->__listen_active) {

        /* take lock */
        take = obj_inst->__mtx->take();

        if (!take)
            continue;

        if (obj_inst->__ready) {
            obj_inst->__mtx->give();
            continue;
        }

        /* serial port read*/
        fread(temp, sizeof(char), 256, stdin);

        /* check for correct ordered message */
        same = true;
        for (int i = 0; i < 4; i++) {
            if (temp[i] != mes_ack[i]) {
                same = false;
            }
        }
        if (!same) {
            fwrite(prev_mes_ack, sizeof(char), 8, stdout); // rasppi probably didnt get ack so send it again
            obj_inst->__mtx->give();
            continue;
        }

        /* clear and write to buffer */
        fill(obj_inst->__buffer, obj_inst->__buffer + 256, '\n');
        obj_inst->__buffer_end = 0;
        // obj_inst->__scpy(temp + obj_inst->__tag_size);
        obj_inst->__scpy(temp);
        obj_inst->__ready = 1;

        /* write to rasppi that message was received */
        fwrite(mes_ack, sizeof(char), 8, stdout);

        prev_mes_ack[3] = mes_ack[3]; // update what the previous ack message is now 

        if (obj_inst->__lastOrder < 9) {
            obj_inst->__lastOrder++;
            mes_ack[3] = '0' + obj_inst->__lastOrder;
        }
        else {
            obj_inst->__lastOrder = 0;
            mes_ack[3] = '0';
        }

        /* release lock */
        obj_inst->__mtx->give();
        
    }

    /* free private buffer and resolve thread */
    // cout<<"destroying object instance"<<endl;
    delete[] obj_inst->__buffer;
    return;
};

void RasppiComms::startModel() {
    // int take;
    // while(1) {
    //     take = __mtx.take();
    //     if (!take)
    //         continue;
    //     __startModelFlag = 1;
    //     __mtx.give();
    //     break;

    // }
    __startModelFlag = 1;
    
    return;
}

void RasppiComms::lowerModel() {
    int take;
    while(1) {
        take = __mtx->take();
        if (!take)
            continue;
        __startModelFlag = 0;
        __mtx->give();
        break;

    }
}

void RasppiComms::__listen_1way(void *context) {
    char temp[256]; // temp that constantly reads from micro usb
    char mes_ack[4] = "IVR"; // acknowledge signal to send to rasppi if message was received (otherwise it will keep spamming the message)
    char ivr_r[5] = "IVRR";
    RasppiComms* obj_inst = (RasppiComms*)context; // cast context to object reference
    int take;
    bool same;


    /* main work that does not end until __listen_active is lowered */
    // while (obj_inst->__listen_active) {
    while(1) {
        cout<<"IVRR"; // for some reason only gets read once

        /* take lock */
        take = obj_inst->__mtx->take();

        if (!take)
            continue;

        // if (obj_inst->__startModelFlag == 0)
        

        if (obj_inst->__ready) {
            obj_inst->__mtx->give();
            continue;
        }

        /* serial port read*/
        fread(temp, sizeof(char), 256, stdin);

        /* check for correct ordered message */
        same = true;
        for (int i = 0; i < 3; i++) {
            if (temp[i] != mes_ack[i]) {
                same = false;
            }
        }
        if (!same) {
            
            obj_inst->__mtx->give();
            
            continue;
        }

        /* clear and write to buffer */
        fill(obj_inst->__buffer, obj_inst->__buffer + 256, '\n');
        obj_inst->__buffer_end = 0;
        // obj_inst->__scpy(temp + obj_inst->__tag_size);
        obj_inst->__scpy(temp);
        obj_inst->__ready = 1;

        /* release lock */
        obj_inst->__mtx->give();
        
    }

    /* free private buffer and resolve thread */
    // cout<<"destroying object instance"<<endl;
    delete[] obj_inst->__buffer;
    return;
};

/*
Description: compares tag from message to private tag
Inputs: 
    char* cmp -- message whose tag is to be checked
Outputs: None
Returns: 
    int same -- 1 if tag is correct (thus a valid message) else 0
Effects: None
*/
int RasppiComms::__compare_tag(char *cmp) {
    int same = 0;
    // for (int j = 0; j < 256 - __tag_size; j++) {
        for (int i = 0; i < __tag_size; i++) {
            same = 1;
            if (cmp[i] != __tag[i]) {
                same = 0;
                break;
            }
        }
    // }
    return same;
}

/*
Description: copies private buffer into an array
Inputs: None
Outputs: 
    char* src -- pointer to array that will contain the buffer's output
Returns:
    int idx -- number of characters copied into the output array
Effects: None
*/
int RasppiComms::__scpy(char *src) {
    int idx = 0;
    while ((src[idx] != '\n') && (__buffer_end < 256)) {
        __buffer[idx] = src[idx];
        ++__buffer_end;
        ++idx;
    }

    return idx; // idx reflects number of characters copied
}

