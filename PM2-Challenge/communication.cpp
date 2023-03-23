#include "communication.h"
#include "mbed.h"
#include <cstdio>

Communicator::Communicator(){
    _dispatcher_thread.start(callback(this, &Communicator::run));
}

void Communicator::send_value(value_type type, int value){
    send_value(type, (double)value);
}

void Communicator::send_value(value_type type, double value){
    value_message *message = _value_messages.try_alloc();

    message->type=type;
    message->value=value;
    
    _value_messages.put(message);
}

void Communicator::run(){
    while(true){
        value_message *message = _value_messages.try_get();

        if(message){
            printf("{\"value_type\": %d, \"value\": %lf} \n", message->type, message->value);

            _value_messages.free(message);
        }


        ThisThread::sleep_for(1ms);
    }
}