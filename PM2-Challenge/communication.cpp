#include "communication.h"
#include "mbed.h"
#include <cstdio>

Communicator::Communicator(){
    _dispatcher_thread.start(callback(this, &Communicator::run));
}

void Communicator::sendValue(value_type type, int value){
    sendValue(type, (double)value);
}

void Communicator::sendValue(value_type type, double value){
    value_message *message = _value_messages.try_alloc();

    message->type=type;
    message->value=value;
    
    _value_messages.put(message);
}

void Communicator::run(){
    while(true){
        value_message *message = _value_messages.try_get();

        if(message){
            printf("{\"t\": %d, \"v\": %lf} \n", message->type, message->value);

            _value_messages.free(message);
        }
    }
}