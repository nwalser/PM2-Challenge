#include "communication.h"
#include "mbed.h"
#include <cstdio>

Communicator::Communicator(){
    _dispatcher_thread.start(callback(this, &Communicator::Run));
}

void Communicator::SendValue(value_type type, int value){
    SendValue(type, (double)value);
}

void Communicator::SendValue(value_type type, double value){
    value_message *message = _value_messages.try_alloc();

    message->type=type;
    message->value=value;
    
    _value_messages.put(message);
}

void Communicator::Run(){
    while(true){
        value_message *message = _value_messages.try_get();

        if(message){
            printf("{\"t\": %d, \"v\": %lf} \n", message->type, message->value);

            _value_messages.free(message);
        }
    }
}