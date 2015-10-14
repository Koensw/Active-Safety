#include "i2c.h"

#include "log.h"

/* DEFINE STATIC MEMBERS */
int I2C::_descriptor = 0;
std::string I2C::_port;

int I2C::start(std::string port){
    _port = port;
    if(_descriptor){
        Log::error("I2C already started %s", port.c_str());
        return -1;
    }
    
    _descriptor = open(port.c_str(), O_RDWR);
    if(_descriptor < 0){
        Log::error("Cannot open I2C port %s", _port.c_str());
        int ret_val = _descriptor;
        _descriptor = 0;
        return ret_val;
    }
    return 0;
}

int I2C::stop(){
    int ret_val = close(_descriptor);
    if(ret_val < 0) Log::error("Cannot close I2C port %s", _port.c_str());
    else _descriptor = 0;
    return ret_val;
}

int I2C::read(unsigned char address, unsigned char reg_addr, unsigned char &data, bool log){
    data = 0;
    if(!isStarted()) {
        if(log) Log::error("Trying to read data while I2C is not started yet");
        return -1;
    }
    
    unsigned char *inbuff, outbuff;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
    
    outbuff = reg_addr;
    messages[0].addr = address;
    messages[0].flags= 0;
    messages[0].len = sizeof(outbuff);
    messages[0].buf = &outbuff;
    
    inbuff = &data;
    messages[1].addr = address;
    messages[1].flags = I2C_M_RD;
    messages[1].len = sizeof(*inbuff);
    messages[1].buf = inbuff;
    
    packets.msgs = messages;
    packets.nmsgs = 2;
    
    int ret_val = ioctl(_descriptor, I2C_RDWR, &packets);
    if(log && ret_val < 0){
        Log::error("Cannot read register address %#1x from device %#1x", reg_addr, address);
    }
    
    return ret_val;
}

int I2C::write(unsigned char address, unsigned char reg_addr, unsigned char data, bool log){
    if(!isStarted()) {
        if(log) Log::error("Trying to write data while I2C is not started yet");
        return -1;
    }
    
    unsigned char buff[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];
    
    buff[0] = reg_addr;
    buff[1] = data;
    
    messages[0].addr = address;
    messages[0].flags = 0;
    messages[0].len = sizeof(buff);
    messages[0].buf = buff;
    
    packets.msgs = messages;
    packets.nmsgs = 1;
    
    int ret_val = ioctl(_descriptor, I2C_RDWR, &packets);
    if(log && ret_val < 0){
        Log::error("Cannot write data to device %#1x at register address %#1x", address, reg_addr);
    }
    
    return ret_val;
}