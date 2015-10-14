#ifndef _BLUEJAY_I2C_H_
#define _BLUEJAY_I2C_H_

#include <string>
#include <cstdlib>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

/* Simple class to communicate with devices on an I2C interface
 * 
 * NOTE: all functions return a status number, negative means an error occurred
 * WARNING: we are assuming that we only need one I2C port (we should have no reason to add more...)
 */

class I2C{
public:
    /* Open and close the I2C communication */
    static int start(std::string port);
    static int stop();
    /* Checks if open */
    static bool isStarted() {
        return _descriptor != 0;
    }
    /* Write to an I2C register */
    static int write(unsigned char address, unsigned char reg_address, unsigned char data, bool log = true);
    /* Read from an I2C register */
    static int read(unsigned char address, unsigned char reg_address, unsigned char &data, bool log = true);
private:
    I2C() {}
    
    static std::string _port;
    static int _descriptor;
};

#endif