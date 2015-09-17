#ifndef _BLUEJAY_SYSTEM_INTERFACE_H_
#define _BLUEJAY_SYSTEM_INTERFACE_H_

/**
 * Defines a global base class for interfaces.
 * 
 * Only used to check if an interface is available
 * 
 * ALERT: all subclasses should ensure that the interface is enabled before use
 */

class SystemInterface{
public:
    SystemInterface(): _available(false) {}
    inline bool isAvailable(){
        return _available;
    }
protected:
    inline void set_available(bool available){
        _available = available;
    }
private:
    bool _available;
};

#endif
