#ifndef _BLUEJAY_SYSTEM_INTERFACE_H_
#define _BLUEJAY_SYSTEM_INTERFACE_H_

/**
 * Defines a global base class for interfaces.
 * 
 * Only used to check if an interface is available
 * Also disables any copy of an interface
 * 
 * Interfaces should ensure that no more work is done than absolutely necessary when 
 * an function pushing function is called. Almost interfaces should pull only and don't push anything
 * 
 * ALERT: all subclasses should ensure that the interface is enabled before use
 */

class SystemInterface{
public:
    SystemInterface(): _available(false) {}
    inline bool isAvailable(){
        return _available;
    }
    
    /* Virtual destructor */
    virtual ~SystemInterface() {}
protected:
    inline void set_available(bool available){
        _available = available;
    }
private:
    //WARNING: disallow copy
    SystemInterface(const SystemInterface&);
    SystemInterface& operator=(const SystemInterface&);
    bool _available;
};

#endif
