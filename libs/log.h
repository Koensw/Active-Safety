#ifndef _BLUEJAY_LOG_H_
#define _BLUEJAY_LOG_H_

#include <iostream>
#include <cstdio>
#include <cstdarg>

/* 
 * Provides logging support while this has not yet been implemented
 */

class Log{
public:
    static void fatal( const char* format, ... ) {
        va_list args;
        fprintf( stderr, "[ERROR] " );
        va_start( args, format );
        vfprintf( stderr, format, args );
        va_end( args );
        fprintf( stderr, "\n" );
    }
    
    static void info( const char* format, ... ) {
        va_list args;
        fprintf( stdout, "[INFO] " );
        va_start( args, format );
        vfprintf( stdout, format, args );
        va_end( args );
        fprintf( stdout, "\n" );
    }
};

#endif