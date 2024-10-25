/*
  Pms5003.h
  
  Arduino library for the Plantower PMS5003 particulate matter sensor 
  attached to the ESP8266. Should also work for ESP32 boards but currently 
  untested. The base Pms5003 class is generic and should also work for most 
  AVR boards.

  https://github.com/rhscdn/esp-plantower-pms
*/

#ifndef _PMS5003_H
#define _PMS5003_H

#if ARDUINO >= 100
#include "Arduino.h"
#include "Print.h"
#else
#include "WProgram.h"
#endif

#include <Stream.h>
#include <circular_queue/Delegate.h>

class Pms5003
{
public:
    static const uint16_t BAUD_RATE = 9600;
    static const uint16_t RESPONSE_TIMEOUT = 500; // > 2 * 66ms

    enum Report_mode
    {
        REPORT_PASSIVE = 0,
        REPORT_ACTIVE = 1
    };

    typedef struct
    {
        // Standard Particles, CF=1
        uint16_t pm10_std, pm25_std, pm100_std;
        // Atmospheric environment
        uint16_t pm10_env, pm25_env, pm100_env;
        // Total particles
        uint16_t prt_03um, prt_05um, prt_10um,
            prt_25um, prt_50um, prt_100um;
    } PMSDATA_t;

    // constructor
    Pms5003(Stream &out) : _out(out)
    {
    }

    bool set_data_reporting_mode(Report_mode mode);
    Report_mode get_data_reporting_mode();

    bool set_sleep(bool sleep);
    bool get_sleep();

    bool set_data_rampup(int secs);
    int get_data_rampup();

    bool query_data(PMSDATA_t &d);
    bool query_data(PMSDATA_t &d, int n);
    bool query_data_auto(PMSDATA_t &d);

    void serialFlush() { _clear_responses(); }

    bool timeout();
    bool crc_ok();
    bool average_data(int n, const PMSDATA_t *data_tbl, PMSDATA_t &data);

protected:
    enum Command
    {
        CMD_REPORTING_MODE,
        CMD_SLEEP_MODE,
        CMD_QUERY
    };

    void _send_cmd(Command cmd, const uint8_t *buf, uint8_t len);
    int _read_byte(long unsigned deadline);
    void _clear_responses();
    bool _read_response(Command cmd);
    String _buf_to_string(uint8_t size);

    Stream &_out;
    uint8_t _buf[32];
    Report_mode _mode = REPORT_ACTIVE; // default mode
    bool _sleep = false;               // default mode
    bool _timeout = false;
    unsigned rampup_s = 30; // PMS datasheet
};                          // end class: Pms5003


#endif
