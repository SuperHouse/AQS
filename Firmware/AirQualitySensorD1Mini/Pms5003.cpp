/*
  Pms5003.cpp
  
  Arduino library for the Plantower PMS5003 particulate matter sensor 
  attached to the ESP8266. Should also work for ESP32 boards but currently 
  untested. The base Pms5003 class is generic and should also work for most 
  AVR boards.

  https://github.com/rhscdn/esp-plantower-pms 
*/

#include "Pms5003.h"

//#define DEBUG true

//#define ((uint8_t) ((w) & 0xff))
//#define highByte(w) ((uint8_t) ((w) >> 8))
//#define makeWord(h,l) ( (h << 8) | l)

#ifdef DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

bool Pms5003::set_data_reporting_mode(Report_mode mode)
{
    bool succ = true;
    uint8_t data[] = {0x00, mode};
    _send_cmd(CMD_REPORTING_MODE, data, 2);
    succ &= _read_response(CMD_REPORTING_MODE) && crc_ok();
    succ &= _buf[4] == 0xE1 && _buf[5] == mode;
    if (succ)
        _mode = mode;
    return succ;
}

Pms5003::Report_mode Pms5003::get_data_reporting_mode()
{
    // There is no way to query the sensor
    return _mode;
}

bool Pms5003::set_sleep(bool sleep)
{
    bool succ = true;
    uint8_t data[] = {0x00, !sleep};
    _send_cmd(CMD_SLEEP_MODE, data, 2);
    if (sleep)
    {
        succ &= _read_response(CMD_SLEEP_MODE) && crc_ok();
        succ &= _buf[4] == 0xE4 && !_buf[5] == sleep;
        if (succ)
            _sleep = true;
    }
    else
    {
        _sleep = false;        // assumed as sensor gives no response to 'wakeup' cmd
        _mode = REPORT_ACTIVE; // always 'boots' into active mode
        // Avoid blocking delay here but user must now check sensor boots before sending commands
        // delay(1300);           // sensor is slow to boot and will drop commands while booting
    }
    return succ;
}

bool Pms5003::get_sleep()
{
    // There is no way to query the sensor
    return _sleep;
}

bool Pms5003::query_data(PMSDATA_t &d)
{
    _send_cmd(CMD_QUERY, NULL, 0);
    bool succ = _read_response(CMD_QUERY);
    if (!succ || !crc_ok())
    {
        return false;
    }

    // Standard
    d.pm10_std = makeWord(_buf[4], _buf[5]);
    d.pm25_std = makeWord(_buf[6], _buf[7]);
    d.pm100_std = makeWord(_buf[8], _buf[9]);
    // Atmospheric Environment
    d.pm10_env = makeWord(_buf[10], _buf[11]);
    d.pm25_env = makeWord(_buf[12], _buf[13]);
    d.pm100_env = makeWord(_buf[14], _buf[15]);
    // Total particles
    d.prt_03um = makeWord(_buf[16], _buf[17]);
    d.prt_05um = makeWord(_buf[18], _buf[19]);
    d.prt_10um = makeWord(_buf[20], _buf[21]);
    d.prt_25um = makeWord(_buf[22], _buf[23]);
    d.prt_50um = makeWord(_buf[24], _buf[25]);
    d.prt_100um = makeWord(_buf[26], _buf[27]);

    return true;
}

bool Pms5003::query_data(PMSDATA_t &data, int n)
{
    PMSDATA_t data_tbl[n];

    for (int i = 0; n > 0 && i < n; ++i)
    {
        bool succ = query_data(data_tbl[i]);
        if (!succ)
        {
            --n;
            --i;
            continue;
        }
        delay(1000); // blocking 
    }

    average_data(n, data_tbl, data);
    return n > 0;
}

bool Pms5003::query_data_auto(PMSDATA_t &d)
{
    bool succ = _read_response(CMD_QUERY);
    if (!succ || !crc_ok())
    {
        return false;
    }

    // Standard
    d.pm10_std = makeWord(_buf[4], _buf[5]);
    d.pm25_std = makeWord(_buf[6], _buf[7]);
    d.pm100_std = makeWord(_buf[8], _buf[9]);
    // Atmospheric Environment
    d.pm10_env = makeWord(_buf[10], _buf[11]);
    d.pm25_env = makeWord(_buf[12], _buf[13]);
    d.pm100_env = makeWord(_buf[14], _buf[15]);
    // Total particles
    d.prt_03um = makeWord(_buf[16], _buf[17]);
    d.prt_05um = makeWord(_buf[18], _buf[19]);
    d.prt_10um = makeWord(_buf[20], _buf[21]);
    d.prt_25um = makeWord(_buf[22], _buf[23]);
    d.prt_50um = makeWord(_buf[24], _buf[25]);
    d.prt_100um = makeWord(_buf[26], _buf[27]);

    return true;
}

bool Pms5003::set_data_rampup(int secs)
{
    rampup_s = secs;
    return true;
}

int Pms5003::get_data_rampup()
{
    return rampup_s;
}

bool Pms5003::timeout()
{
    return _timeout;
}

bool Pms5003::crc_ok()
{
    uint16_t crc = 0;
    uint16_t crc_ex = 0;

    // Handle response or data packets
    // packet length doesn't include 4 header bytes
    uint16_t pktLen = 4 + makeWord(_buf[2], _buf[3]);

    // expected
    crc_ex = makeWord(_buf[pktLen - 2], _buf[pktLen - 1]);

    // checksum doesn't include last 2 bytes
    for (int i = 0; i < pktLen - 2; i++)
        crc += _buf[i];

    DEBUG_PRINT("Packet length: ");
    DEBUG_PRINTLN(pktLen);
    DEBUG_PRINT("Expected CRC: ");
    DEBUG_PRINTLN(crc_ex);
    DEBUG_PRINT("Calculated CRC: ");
    DEBUG_PRINTLN(crc);

    return crc == crc_ex;
}

void Pms5003::_send_cmd(Command cmd, const uint8_t *data, uint8_t len)
{
    uint8_t i;
    uint16_t crc;

    _buf[0] = 0x42;
    _buf[1] = 0x4D;

    switch (cmd)
    {
    case CMD_REPORTING_MODE:
        _buf[2] = {0xE1};
        break;
    case CMD_SLEEP_MODE:
        _buf[2] = {0xE4};
        break;
    case CMD_QUERY:
        _buf[2] = {0xE2};
        break;
    }

    crc = _buf[0] + _buf[1] + _buf[2];

    for (i = 0; i < 2; ++i) // copy data1 and data2 to _buf
    {
        if (i < len)
        {
            _buf[3 + i] = data[i];
        }
        else
        {
            _buf[3 + i] = 0;
        }
        crc += _buf[3 + i];
    }

    _buf[5] = highByte(crc);
    _buf[6] = lowByte(crc);

    DEBUG_PRINT("Sending:");
    DEBUG_PRINTLN(_buf_to_string(32));

    // clear un-read serial data before sending
    _clear_responses();
    // messages are only 7 bytes long
    _out.write(_buf, sizeof(_buf[0]) * 7);
}

int Pms5003::_read_byte(long unsigned deadline)
{
    uint32_t start = millis();
    while (!_out.available())
    {
        if (millis() - start < deadline)
        {
            delay(1);
            continue;
        }
        _timeout = true;
        return -1;
    }

    _timeout = false;
    _sleep = false; // serial data means sensor not sleeping
    return _out.read();
}

void Pms5003::_clear_responses()
{
    _out.flush();
    auto avail = _out.available();
    while (avail-- && _out.read() >= 0)
    {
    }
}

// read_response parses the returning data packet
bool Pms5003::_read_response(enum Command cmd)
{
    uint8_t i = 0;
    uint16_t pktLen = 0;
    const long unsigned deadline = 1000;

    while (i < 4)
    {
        _buf[i] = _read_byte(deadline);
        // DEBUG_PRINT(_buf[i], HEX);
        // DEBUG_PRINT(" (");
        // DEBUG_PRINT(i);
        // DEBUG_PRINT("), ");

        if (timeout())
            break;
        switch (i++)
        {
        case 0: // defined 1st byte
            if (_buf[0] != 0x42)
                i = 0;
            break;
        case 1: // defined 2nd byte
            if (_buf[1] != 0x4d)
                i = 0;
            break;
        case 2: // packet length high byte
            pktLen = _buf[2] << 8;
            break;
        case 3: // packet length low byte
            pktLen |= _buf[3];
            break;
        }
    }
    DEBUG_PRINTLN();

    DEBUG_PRINT("Read Response Packet length: ");
    DEBUG_PRINTLN(pktLen);

    // Packet data
    for (i = 4; i < (pktLen + 4); i++)
    {
        if (timeout())
            break;
        _buf[i] = _read_byte(deadline);
    }

    DEBUG_PRINTLN("Received:");
    DEBUG_PRINTLN(_buf_to_string(32));

    return !timeout();
}

String Pms5003::_buf_to_string(uint8_t size)
{
    String ret;
    for (int i = 0; i < size; i++)
    {
        if (_buf[i] < 0x10)
            ret += '0';
        ret += String(_buf[i], 16);
        if (7 == (i % 8))
            ret += '|';
        if (i < 32)
            ret += ' ';
    }
    return ret;
}

// Average the data. Not a robust method. Treats each value as independent.
// Resulting data average may or may not be self consistent.
bool Pms5003::average_data(int n, const PMSDATA_t *data_tbl, PMSDATA_t &data)
{
    if (n < 1)
        return false;

    PMSDATA_t data_max, data_min, data_sum;

    // filters each datum independently
    data_sum = data_min = data_max = data_tbl[0];
    data_sum = data_min = data_max = data_tbl[0];

    for (int i = 1; i < n; ++i)
    {
        // Standard
        // d.pm10_std, d.pm25_std, d.pm100_std
        if (data_tbl[i].pm10_std < data_min.pm10_std)
        {
            data_min.pm10_std = data_tbl[i].pm10_std;
        }
        if (data_tbl[i].pm25_std < data_min.pm25_std)
        {
            data_min.pm25_std = data_tbl[i].pm25_std;
        }
        if (data_tbl[i].pm100_std < data_min.pm100_std)
        {
            data_min.pm100_std = data_tbl[i].pm100_std;
        }
        if (data_tbl[i].pm10_std > data_max.pm10_std)
        {
            data_max.pm10_std = data_tbl[i].pm10_std;
        }
        if (data_tbl[i].pm25_std > data_max.pm25_std)
        {
            data_max.pm25_std = data_tbl[i].pm25_std;
        }
        if (data_tbl[i].pm100_std > data_max.pm100_std)
        {
            data_max.pm100_std = data_tbl[i].pm100_std;
        }
        data_sum.pm10_std += data_tbl[i].pm10_std;
        data_sum.pm25_std += data_tbl[i].pm25_std;
        data_sum.pm100_std += data_tbl[i].pm100_std;

        // Atmospheric Environment
        // d.pm10_env, d.pm25_env, d.pm100_env
        if (data_tbl[i].pm10_env < data_min.pm10_env)
        {
            data_min.pm10_env = data_tbl[i].pm10_env;
        }
        if (data_tbl[i].pm25_env < data_min.pm25_env)
        {
            data_min.pm25_env = data_tbl[i].pm25_env;
        }
        if (data_tbl[i].pm100_env < data_min.pm100_env)
        {
            data_min.pm100_env = data_tbl[i].pm100_env;
        }
        if (data_tbl[i].pm10_env > data_max.pm10_env)
        {
            data_max.pm10_env = data_tbl[i].pm10_env;
        }
        if (data_tbl[i].pm25_env > data_max.pm25_env)
        {
            data_max.pm25_env = data_tbl[i].pm25_env;
        }
        if (data_tbl[i].pm100_env > data_max.pm100_env)
        {
            data_max.pm100_env = data_tbl[i].pm100_env;
        }
        data_sum.pm10_env += data_tbl[i].pm10_env;
        data_sum.pm25_env += data_tbl[i].pm25_env;
        data_sum.pm100_env += data_tbl[i].pm100_env;

        // Total particles
        // d.prt_03um, d.prt_05um, d.prt_10um,
        // d.prt_25um, d.prt_50um, d.prt_100um
        if (data_tbl[i].prt_03um < data_min.prt_03um)
        {
            data_min.prt_03um = data_tbl[i].prt_03um;
        }
        if (data_tbl[i].prt_05um < data_min.prt_05um)
        {
            data_min.prt_05um = data_tbl[i].prt_05um;
        }
        if (data_tbl[i].prt_10um < data_min.prt_10um)
        {
            data_min.prt_10um = data_tbl[i].prt_10um;
        }
        if (data_tbl[i].prt_25um < data_min.prt_25um)
        {
            data_min.prt_25um = data_tbl[i].prt_25um;
        }
        if (data_tbl[i].prt_50um < data_min.prt_50um)
        {
            data_min.prt_50um = data_tbl[i].prt_50um;
        }
        if (data_tbl[i].prt_100um < data_min.prt_100um)
        {
            data_min.prt_100um = data_tbl[i].prt_100um;
        }
        if (data_tbl[i].prt_03um > data_max.prt_03um)
        {
            data_max.prt_03um = data_tbl[i].prt_03um;
        }
        if (data_tbl[i].prt_05um > data_max.prt_05um)
        {
            data_max.prt_05um = data_tbl[i].prt_05um;
        }
        if (data_tbl[i].prt_10um > data_max.prt_10um)
        {
            data_max.prt_10um = data_tbl[i].prt_10um;
        }
        if (data_tbl[i].prt_25um > data_max.prt_25um)
        {
            data_max.prt_25um = data_tbl[i].prt_25um;
        }
        if (data_tbl[i].prt_50um > data_max.prt_50um)
        {
            data_max.prt_50um = data_tbl[i].prt_50um;
        }
        if (data_tbl[i].prt_100um > data_max.prt_100um)
        {
            data_max.prt_100um = data_tbl[i].prt_100um;
        }
        data_sum.prt_03um += data_tbl[i].prt_03um;
        data_sum.prt_05um += data_tbl[i].prt_05um;
        data_sum.prt_10um += data_tbl[i].prt_10um;
        data_sum.prt_25um += data_tbl[i].prt_25um;
        data_sum.prt_50um += data_tbl[i].prt_50um;
        data_sum.prt_100um += data_tbl[i].prt_100um;
    }

    // Compute a simple average over n samples

    // Standard
    data.pm10_std = data_sum.pm10_std / n;
    data.pm25_std = data_sum.pm25_std / n;
    data.pm100_std = data_sum.pm100_std / n;
    // Atmospheric Environment
    data.pm10_env = data_sum.pm10_env / n;
    data.pm25_env = data_sum.pm25_env / n;
    data.pm100_env = data_sum.pm100_env / n;
    // Total particles
    data.prt_03um = data_sum.prt_03um / n;
    data.prt_05um = data_sum.prt_05um / n;
    data.prt_10um = data_sum.prt_10um / n;
    data.prt_25um = data_sum.prt_25um / n;
    data.prt_50um = data_sum.prt_50um / n;
    data.prt_100um = data_sum.prt_100um / n;

    return true;
}
