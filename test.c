#include <windows.h>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <chrono>
#include <thread>

using namespace std;

// -----------------------------
// Protocol constants
// -----------------------------
static const uint8_t HEADER[2] = { 0xFF, 0xFE };

// MC-control
#define MODE_POS_SPEED      0x01
#define MODE_ACC_POS        0x02
#define MODE_ACC_SPEED      0x03
#define MODE_OPEN_LOOP      0x11   // SA 전용

// Settings
#define MODE_SET_POS_CTRL   0x04
#define MODE_SET_SPD_CTRL   0x05
#define MODE_SET_ID         0x06
#define MODE_SET_BAUD       0x07
#define MODE_SET_RESPTIME   0x08
#define MODE_SET_RATEDSPD   0x09
#define MODE_SET_RESOLUTION 0x0A
#define MODE_SET_RATIO      0x0B
#define MODE_SET_ONOFF      0x0C
#define MODE_SET_POSMODE    0x0D
#define MODE_SET_CTRLDIR    0x0E
#define MODE_RESET_POS      0x0F
#define MODE_RESET_FACTORY  0x10

// Feedback Requests
#define REQ_PING           0xA0
#define REQ_POS            0xA1
#define REQ_SPD            0xA2
#define REQ_POS_CTRL       0xA3
#define REQ_SPD_CTRL       0xA4
#define REQ_RESP_TIME      0xA5
#define REQ_RATEDSPD       0xA6
#define REQ_RESOLUTION     0xA7
#define REQ_RATIO          0xA8
#define REQ_ONOFF          0xA9
#define REQ_POSMODE        0xAA
#define REQ_CTRLDIR        0xAB
#define REQ_FIRMWARE       0xCD


// =====================================================
// Utility
// =====================================================
static uint16_t be16(uint8_t hi, uint8_t lo)
{
    return ((uint16_t)hi << 8) | lo;
}

// 체크섬: 헤더(2바이트) & checksum 바이트 제외한 모든 바이트 합의 NOT
uint8_t calcChecksum(const vector<uint8_t>& frame)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < frame.size(); i++)
    {
        if (i == 0 || i == 1 || i == 4) continue;
        sum += frame[i];
    }
    return (uint8_t)(~(sum & 0xFF));
}

// 프레임 생성
vector<uint8_t> buildFrame(uint8_t id, uint8_t mode, const vector<uint8_t>& data)
{
    size_t size = 1 + 1 + data.size();
    vector<uint8_t> frame(6 + data.size());

    frame[0] = 0xFF;
    frame[1] = 0xFE;
    frame[2] = id;
    frame[3] = (uint8_t)size;
    frame[5] = mode;

    if (!data.empty())
        memcpy(&frame[6], &data[0], data.size());

    frame[4] = calcChecksum(frame);
    return frame;
}

// 프레임 출력
void printFrame(const string& tag, const vector<uint8_t>& f)
{
    cout << tag;
    for (auto b : f)
        cout << " " << hex << setw(2) << setfill('0') << (int)b;
    cout << dec << endl;
}

// =====================================================
// Windows Serial Class
// =====================================================
class SerialPort
{
private:
    HANDLE hSerial = INVALID_HANDLE_VALUE;

public:
    bool openPort(const string& port, uint32_t baud)
    {
        hSerial = CreateFileA(
            port.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            0,
            NULL
        );

        if (hSerial == INVALID_HANDLE_VALUE)
        {
            cout << "Failed to open port" << endl;
            return false;
        }

        DCB dcbSerialParams = { 0 };
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

        if (!GetCommState(hSerial, &dcbSerialParams))
            return false;

        dcbSerialParams.BaudRate = baud;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;

        SetCommState(hSerial, &dcbSerialParams);
        return true;
    }

    void closePort()
    {
        if (hSerial != INVALID_HANDLE_VALUE)
            CloseHandle(hSerial);
    }

    bool writeBytes(const vector<uint8_t>& data)
    {
        DWORD written;
        BOOL ok = WriteFile(hSerial, data.data(), data.size(), &written, NULL);

        return ok && written == data.size();
    }

    vector<uint8_t> readBytes()
    {
        uint8_t buf[1024];
        DWORD dwRead = 0;
        ReadFile(hSerial, buf, 1024, &dwRead, NULL);

        vector<uint8_t> out;
        for (size_t i = 0; i < dwRead; i++)
            out.push_back(buf[i]);

        return out;
    }
};


// ================================================
// StreamParser (FF FE + LEN+4 기준 파싱)  :contentReference[oaicite:2]{index=2}
// ================================================
class StreamParser
{
private:
    vector<uint8_t> buf;

public:
    vector<vector<uint8_t>> feed(const vector<uint8_t>& chunk)
    {
        vector<vector<uint8_t>> frames;
        buf.insert(buf.end(), chunk.begin(), chunk.end());

        while (true)
        {
            // FF FE 검색
            size_t pos = string::npos;
            for (size_t i = 0; i + 1 < buf.size(); i++)
            {
                if (buf[i] == 0xFF && buf[i + 1] == 0xFE)
                {
                    pos = i;
                    break;
                }
            }

            if (pos == string::npos)
            {
                if (buf.size() > 1)
                    buf.erase(buf.begin(), buf.end() - 1);
                break;
            }

            if (pos > 0)
                buf.erase(buf.begin(), buf.begin() + pos);

            if (buf.size() < 6)
                break;

            uint8_t len = buf[3];
            size_t total = len + 4;
            if (buf.size() < total)
                break;

            vector<uint8_t> frame(buf.begin(), buf.begin() + total);

            // 체크섬 확인
            if (calcChecksum(frame) == frame[4])
                frames.push_back(frame);

            buf.erase(buf.begin(), buf.begin() + total);
        }

        return frames;
    }
};


// ================================================
// Build control commands
// ================================================
vector<uint8_t> cmd_pos_speed(uint8_t id, bool cw, float pos_deg, float spd)
{
    uint16_t pos_raw = (uint16_t)(pos_deg * 100.0f);   // 0.01°
    uint16_t spd_raw = (uint16_t)(spd * 10.0f);        // 0.1RPM

    vector<uint8_t> d;
    d.push_back(cw ? 0x01 : 0x00);
    d.push_back((pos_raw >> 8) & 0xFF);
    d.push_back(pos_raw & 0xFF);
    d.push_back((spd_raw >> 8) & 0xFF);
    d.push_back(spd_raw & 0xFF);

    return buildFrame(id, MODE_POS_SPEED, d);
}

vector<uint8_t> cmd_acc_pos(uint8_t id, bool cw, float pos_deg, float arrive_s)
{
    uint16_t pos_raw = (uint16_t)(pos_deg * 100.0f);
    uint8_t arr_raw = (uint8_t)(arrive_s * 10.0f);

    vector<uint8_t> d;
    d.push_back(cw ? 1 : 0);
    d.push_back((pos_raw >> 8) & 0xFF);
    d.push_back(pos_raw & 0xFF);
    d.push_back(arr_raw);

    return buildFrame(id, MODE_ACC_POS, d);
}

vector<uint8_t> cmd_acc_speed(uint8_t id, bool cw, float spd, float arrive_s)
{
    uint16_t spd_raw = (uint16_t)(spd * 10.0f);
    uint8_t arr_raw = (uint8_t)(arrive_s * 10.0f);

    vector<uint8_t> d;
    d.push_back(cw ? 1 : 0);
    d.push_back((spd_raw >> 8) & 0xFF);
    d.push_back(spd_raw & 0xFF);
    d.push_back(arr_raw);

    return buildFrame(id, MODE_ACC_SPEED, d);
}

vector<uint8_t> cmd_open_loop(uint8_t id, bool cw, float duty_percent)
{
    uint16_t duty = (uint16_t)(duty_percent * 100);

    vector<uint8_t> d;
    d.push_back(cw ? 1 : 0);
    d.push_back((duty >> 8) & 0xFF);
    d.push_back(duty & 0xFF);

    return buildFrame(id, MODE_OPEN_LOOP, d);
}

// 안전 정지
void safe_stop(SerialPort& sp, uint8_t id)
{
    auto f = cmd_acc_speed(id, false, 0.0f, 0.2f);
    sp.writeBytes(f);
    this_thread::sleep_for(chrono::milliseconds(250));

    vector<uint8_t> off = buildFrame(id, MODE_SET_ONOFF, { 0x01 }); // Off
    sp.writeBytes(off);
}


// =====================================================
// Main Demo (파이썬 demo() 동일 로직)
// =====================================================
int main()
{
    SerialPort serial;
    if (!serial.openPort("\\\\.\\COM10", 115200))
    {
        cout << "Port open failed" << endl;
        return 0;
    }

    cout << "[OPEN] COM10 115200bps" << endl;

    uint8_t id = 0x00;

    // 제어 ON
    serial.writeBytes(buildFrame(id, MODE_SET_ONOFF, { 0x00 }));
    this_thread::sleep_for(chrono::milliseconds(50));

    // ---- 위치 속도제어 ----
    cout << "[RUN] PosSpeed 1s" << endl;
    serial.writeBytes(cmd_pos_speed(id, false, 180.0f, 5.0f));
    this_thread::sleep_for(chrono::seconds(1));
    safe_stop(serial, id);

    // ---- 가감속 위치제어 ----
    serial.writeBytes(buildFrame(id, MODE_SET_ONOFF, { 0x00 }));
    cout << "[RUN] AccPos 1s" << endl;
    serial.writeBytes(cmd_acc_pos(id, true, 360.0f, 1.0f));
    this_thread::sleep_for(chrono::seconds(1));
    safe_stop(serial, id);

    // ---- 가감속 속도제어 ----
    serial.writeBytes(buildFrame(id, MODE_SET_ONOFF, { 0x00 }));
    cout << "[RUN] AccSpeed 1s" << endl;
    serial.writeBytes(cmd_acc_speed(id, false, 100.0f, 0.5f));
    this_thread::sleep_for(chrono::seconds(1));
    safe_stop(serial, id);

    // ---- Open-loop ----
    serial.writeBytes(buildFrame(id, MODE_SET_ONOFF, { 0x00 }));
    cout << "[RUN] OpenLoop 1s" << endl;
    serial.writeBytes(cmd_open_loop(id, false, 50.0f));
    this_thread::sleep_for(chrono::seconds(1));
    safe_stop(serial, id);

    serial.closePort();
    cout << "[DONE]" << endl;

    return 0;
}
