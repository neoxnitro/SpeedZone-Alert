#include "driver_gps6mv2.h"
#include <Arduino.h>

// Send UBX CFG-RATE message to set update rate to 5Hz (measurement rate = 200 ms)
void sendUbxCfgRate5Hz()
{
    const uint8_t ubx[] = {
        0xB5, 0x62, // header
        0x06, 0x08, // class, id (CFG-RATE)
        0x06, 0x00, // length = 6
        0xC8, 0x00, // measRate = 200 ms (little-endian)
        0x01, 0x00, // navRate = 1
        0x00, 0x00, // timeRef = 0 (UTC)
        0xDD, 0x68  // checksum CK_A, CK_B (precomputed)
    };

    int len = sizeof(ubx);
    int written = uart_write_bytes(GPS_UART, (const char *)ubx, len);
    Serial.printf("[GPS UBX] Sent CFG-RATE 5Hz (%d bytes) -> written=%d\n", len, written);
}

// Send UBX CFG-PRT to change GPS UART baudrate to 115200 (must be sent at current baud)
void sendUbxCfgPrt115200()
{
    const uint16_t payload_len = 20; // use 20 bytes payload (pad zeros if needed)
    uint8_t payload[payload_len];
    memset(payload, 0, sizeof(payload));

    // Build payload fields (little endian as required)
    payload[0] = 0x01; // portID = 1 (UART)
    payload[1] = 0x00; // reserved
    payload[2] = 0x00; // txReady LSB
    payload[3] = 0x00; // txReady MSB

    // mode (4 bytes)
    payload[4] = 0xD0;
    payload[5] = 0x08;
    payload[6] = 0x00;
    payload[7] = 0x00;

    // baudRate (4 bytes) little endian for 115200 = 0x0001C200 -> 00 C2 01 00
    payload[8] = 0x00;
    payload[9] = 0xC2;
    payload[10] = 0x01;
    payload[11] = 0x00;

    // inProtoMask (2 bytes) = 0x0007 (UBX+NMEA)
    payload[12] = 0x07;
    payload[13] = 0x00;
    // outProtoMask (2 bytes)
    payload[14] = 0x07;
    payload[15] = 0x00;

    // Build message buffer: header + class,id + length + payload + checksum
    const uint8_t cls = 0x06;
    const uint8_t id = 0x00;
    uint8_t msg[6 + payload_len + 2];
    int idx = 0;
    msg[idx++] = 0xB5;
    msg[idx++] = 0x62;
    msg[idx++] = cls;
    msg[idx++] = id;
    msg[idx++] = (uint8_t)(payload_len & 0xFF);
    msg[idx++] = (uint8_t)((payload_len >> 8) & 0xFF);
    memcpy(&msg[idx], payload, payload_len);
    idx += payload_len;

    // compute checksum over class,id,lengthL, lengthH and payload
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6 + payload_len; ++i)
    {
        ck_a = ck_a + msg[i];
        ck_b = ck_b + ck_a;
    }
    msg[idx++] = ck_a;
    msg[idx++] = ck_b;

    int tosend = idx;
    int written = uart_write_bytes(GPS_UART, (const char *)msg, tosend);
    Serial.printf("[GPS UBX] Sent CFG-PRT (change baud->115200) %d bytes, written=%d\n", tosend, written);
}
