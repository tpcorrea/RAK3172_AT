#ifndef RAK3172_H
#define RAK3172_H

#include <stdio.h>
#include <string.h>
#include <Arduino.h>

#include "driver/uart.h"
#include "driver/gpio.h"

#define EVT_AT_RESPONSE   (1 << 0)
#define EVT_JOIN_RESPONSE   0x3 // 0b0011

struct DownlinkMessage
{
    int rssi;
    int snr;
    char type[16];     // "UNICAST" ou "MULTICAST"
    int fport;
    char data[256];
};


enum class LoraRegion_t
{
    EU433    = 0,
    CN470    = 1,
    RU864    = 2,
    IN865    = 3,
    EU868    = 4,
    US915    = 5,
    AU915    = 6,
    KR920    = 7,
    AS923_1  = 8,
    AS923_2  = 9,
    AS923_3  = 10,
    AS923_4  = 11,
    LA915    = 12,
    UNKNOWN  = 13
};

enum class LoraClass_t
{
    CLASS_A = 0,
    CLASS_B = 1,
    CLASS_C = 2,
    UNKNOWN  = 3
} ;

enum class LoraMode_t
{
    ABP = 0,
    OTAA = 1,
    UNKNOWN  = 2
};

enum class RAKStatus
{
    OK,
    ERROR,
    TIMEOUT,
    INVALID_RESPONSE
};

enum class JoinStatus
{
    IDLE,
    JOINING,
    JOINED,
    JOIN_FAILED
};

class RAK3172
{
public:
    RAK3172();
    ~RAK3172();

    int init(uart_port_t uart, int tx, int rx);
    RAKStatus setRegion(LoraRegion_t region);
    LoraRegion_t getRegion();
    RAKStatus setClass(LoraClass_t lora_class);
    LoraClass_t getClass();
    RAKStatus setMode(LoraMode_t mode);
    LoraMode_t getMode();
    uart_port_t getUart();
    RAKStatus setSubBand(int band);
    int getSubBand();
    RAKStatus setADR(int adr);
    int getADR();
    RAKStatus setDR(int dr);
    int getDR();
    RAKStatus setAck(int ack);
    int getAck();
    RAKStatus setJoin1Delay(int delay_sec);
    int getJoin1Delay();
    RAKStatus setJoin2Delay(int delay_sec);
    int getJoin2Delay();
    RAKStatus setRx1Delay(int delay_sec);
    int getRx1Delay();
    RAKStatus setRx2Delay(int delay_sec);
    int getRx2Delay();
    RAKStatus setTransmitPower(int power);
    int getTransmitPower();
    int getRSSI();
    int getSNR();
    RAKStatus join(int autoJoin = 0, int interval = 8, int numAttempts = 0);
    RAKStatus send(int port, char* data);

    // Module information (read-only)
    RAKStatus getVersion(char* version);
    RAKStatus getHwModel(char* model);

    // Set and Get keys
    RAKStatus setDevEUI(char* deveui);
    RAKStatus getDevEUI(char* deveui);
    RAKStatus setAppKey(char* appkey);
    RAKStatus getAppKey(char* appkey);
    RAKStatus setDevAddr(char* devaddr);
    RAKStatus getDevAddr(char* devaddr);
    RAKStatus setAppsKey(char* appskey);
    RAKStatus getAppsKey(char* appskey);
    RAKStatus setNwksKey(char* nwkskey);
    RAKStatus getNwksKey(char* nwkskey);

    // Multicast configuration
    RAKStatus addMultiGroup(char group, char* devAddr, char* NwsKey, char* AppsKey, int frequency, int DR, int periodicity);
    RAKStatus rmvMultiGroup(char *devAddr);
    bool getMulticastConfig(char* response, size_t maxLen);

    // Downlink functions
    void parseDownlink(char* downlink);
    DownlinkMessage getLastDownlink();

    void setDownlinkCallback(void (*callback)(const char* message));
    void (*getDownlinkCallback())(const char*) {
        return _DownlinkCallback;
    }

    void setJoinStatus(JoinStatus status){_joinStatus = status;};
    JoinStatus getJoinStatus(void){return _joinStatus;};
    bool joinFail();

    bool isWaitingAT() const;
    void setWaitingAT(bool v);
    void appendAT(const char* data, size_t len);
    EventGroupHandle_t getCommandsEventGroup();
    EventGroupHandle_t getJoinEventGroup();

private:
    void (*_DownlinkCallback)(const char* message) = nullptr;
    uart_port_t _uart;
    JoinStatus _joinStatus = JoinStatus::IDLE;
    EventGroupHandle_t _commandsEventGroup;
    EventGroupHandle_t _joinEventGroup;
    char _atResponse[256];
    size_t _atLen = 0;
    int _joinAttempts = 0;
    bool _waitingAT = false;
    DownlinkMessage _lastDownlink;
    bool sendATCommand(const char* cmd, char* response, size_t maxLen, uint32_t timeout_ms);

    int extractInt(char* message);
    char extractChar(char* message);
    void extractString(char* message, char* out);
    
};

#endif