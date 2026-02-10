#include "RAK3172.h"

static QueueHandle_t uart_queue;
static uart_event_t event;

RAK3172::RAK3172()
{}

RAK3172::~RAK3172()
{}

/**
 * @brief  UART task to handle received messages.
 * @param  none
 * @retval none
 */
static void uart_task(void *param) {
    RAK3172* rak = (RAK3172*)param;
    uart_event_t event;
    char data[256];

    while (true) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                int len = uart_read_bytes(rak->getUart(), (uint8_t*)data, event.size, pdMS_TO_TICKS(100));
                data[len] = '\0';

                /* ---------- AT RESPONSE ---------- */
                if (rak->isWaitingAT()) {
                    rak->appendAT(data, len);

                    if (strchr(data, '\n')) {
                        xEventGroupSetBits(
                            rak->getCommandsEventGroup(),
                            EVT_AT_RESPONSE
                        );
                    }
                }

                 /* -------- JOIN DETECTION -------- */

                else if(rak->getJoinStatus() == JoinStatus::JOINING){
                    if (strstr(data, "JOINED") != nullptr ||
                        strstr(data, "+EVT:JOINED") != nullptr)
                    {
                        xEventGroupSetBits(
                            rak->getJoinEventGroup(),
                            0x1
                        );
                    }

                    if (strstr(data, "JOIN FAILED") != nullptr ||
                        strstr(data, "+EVT:JOIN FAILED") != nullptr)
                    {
                        if(!rak->joinFail()){
                            xEventGroupSetBits(
                                rak->getJoinEventGroup(),
                                0x2
                            );
                        }
                    }
                }

                /* -------- DOWNLINK -------- */

                else if (strstr(data, "RX") != nullptr ||
                        strstr(data, "+EVT:RX") != nullptr)
                {
                    rak->parseDownlink(data);
                    if (rak->getDownlinkCallback()) {
                        rak->getDownlinkCallback()(data);
                    }
                }
            }
        }
    }
}

/**
 * @brief  Getter of uart port.
 * @param  none
 * @retval Uart port used in RAK communication
 */
uart_port_t RAK3172::getUart()
{
    return _uart;
}

/**
 * @brief  Setter to Downlink callback function pointer.
 * @param  none
 * @retval none
 */
void RAK3172::setDownlinkCallback(void (*callback)(const char* message)) {
    _DownlinkCallback = callback;
}

/**
 * @brief  Sends AT commands to RAK.
 * @param[in]  cmd AT command that will be sent.
 * @param[out] response Pointer to the string that will receive the RAK response.
 * @param[in] maxLen Size of response string.
 * @param[in] timeout_ms Timeout of the RAK response message in milliseconds.
 * 
 * @retval 1  Command sent and response received successfully.   
 * @retval 0  Timeout occurred while waiting for response.
 */
bool RAK3172::sendATCommand(const char* cmd, char* response, size_t maxLen, uint32_t timeout_ms)
{
    if (!cmd || !response || maxLen == 0) return false;

    _atLen = 0;
    memset(_atResponse, 0, sizeof(_atResponse));

    setWaitingAT(true);

    uart_write_bytes(_uart, cmd, strlen(cmd));

    EventBits_t bits = xEventGroupWaitBits(
        _commandsEventGroup,
        EVT_AT_RESPONSE,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(timeout_ms)
    );

    setWaitingAT(false);

    if (!(bits & EVT_AT_RESPONSE)) {
        return false; // TIMEOUT REAL
    }

    strncpy(response, _atResponse, maxLen - 1);
    response[maxLen - 1] = '\0';

    return true;
}

/**
 * @brief  Initialize RAK communication.
 * @param[in]  uart Uart port that will be used to communicate with RAK.
 * @param[in] tx GPIO port that will be used as TX.
 * @param[in] rx GPIO port that will be used as TX.
 * 
 * @retval 1  Communication initialized correctly.   
 * @retval 0  Error.
 */
int RAK3172::init(uart_port_t uart, uart_port_t tx, uart_port_t rx)
{
    _uart = uart;

    // Deleta driver UART antigo se existir
    uart_driver_delete(uart);

    // Configuração da UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    if (uart_param_config(uart, &uart_config) != ESP_OK) return 0;
    if (uart_set_pin(uart, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return 0;

    // Instala driver UART com buffer e queue
    if (uart_driver_install(uart, 512, 512, 20, &uart_queue, 0) != ESP_OK) return 0;

    // Cria task interna para processar UART
    xTaskCreate(
        uart_task,
        "rak_uart_task",
        4096,
        this,
        10,
        NULL
    );

    _commandsEventGroup = xEventGroupCreate();
    _joinEventGroup = xEventGroupCreate();

    return 1;
}

/**
 * @brief Sets the LoRaWAN region.
 *
 * @param[in] region LoRaWAN region to be set. Possible values:
 *                   - EU433    (0)  : Europe 433 MHz
 *                   - CN470    (1)  : China 470 MHz
 *                   - RU864    (2)  : Russia 864 MHz
 *                   - IN865    (3)  : India 865 MHz
 *                   - EU868    (4)  : Europe 868 MHz
 *                   - US915    (5)  : United States 915 MHz
 *                   - AU915    (6)  : Australia 915 MHz
 *                   - KR920    (7)  : Korea 920 MHz
 *                   - AS923_1  (8)  : Asia 923 MHz – Channel Plan 1
 *                   - AS923_2  (9)  : Asia 923 MHz – Channel Plan 2
 *                   - AS923_3  (10) : Asia 923 MHz – Channel Plan 3
 *                   - AS923_4  (11) : Asia 923 MHz – Channel Plan 4
 *                   - LA915    (12) : Latin America 915 MHz
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setRegion(LoraRegion_t region)
{
    char message[16];
    char response[32]; 

    snprintf(message, sizeof(message), "AT+BAND=%d\r\n", region);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets the LoRaWAN region.
 *
 * @retval LoRaWAN region. Possible values:
 *         - EU433    (0)  : Europe 433 MHz
 *         - CN470    (1)  : China 470 MHz
 *         - RU864    (2)  : Russia 864 MHz
 *         - IN865    (3)  : India 865 MHz
 *         - EU868    (4)  : Europe 868 MHz
 *         - US915    (5)  : United States 915 MHz
 *         - AU915    (6)  : Australia 915 MHz
 *         - KR920    (7)  : Korea 920 MHz
 *         - AS923_1  (8)  : Asia 923 MHz – Channel Plan 1
 *         - AS923_2  (9)  : Asia 923 MHz – Channel Plan 2
 *         - AS923_3  (10) : Asia 923 MHz – Channel Plan 3
 *         - AS923_4  (11) : Asia 923 MHz – Channel Plan 4
 *         - LA915    (12) : Latin America 915 MHz
 */
LoraRegion_t RAK3172::getRegion()
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+BAND=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return LoraRegion_t::UNKNOWN; // ou algum valor padrão de erro
    }

    int region = extractInt(response); // função de parsing segura
    return (LoraRegion_t)region;
}

/**
 * @brief Sets the LoRaWAN class.
 *
 * @param[in] lora_class LoRaWAN class to be set. Possible values:
 *                   - CLASS_A    (0)
 *                   - CLASS_B    (1)
 *                   - CLASS_C    (2)
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setClass(LoraClass_t lora_class)
{
    char _lora_class;
    switch (lora_class)
    {
    case LoraClass_t::CLASS_A:
        _lora_class = 'A';
        break;

    case LoraClass_t::CLASS_B:
        _lora_class = 'B';
        break;

    case LoraClass_t::CLASS_C:
        _lora_class = 'C';
        break;
    
    default:
        break;
    }   

    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+CLASS=%c\r\n", _lora_class);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets LoRaWAN class.
 *
 * @retval  LoRaWAN class. Possible values:
 *          - CLASS_A    (0)
 *          - CLASS_B    (1)
 *          - CLASS_C    (2)
 */
LoraClass_t RAK3172::getClass()
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+CLASS=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return LoraClass_t::UNKNOWN; // ou algum valor padrão de erro
    }

    char lora_class = extractChar(response);
    switch (lora_class)
    {
    case 'A':
        return LoraClass_t::CLASS_A;
        break;

    case 'B':
        return LoraClass_t::CLASS_B;
        break;

    case 'C':
        return LoraClass_t::CLASS_C;
        break;
    
    default:
        return LoraClass_t::UNKNOWN;
        break;
    }
}

/**
 * @brief Sets the LoRaWAN mode.
 *
 * @param[in] mode LoRaWAN mode to be set. Possible values:
 *             - ABP    (0)
 *             - OTA    (1)
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setMode(LoraMode_t mode)
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+NJM=%d\r\n", mode);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets the LoRaWAN mode.
 *
 * @retval LoRaWAN mode. Possible values:
 *         - ABP    (0)
 *         - OTA    (1)
 */
LoraMode_t RAK3172::getMode()
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+NJM=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return LoraMode_t::UNKNOWN; // ou algum valor padrão de erro
    }

    return (LoraMode_t)extractInt(response);

}

/**
 * @brief Sets the LoRaWAN sub-band.
 *
 * @param[in] subBand LoRaWAN sub-band to be set.
 *                   Valid values:
 *                   - 0 : Enable all 64 uplink channels (channels 0–63)  
 *                         and all 8 downlink channels (channels 0–7)
 *                   - 1 : Uplink channels 0–7     + Downlink channel 0
 *                   - 2 : Uplink channels 8–15    + Downlink channel 1
 *                   - 3 : Uplink channels 16–23   + Downlink channel 2
 *                   - 4 : Uplink channels 24–31   + Downlink channel 3
 *                   - 5 : Uplink channels 32–39   + Downlink channel 4
 *                   - 6 : Uplink channels 40–47   + Downlink channel 5
 *                   - 7 : Uplink channels 48–55   + Downlink channel 6
 *                   - 8 : Uplink channels 56–63   + Downlink channel 7
 *
 * @note Applicable only for US915 and AU915 regions.
 *       For other regions, this parameter is ignored.
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setSubBand(int band)
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+CHE=%d\r\n", band);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets the LoRaWAN sub-band.
 *
 * @retval LoRaWAN sub-band.
 *                   Valid values:
 *                   - 0 : Enable all 64 uplink channels (channels 0–63)  
 *                         and all 8 downlink channels (channels 0–7)
 *                   - 1 : Uplink channels 0–7     + Downlink channel 0
 *                   - 2 : Uplink channels 8–15    + Downlink channel 1
 *                   - 3 : Uplink channels 16–23   + Downlink channel 2
 *                   - 4 : Uplink channels 24–31   + Downlink channel 3
 *                   - 5 : Uplink channels 32–39   + Downlink channel 4
 *                   - 6 : Uplink channels 40–47   + Downlink channel 5
 *                   - 7 : Uplink channels 48–55   + Downlink channel 6
 *                   - 8 : Uplink channels 56–63   + Downlink channel 7
 */
int RAK3172::getSubBand()
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+CHE=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return -1; // ou algum valor padrão de erro
    }

    return extractInt(response);
}

/**
 * @brief Sets the Adaptive Data Rate (ADR) status. 
 * @param[in] adr Adaptive Data Rate status. Valid values:
 *        - 0 ADR disabled.
 *        - 1 ADR enabled.
 * 
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setADR(int adr)
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+ADR=%d\r\n", adr);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets the Adaptive Data Rate (ADR) status.
 *
 * @retval 0 ADR disabled.
 * @retval 1 ADR enabled.
 */
int RAK3172::getADR()
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+ADR=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return -1; // ou algum valor padrão de erro
    }

    return extractInt(response);
}

/**
 * @brief Sets the LoRaWAN data rate.
 *
 * @param[in] dr Data rate index (region dependent).
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setDR(int dr)
{
    char message[16];
    char response[32]; 

    snprintf(message, sizeof(message), "AT+DR=%d\r\n", dr);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets the current LoRaWAN data rate.
 *
 * @return Current data rate index.
 */
int RAK3172::getDR()
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+DR=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return -1; // ou algum valor padrão de erro
    }

    return extractInt(response);
}

/**
 * @brief Enables or disables confirmed uplink messages.
 *
 * @param[in] ack ACK mode:
 *                - 0 : Unconfirmed messages
 *                - 1 : Confirmed messages
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setAck(int ack)
{
    char message[16];
    char response[32]; 

    snprintf(message, sizeof(message), "AT+CFM=%d\r\n", ack);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets the ACK mode.
 *
 * @retval 0 Unconfirmed messages.
 * @retval 1 Confirmed messages.
 */
int RAK3172::getAck()
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+CFM=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return -1; // ou algum valor padrão de erro
    }

    return extractInt(response);
}

/**
 * @brief Sets the Join Accept Delay 1.
 *
 * @param[in] delay_sec Delay in seconds.
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setJoin1Delay(int delay_sec)
{
    char message[16];
    char response[32]; 

    snprintf(message, sizeof(message), "AT+JN1DL=%d\r\n", delay_sec);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}


/**
 * @brief Gets the Join Accept Delay 1.
 *
 * @return Delay in seconds.
 */
int RAK3172::getJoin1Delay()
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+JN1DL=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return -1; // ou algum valor padrão de erro
    }

    return extractInt(response);
}

/**
 * @brief Sets the Join Accept Delay 2.
 *
 * @param[in] delay_sec Delay in seconds.
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setJoin2Delay(int delay_sec)
{
    char message[16];
    char response[32]; 

    snprintf(message, sizeof(message), "AT+JN2DL=%d\r\n", delay_sec);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets the Join Accept Delay 2.
 *
 * @return Delay in seconds.
 */
int RAK3172::getJoin2Delay()
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+JN2DL=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return -1; // ou algum valor padrão de erro
    }

    return extractInt(response);
}

/**
 * @brief Sets RX1 delay.
 *
 * @param[in] delay_sec Delay in seconds.
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setRx1Delay(int delay_sec)
{
    char message[16];
    char response[32]; 

    snprintf(message, sizeof(message), "AT+RX1DL=%d\r\n", delay_sec);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets RX1 delay.
 *
 * @return Delay in seconds.
 */
int RAK3172::getRx1Delay()
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+RX1DL=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return -1; // ou algum valor padrão de erro
    }

    return extractInt(response);
}

/**
 * @brief Sets RX2 delay.
 *
 * @param[in] delay_sec Delay in seconds.
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setRx2Delay(int delay_sec)
{
    char message[16];
    char response[32]; 

    snprintf(message, sizeof(message), "AT+RX2DL=%d\r\n", delay_sec);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets RX2 delay.
 *
 * @return Delay in seconds.
 */
int RAK3172::getRx2Delay()
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+RX2DL=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return -1; // ou algum valor padrão de erro
    }

    return extractInt(response);
}

/**
 * @brief Sets the transmission power.
 *
 * @param[in] power Transmission power index (region dependent).
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setTransmitPower(int power)
{
    char message[16];
    char response[32]; 

    snprintf(message, sizeof(message), "AT+TXP=%d\r\n", power);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets the current transmission power.
 *
 * @return Transmission power index.
 */
int RAK3172::getTransmitPower()
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+TXP=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return -1; // ou algum valor padrão de erro
    }

    return extractInt(response);
}

/**
 * @brief Gets the last received RSSI value.
 *
 * @return RSSI value in dBm.
 */
int RAK3172::getRSSI()
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+RSSI=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return -1; // ou algum valor padrão de erro
    }

    return extractInt(response);
}

/**
 * @brief Gets the last received SNR value.
 *
 * @return SNR value in dB.
 */
int RAK3172::getSNR()
{
    char message[16];
    char response[32];

    snprintf(message, sizeof(message), "AT+SNR=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return -1; // ou algum valor padrão de erro
    }

    return extractInt(response);
}

/**
 * @brief Joins the LoRaWAN network.
 *
 * @param[in] autoJoin    Enable automatic join retry:
 *                        - 0 : Disabled
 *                        - 1 : Enabled
 * @param[in] interval    Interval between join attempts (seconds).
 * @param[in] numAttempts Number of join attempts (0 = infinite).
 *
 * @retval OK       Join procedure started successfully.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::join(int autoJoin, int interval, int numAttempts)
{
    char message[32];
    char response[32]; 

    _joinAttempts = numAttempts;
    snprintf(message, sizeof(message), "AT+JOIN=1:%d:%d:%d\r", autoJoin, interval, numAttempts);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        _joinStatus = JoinStatus::JOINING;

        EventBits_t bits = xEventGroupWaitBits(
        _joinEventGroup,
        EVT_JOIN_RESPONSE,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(300000) // 5 MIN
        );

        if(bits == 0x1){ // JOINED
            _joinStatus = JoinStatus::JOINED;
        }
        else if(bits == 0x2){ //JOIN FAILED
            _joinStatus = JoinStatus::JOIN_FAILED;
        }
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Sends an uplink data packet.
 *
 * @param[in] port LoRaWAN application port (1–223).
 * @param[in] data Payload to be sent (hex string).
 *
 * @retval OK       Data sent successfully.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::send(int port, char* data)
{
    char message[514];
    char response[32];

    if(_joinStatus != JoinStatus::JOINED){
        return RAKStatus::ERROR;
    }

    snprintf(message, sizeof(message), "AT+SEND=%d:%s\n\r", port, data);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

//############## Keys Configurations ###################

/**
 * @brief Sets the Device EUI.
 *
 * @param[in] deveui Device EUI (16 hex characters).
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setDevEUI(char* deveui)
{
    char message[64];
    char response[32]; 

    snprintf(message, sizeof(message), "AT+DEVEUI=%s\r\n", deveui);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets the Device EUI.
 *
 * @param[out] deveui Buffer to store the Device EUI.
 */
RAKStatus RAK3172::getDevEUI(char* deveui)
{
    char message[32];
    char response[64];

    snprintf(message, sizeof(message), "AT+DEVEUI=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    extractString(response, deveui);
    return RAKStatus::OK;
}

/**
 * @brief Sets the Application Key.
 *
 * @param[in] appkey Application key (32 hex characters).
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setAppKey(char* appkey)
{
    char message[64];
    char response[32]; 

    snprintf(message, sizeof(message), "AT+APPKEY=%s\r\n", appkey);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets the Application Key.
 *
 * @param[out] appkey Buffer to store the Application Key.
 */
RAKStatus RAK3172::getAppKey(char* appkey)
{
    char message[32];
    char response[64];

    snprintf(message, sizeof(message), "AT+APPKEY=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    extractString(response, appkey);
    return RAKStatus::OK;
}

/**
 * @brief Sets the Device Address.
 *
 * @param[in] devaddr Device address (8 hex characters).
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setDevAddr(char* devaddr)
{
    char message[64];
    char response[32]; 

    snprintf(message, sizeof(message), "AT+DEVADDR=%s\r\n", devaddr);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets the Device Address.
 *
 * @param[out] devaddr Buffer to store the Device Address.
 */
RAKStatus RAK3172::getDevAddr(char* devaddr)
{
    char message[32];
    char response[64];

    snprintf(message, sizeof(message), "AT+DEVADDR=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    extractString(response, devaddr);
    return RAKStatus::OK;
}

/**
 * @brief Sets the Application Session Key.
 *
 * @param[in] appskey Application session key (32 hex characters).
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setAppsKey(char* appskey)
{
    char message[64];
    char response[32]; 

    snprintf(message, sizeof(message), "AT+APPSKEY=%s\r\n", appskey);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets the Application Session Key.
 *
 * @param[out] appskey Buffer to store the Application Session Key.
 */
RAKStatus RAK3172::getAppsKey(char* appskey)
{
    char message[32];
    char response[64];

    snprintf(message, sizeof(message), "AT+APPSKEY=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    extractString(response, appskey);
    return RAKStatus::OK;
}

/**
 * @brief Sets the Network Session Key.
 *
 * @param[in] nwkskey Network session key (32 hex characters).
 *
 * @retval OK       Command received correctly.
 * @retval ERROR    Command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::setNwksKey(char* nwkskey)
{
    char message[64];
    char response[32]; 

    snprintf(message, sizeof(message), "AT+NWKSKEY=%s\r\n", nwkskey);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Gets the Network Session Key.
 *
 * @param[out] nwkskey Buffer to store the Network Session Key.
 */
RAKStatus RAK3172::getNwksKey(char* nwkskey)
{
    char message[32];
    char response[64];

    snprintf(message, sizeof(message), "AT+NWKSKEY=?\r\n");

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    extractString(response, nwkskey);
    return RAKStatus::OK;
}

//############## Multicast Configuration ###################

/**
 * @brief Adds a device to a multi-device group for LoRaWAN.
 *
 * @param[in] group      Group identifier (single character).
 * @param[in] devAddr    Device address (8 hex characters).
 * @param[in] NwsKey     Network session key (32 hex characters).
 * @param[in] AppsKey    Application session key (32 hex characters).
 * @param[in] frequency  Transmission frequency in Hz.
 * @param[in] DR         Data rate index.
 * @param[in] periodicity Transmission interval in seconds.
 *
 * @retval OK       Device added successfully to the group.
 * @retval ERROR    Command or parameter error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::addMultiGroup(char group, char* devAddr, char* NwsKey, char* AppsKey, int frequency, int DR, int periodicity)
{
    char message[256];
    char response[32]; 

    snprintf(message, sizeof(message),  "AT+ADDMULC=%c:%s:%s:%s:%d:%d:%d\r", group, devAddr, NwsKey, AppsKey, frequency, DR, periodicity);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

/**
 * @brief Removes a device from a multi-device group.
 *
 * @param[in] devAddr Device address of the device to remove (8 hex characters).
 *
 * @retval OK       Device removed successfully.
 * @retval ERROR    Device not found or command error.
 * @retval TIMEOUT  Timeout in RAK communication.
 */
RAKStatus RAK3172::rmvMultiGroup(char *devAddr)
{
    char message[64];
    char response[32]; 

    snprintf(message, sizeof(message),  "AT+RMVMULC=%s\r", devAddr);

    if (!sendATCommand(message, response, sizeof(response), 2000)) {
        return RAKStatus::TIMEOUT;
    }

    if (strstr(response, "OK") != nullptr) {
        return RAKStatus::OK;
    } else {
        return RAKStatus::ERROR;
    }
}

//############## Downlink functions ###################

/**
 * @brief  Parses the Downlink message in a DownlinkMessage struct to organize the information.
 * @param[in]  downlink Downlink message received.
 */
void RAK3172::parseDownlink(char* downlink)
{
    sscanf(
    downlink,
    "+EVT:RX_%*c:%d:%d:%11[^:]:%d:%127s",
    &_lastDownlink.rssi,
    &_lastDownlink.snr,
    _lastDownlink.type,
    &_lastDownlink.fport,
    _lastDownlink.data
    );
}

/**
 * @brief  Gets the last downlink message.
 * 
 * @retval last downlink message in a DownlinkMessage struct form.
 */
DownlinkMessage RAK3172::getLastDownlink()
{
    return _lastDownlink;
}

//############## Auxiliary Functions ###################

int RAK3172::extractInt(char* message)
{
    const char *p = strchr(message, '=');   // procura '='
    if (!p) return -1;                  // formato inválido

    return (int)strtol(p + 1, NULL, 10);  // converte o número
}

char RAK3172::extractChar(char* message)
{
    const char *p = strchr(message, '=');   // procura '='
    if (!p) return -1;                  // formato inválido

    return *(p+1);  // converte o número
}

void RAK3172::extractString(char* message, char* out)
{
    const char *p = strchr(message, '=');
    if (!p) {
        out[0] = '\0';
        return;
    }

    strcpy(out, p + 1);  // copia tudo depois do '='
}

bool RAK3172::isWaitingAT() const {
    return _waitingAT;
}

void RAK3172::setWaitingAT(bool v) {
    _waitingAT = v;
}

void RAK3172::appendAT(const char* data, size_t len) {
    size_t space = sizeof(_atResponse) - _atLen - 1;
    size_t toCopy = len < space ? len : space;
    memcpy(&_atResponse[_atLen], data, toCopy);
    _atLen += toCopy;
    _atResponse[_atLen] = '\0';
}

EventGroupHandle_t RAK3172::getCommandsEventGroup() {
    return _commandsEventGroup;
}

EventGroupHandle_t RAK3172::getJoinEventGroup() {
    return _joinEventGroup;
}

bool RAK3172::joinFail()
{
    return _joinAttempts-- > 0;
}