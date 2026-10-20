#ifndef M66_F4_H
#define M66_F4_H
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stddef.h>
#ifndef M66_RX_DMA_SIZE
#define M66_RX_DMA_SIZE 2048u
#endif
#ifndef M66_TX_QUEUE_SIZE
#define M66_TX_QUEUE_SIZE 4096u
#endif
#ifndef M66_EVENT_QUEUE_SIZE
#define M66_EVENT_QUEUE_SIZE 24u
#endif
#ifndef M66_LINE_SIZE
#define M66_LINE_SIZE 512u
#endif
#ifndef M66_RAW_DATA_SIZE
#define M66_RAW_DATA_SIZE 1024u
#endif
#ifndef M66_CMD_SIZE
#define M66_CMD_SIZE 256u
#endif
#ifndef M66_TX_CHUNK_SIZE
#define M66_TX_CHUNK_SIZE 256u
#endif
typedef enum {
    M66_OK = 0, M66_E_ARG, M66_E_BUSY, M66_E_TIMEOUT, M66_E_MODEM,
    M66_E_QUEUE_FULL, M66_E_UART, M66_E_PARSE, M66_E_NOT_READY,
    M66_E_UNSUPPORTED, M66_E_OVERFLOW, M66_E_NETWORK, M66_E_STATE
} M66_Status;
typedef enum {
    M66_RX_CIRCULAR = 0, M66_RX_TO_IDLE = 1
} M66_RxMode;
typedef enum {
    M66_EV_READY = 1, M66_EV_RING, M66_EV_CALLER_ID, M66_EV_CALL_INCOMING,
    M66_EV_CALL_CONNECTED, M66_EV_CALL_ENDED, M66_EV_DTMF,
    M66_EV_SMS_NOTIFICATION, M66_EV_SMS_RECEIVED, M66_EV_SMS_SENT,
    M66_EV_REGISTERED, M66_EV_REGISTRATION_LOST, M66_EV_GPRS_UP,
    M66_EV_GPRS_DOWN, M66_EV_IP, M66_EV_SOCKET_CONNECTED, M66_EV_SOCKET_CLOSED,
    M66_EV_SOCKET_DATA, M66_EV_UART_ERROR, M66_EV_RX_OVERFLOW, M66_EV_MODEM_ERROR,
    M66_EV_USSD, M66_EV_HTTP_COMPLETE, M66_EV_MQTT_MESSAGE, M66_EV_RAW_LINE
} M66_EventType;
typedef struct {
    M66_EventType type;
    uint32_t tick;
    int32_t code;
    uint16_t len;
    uint8_t socket_id;
    char number[32];
    char text[M66_LINE_SIZE];
    uint8_t data[M66_RAW_DATA_SIZE];
} M66_Event;
typedef void (*M66_Callback)(const M66_Event *event, void *user);
typedef struct {
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef *hdma_rx;
    /* optional: defaults to huart->hdmarx */
    DMA_HandleTypeDef *hdma_tx;
    /* optional: defaults to huart->hdmatx */
    M66_RxMode rx_mode;
    uint16_t rx_dma_size;
    /* <= M66_RX_DMA_SIZE */
    uint16_t tx_queue_size;
    /* <= M66_TX_QUEUE_SIZE */
    uint32_t default_timeout_ms;
    uint32_t prompt_timeout_ms;
    uint8_t enable_rx_half_callback;
    /* 0 disables HT interrupt; use IDLE/TC */
} M66_Config;
typedef struct {
    uint32_t rx_bytes, tx_bytes, rx_overflow_events, uart_errors, parser_errors;
    uint32_t event_drops, tx_queue_drops, commands_ok, commands_failed;
    uint32_t rx_dma_overrun_suspected;
} M66_Stats;
typedef struct {
    UART_HandleTypeDef *huart;
    M66_Config cfg;
    uint8_t rx_dma[M66_RX_DMA_SIZE];
    volatile uint16_t rx_last_pos;
    volatile uint8_t rx_pending;
    /* ReceiveToIdle single-buffer mode guard */
    uint8_t tx_queue[M66_TX_QUEUE_SIZE];
    volatile uint16_t tx_head, tx_tail;
    volatile uint8_t tx_busy;
    uint8_t tx_active[M66_TX_CHUNK_SIZE];
    uint16_t tx_active_len;
    uint8_t line[M66_LINE_SIZE];
    uint16_t line_len;
    uint8_t line_discard;
    M66_Event events[M66_EVENT_QUEUE_SIZE];
    volatile uint8_t ev_head, ev_tail;
    M66_Callback callback;
    void *callback_user;
    uint8_t initialized, command_active, command_done, command_prompt;
    M66_Status command_result;
    char response[M66_LINE_SIZE];
    uint16_t response_len;
    uint32_t command_deadline;
    uint8_t collect_response;
    uint8_t sms_header_pending;
    char sms_sender[32];
    uint8_t socket_payload_mode;
    uint16_t socket_payload_expected, socket_payload_got;
    uint8_t socket_payload_id;
    /*Length-delimited binary capture for QIRD and QHTTPREAD. */
    volatile uint8_t raw_capture_mode;
    uint16_t raw_capture_expected, raw_capture_got;
    uint32_t raw_capture_total;
    uint8_t raw_tail[6], raw_tail_pos, raw_tail_count;
    uint8_t raw_capture_overflow;
    uint8_t raw_capture[M66_RAW_DATA_SIZE];
    uint16_t raw_len;
    uint8_t http_read_armed;
    uint8_t socket_connected;
    uint8_t socket_open_failed;
    uint8_t http_done;
    int16_t http_status;
    int16_t http_error;
    int32_t http_content_length;
    char ussd_text[M66_LINE_SIZE];
    volatile uint8_t mqtt_done;
    int16_t mqtt_last_result;
    volatile uint8_t ftp_done;
    int32_t ftp_last_result;
    M66_Stats stats;
} M66_Handle;
/*Core / AT transaction */
M66_Status M66_Init(M66_Handle *, const M66_Config *);
void M66_Process(M66_Handle *);
void M66_RegisterCallback(M66_Handle *, M66_Callback, void *);
M66_Status M66_AT(M66_Handle *, const char *, char *, size_t, uint32_t);
void M66_UART_RxEventCallback(M66_Handle *, uint16_t);
void M66_UART_TxCpltCallback(M66_Handle *);
void M66_UART_ErrorCallback(M66_Handle *);
void M66_GetStats(const M66_Handle *, M66_Stats *);
void M66_ClearStats(M66_Handle *);
/*Common modem, SIM, radio and network status */
M66_Status M66_Test(M66_Handle *);
M66_Status M66_Echo(M66_Handle *, uint8_t);
M66_Status M66_GetManufacturer(M66_Handle *, char *, size_t);
M66_Status M66_GetModel(M66_Handle *, char *, size_t);
M66_Status M66_GetFirmware(M66_Handle *, char *, size_t);
M66_Status M66_GetIMEI(M66_Handle *, char *, size_t);
M66_Status M66_GetIMSI(M66_Handle *, char *, size_t);
M66_Status M66_GetICCID(M66_Handle *, char *, size_t);
M66_Status M66_GetSIMReady(M66_Handle *, char *, size_t);
M66_Status M66_GetSignalQuality(M66_Handle *, int *, int *);
M66_Status M66_GetNetworkRegistration(M66_Handle *, char *, size_t);
M66_Status M66_GetOperator(M66_Handle *, char *, size_t);
M66_Status M66_GetClock(M66_Handle *, char *, size_t);
M66_Status M66_SetBaud(M66_Handle *, uint32_t, uint8_t);
M66_Status M66_SaveProfile(M66_Handle *);
M66_Status M66_RestoreProfile(M66_Handle *);
M66_Status M66_FactoryReset(M66_Handle *);
M66_Status M66_SoftReset(M66_Handle *);
M66_Status M66_PowerDown(M66_Handle *);
M66_Status M66_SetPIN(M66_Handle *, const char *);
M66_Status M66_GetBattery(M66_Handle *, char *, size_t);
/*SMS: text mode; sender/body parsing; SMS notification is a separate event */
M66_Status M66_SMSInit(M66_Handle *, uint8_t text_mode);
M66_Status M66_SMSSend(M66_Handle *, const char *, const char *, uint32_t);
M66_Status M66_SMSRead(M66_Handle *, uint32_t, char *, size_t, char *, size_t, uint32_t);
M66_Status M66_SMSDelete(M66_Handle *, uint32_t);
M66_Status M66_SMSList(M66_Handle *, char *, size_t);
M66_Status M66_SMSSetStorage(M66_Handle *, const char *, const char *, const char *);
/*Voice call / DTMF */
M66_Status M66_Call(M66_Handle *, const char *);
M66_Status M66_Answer(M66_Handle *);
M66_Status M66_Hangup(M66_Handle *);
M66_Status M66_DTMFEnable(M66_Handle *, uint8_t);
M66_Status M66_DTMFSend(M66_Handle *, char, uint16_t);
/*GPRS and sockets. Host/port/socket IDs must be validated for product use. */
M66_Status M66_GPRSSetAPN(M66_Handle *, const char *, const char *, const char *);
M66_Status M66_GPRSActivate(M66_Handle *, uint32_t);
M66_Status M66_GPRSDeactivate(M66_Handle *);
M66_Status M66_GPRSGetIP(M66_Handle *, char *, size_t);
M66_Status M66_GPRSSetDNS(M66_Handle *, const char *, const char *);
M66_Status M66_SocketOpen(M66_Handle *, uint8_t, const char *, const char *, uint16_t, uint32_t);
M66_Status M66_SocketSend(M66_Handle *, uint8_t, const uint8_t *, uint16_t, uint32_t);
M66_Status M66_SocketRead(M66_Handle *, uint8_t, uint16_t, uint32_t);
/*Binary-safe receive: actual bytes copied to out; returned in *received. */
M66_Status M66_SocketReadData(M66_Handle *, uint8_t, uint16_t, uint8_t *, size_t, size_t *, uint32_t);
M66_Status M66_SocketClose(M66_Handle *, uint8_t);
/*Extra useful AT wrappers */
M66_Status M66_GetTemperature(M66_Handle *, char *, size_t);
M66_Status M66_SetFunctionality(M66_Handle *, uint8_t);
M66_Status M66_GetErrorReportMode(M66_Handle *, uint8_t *);
M66_Status M66_SetErrorReportMode(M66_Handle *, uint8_t);
M66_Status M66_SetCLIP(M66_Handle *, uint8_t);
M66_Status M66_SetCallerIDPresentation(M66_Handle *, uint8_t);
M66_Status M66_SetSMSDeliveryReport(M66_Handle *, uint8_t);
/*USSD: response is also emitted as M66_EV_USSD; network/operator syntax varies. */
M66_Status M66_USSD(M66_Handle *, const char *, uint32_t);
M66_Status M66_USSDCancel(M66_Handle *);
/*HTTP client using the M66 Quectel HTTP AT command set. Body is length-delimited.
   HTTP(S) support depends on firmware and server TLS requirements. */
M66_Status M66_HTTPSetURL(M66_Handle *, const char *, uint32_t);
M66_Status M66_HTTPSetParameter(M66_Handle *, const char *, const char *);
M66_Status M66_HTTPSetSSL(M66_Handle *, uint8_t);
M66_Status M66_HTTPGet(M66_Handle *, uint32_t, int *, int32_t *);
M66_Status M66_HTTPPost(M66_Handle *, const char *, const uint8_t *, size_t, uint32_t, int *, int32_t *);
M66_Status M66_HTTPRead(M66_Handle *, uint8_t *, size_t, size_t *, uint32_t);
M66_Status M66_HTTPStop(M66_Handle *);
M66_Status M66_HTTPDownloadToModemFS(M66_Handle *, const char *, uint32_t, char *, size_t, uint32_t);
/*Optional FTP command wrappers (feature support depends on installed M66 firmware). */
M66_Status M66_FTPSetUser(M66_Handle *, const char *);
M66_Status M66_FTPSetPassword(M66_Handle *, const char *);
M66_Status M66_FTPSetPath(M66_Handle *, const char *);
M66_Status M66_FTPSetConfig(M66_Handle *, uint8_t, const char *);
M66_Status M66_FTPOpen(M66_Handle *, const char *, uint16_t, uint32_t);
M66_Status M66_FTPGet(M66_Handle *, const char *, uint32_t, uint32_t);
M66_Status M66_FTPPut(M66_Handle *, const char *, uint32_t, uint32_t);
M66_Status M66_FTPClose(M66_Handle *);
M66_Status M66_FTPStatus(M66_Handle *, char *, size_t);
M66_Status M66_FTPLength(M66_Handle *, char *, size_t);
/*MQTT built into supported M66 firmware. Receive messages arrive as M66_EV_MQTT_MESSAGE. */
M66_Status M66_MQTTOpen(M66_Handle *, uint8_t, const char *, uint16_t, uint32_t);
M66_Status M66_MQTTConnect(M66_Handle *, uint8_t, const char *, const char *, const char *, uint32_t);
M66_Status M66_MQTTSubscribe(M66_Handle *, uint8_t, uint16_t, const char *, uint8_t, uint32_t);
M66_Status M66_MQTTUnsubscribe(M66_Handle *, uint8_t, uint16_t, const char *, uint32_t);
M66_Status M66_MQTTPublish(M66_Handle *, uint8_t, uint16_t, uint8_t, uint8_t, const char *, const uint8_t *, size_t, uint32_t);
M66_Status M66_MQTTDisconnect(M66_Handle *, uint8_t, uint32_t);
M66_Status M66_MQTTClose(M66_Handle *, uint8_t, uint32_t);
M66_Status M66_MQTTSetKeepAlive(M66_Handle *, uint8_t, uint16_t);
M66_Status M66_MQTTSetSSL(M66_Handle *, uint8_t, uint8_t, uint8_t);
/*GPRS/network convenience and diagnostics */
M66_Status M66_GPRSInit(M66_Handle *, const char *, const char *, const char *, uint32_t);
M66_Status M66_GetGPRSRegistration(M66_Handle *, char *, size_t);
M66_Status M66_GetNetworkMode(M66_Handle *, char *, size_t);
M66_Status M66_GetIMEIAlt(M66_Handle *, char *, size_t);
#ifdef __cplusplus
}
#endif
#endif
