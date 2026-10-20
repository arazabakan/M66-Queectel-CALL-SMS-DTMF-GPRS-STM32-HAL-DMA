#include "m66_f4.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/*
 *STM32F4 HAL implementation foundation.
 *This file intentionally makes prompt/raw-data operations explicit and checks
 * command result. Review AT syntax against the exact M66 AT manual/firmware.
 */
static uint32_t tick(M66_Handle *m) {
    (void) m;
    return HAL_GetTick();
}
static void copystr(char *d, size_t cap, const char *s) {
    if (!d || !cap) return;
    if (!s) s = "";
    size_t n = strlen(s);
    if (n >= cap) n = cap - 1;
    memcpy(d, s, n);
    d[n] = 0;
}
static int prefix(const char *s, const char *p) {
    return strncmp(s, p, strlen(p)) == 0;
}
static uint16_t qnext(uint16_t p, uint16_t cap) {
    return (uint16_t)((p + 1u) % cap);
}
static void push(M66_Handle *m, M66_EventType t, const char *txt, const char *num, int32_t code) {
    uint8_t n = (uint8_t)((m->ev_head + 1u) % M66_EVENT_QUEUE_SIZE);
    if (n == m->ev_tail) {
        m->stats.event_drops++;
        return;
    }
    M66_Event *e = & m->events[m->ev_head];
    memset(e, 0, sizeof(*e));
    e->type = t;
    e->tick = tick(m);
    e->code = code;
    copystr(e->text, sizeof(e->text), txt);
    copystr(e->number, sizeof(e->number), num);
    e->len = (uint16_t) strlen(e->text);
    m->ev_head = n;
}
static M66_Status tx_enqueue(M66_Handle *m, const uint8_t *p, uint16_t len) {
    uint16_t cap = m->cfg.tx_queue_size;
    if (!p || !len || len >= cap) return M66_E_ARG;
    uint16_t used = (uint16_t)((m->tx_head + cap - m->tx_tail) % cap);
    if ((uint32_t) used + len >= (uint32_t) cap) {
        m->stats.tx_queue_drops++;
        return M66_E_QUEUE_FULL;
    }
    for (uint16_t i = 0; i < len; i++) {
        m->tx_queue[m->tx_head] = p[i];
        m->tx_head = qnext(m->tx_head, cap);
    }
    return M66_OK;
}
static void tx_kick(M66_Handle *m) {
    if (m->tx_busy || m->tx_tail == m->tx_head) return;
    uint16_t cap = m->cfg.tx_queue_size, n = 0;
    while (m->tx_tail != m->tx_head && n < M66_TX_CHUNK_SIZE) {
        m->tx_active[n++] = m->tx_queue[m->tx_tail];
        m->tx_tail = qnext(m->tx_tail, cap);
    }
    if (n) {
        m->tx_active_len = n;
        m->tx_busy = 1;
        if (HAL_UART_Transmit_DMA(m->huart, m->tx_active, n) != HAL_OK) {
            m->tx_busy = 0;
            m->stats.uart_errors++;
            push(m, M66_EV_UART_ERROR, "HAL_UART_Transmit_DMA failed", NULL, HAL_UART_GetError(m->huart));
        }
    }
}
static void parse_line(M66_Handle *m, const char *s) {
    if (!*s) return;
    if (strcmp(s, "OK") == 0) {
        if (m->command_active) {
            m->command_result = M66_OK;
            m->command_done = 1;
        }
        return;
    }
    if (strcmp(s, "SEND OK") == 0 || strcmp(s, "SEND ACCEPT") == 0 || strcmp(s, "DEACT OK") == 0 || strcmp(s, "CLOSE OK") == 0 || strcmp(s, "POWERED DOWN") == 0 || strcmp(s, "NORMAL POWER DOWN") == 0) {
        if (m->command_active) {
            m->command_result = M66_OK;
            m->command_done = 1;
        }
        return;
    }
    if (strcmp(s, "SEND FAIL") == 0 || strcmp(s, "CONNECT FAIL") == 0) {
        m->socket_open_failed = 1;
        if (m->command_active) {
            m->command_result = M66_E_NETWORK;
            m->command_done = 1;
        }
        push(m, M66_EV_MODEM_ERROR, s, NULL, - 2);
        return;
    }
    if (strcmp(s, "CONNECT OK") == 0 || strcmp(s, "ALREADY CONNECT") == 0) {
        m->socket_connected = 1;
        if (m->command_active && strcmp(s, "ALREADY CONNECT") == 0) {
            m->command_result = M66_OK;
            m->command_done = 1;
        }
        push(m, M66_EV_SOCKET_CONNECTED, s, NULL, 0);
        return;
    }
    if (strcmp(s, "CLOSED") == 0) {
        push(m, M66_EV_SOCKET_CLOSED, s, NULL, 0);
        return;
    }
    if (strcmp(s, "NO CARRIER") == 0 || strcmp(s, "BUSY") == 0 || strcmp(s, "NO ANSWER") == 0) {
        push(m, M66_EV_CALL_ENDED, s, NULL, 0);
        return;
    }
    if (strcmp(s, "CONNECT") == 0) {
        m->command_prompt = 1;
        if (m->http_read_armed) {
            m->raw_capture_expected = (uint16_t)((m->http_content_length < 0)?0: m->http_content_length);
            m->raw_capture_got = 0;
            m->raw_capture_total = 0;
            m->raw_tail_pos = 0;
            m->raw_tail_count = 0;
            memset(m->raw_tail, 0, sizeof(m->raw_tail));
            m->raw_capture_overflow = 0;
            m->raw_capture_mode = (m->http_content_length < 0)?2u: ((m->http_content_length > 0)?1u: 0u);
            m->http_read_armed = 0;
        }
        return;
    }
    if (strcmp(s, "ERROR") == 0 || prefix(s, "+CME ERROR") || prefix(s, "+CMS ERROR")) {
        if (m->command_active) {
            m->command_result = M66_E_MODEM;
            m->command_done = 1;
        }
        push(m, M66_EV_MODEM_ERROR, s, NULL, - 1);
        return;
    }
    if (strcmp(s, ">") == 0 || strcmp(s, "> ") == 0) {
        m->command_prompt = 1;
        return;
    }
    if (strcmp(s, "RING") == 0) {
        push(m, M66_EV_RING, s, NULL, 0);
        return;
    }
    if (prefix(s, "+CLIP:")) {
        const char *q = strchr(s, '"');
        if (q) {
            const char *e = strchr(q + 1, '"');
            if (e) {
                char num[32] = {
                    0
                };
                size_t n = (size_t)(e - q - 1);
                if (n > 31) n = 31;
                memcpy(num, q + 1, n);
                push(m, M66_EV_CALLER_ID, s, num, 0);
                push(m, M66_EV_CALL_INCOMING, s, num, 0);
            }
        }
        return;
    }
    if (prefix(s, "+DTMF:")) {
        const char *p = strchr(s, ':');
        if (p) {
            while (*++p == ' ');
            char d[2] = {
                *p, 0
            };
            push(m, M66_EV_DTMF, s, d, 0);
        }
        return;
    }
    if (prefix(s, "+CMTI:")) {
        const char *p = strrchr(s, ',');
        push(m, M66_EV_SMS_NOTIFICATION, s, NULL, p?atoi(p + 1): - 1);
        return;
    }
    if (prefix(s, "+CUSD:")) {
        copystr(m->ussd_text, sizeof(m->ussd_text), s);
        push(m, M66_EV_USSD, s, NULL, 0);
        return;
    }
    if (prefix(s, "+QMTRECV:")) {
        push(m, M66_EV_MQTT_MESSAGE, s, NULL, 0);
        return;
    }
    if (prefix(s, "+QMTOPEN:") || prefix(s, "+QMTCONN:") || prefix(s, "+QMTSUB:") || prefix(s, "+QMTUNS:") || prefix(s, "+QMTPUB:") || prefix(s, "+QMTDISC:") || prefix(s, "+QMTCLOSE:")) {
        const char *p = strchr(s, ':');
        int id = - 1, msg = 0, res = - 99, ret = 0;
        if (p) {
            p++;
            if (prefix(s, "+QMTCONN:")) {
                if (sscanf(p, " %d , %d , %d", & id, & res, & ret) >= 2) {
                }
            }
            else if (prefix(s, "+QMTSUB:") || prefix(s, "+QMTUNS:") || prefix(s, "+QMTPUB:")) {
                (void) sscanf(p, " %d , %d , %d", & id, & msg, & res);
            }
            else (void) sscanf(p, " %d , %d", & id, & res);
        }
        if (res != - 99) {
            m->mqtt_last_result = (int16_t) res;
            m->mqtt_done = 1;
        }
    }
    if (prefix(s, "+QFTPOPEN:") || prefix(s, "+QFTPCLOSE:") || prefix(s, "+QFTPGET:") || prefix(s, "+QFTPPUT:")) {
        const char *p = strchr(s, ':');
        long v = - 999;
        if (p) sscanf(p + 1, " %ld", & v);
        if (v != - 999) {
            m->ftp_last_result = (int32_t) v;
            m->ftp_done = 1;
            if (m->command_active) {
                m->command_result = (v >= 0)?M66_OK: M66_E_NETWORK;
                m->command_done = 1;
            }
        }
    }
    if (prefix(s, "+QHTTPGET:") || prefix(s, "+QHTTPPOST:")) {
        int err = 0, status = 0;
        long len = - 1;
        if (sscanf(strchr(s, ':') + 1, " %d , %d , %ld", & err, & status, & len) >= 2) {
            m->http_error = (int16_t) err;
            m->http_status = (int16_t) status;
            m->http_content_length = (int32_t) len;
            m->http_done = 1;
            push(m, M66_EV_HTTP_COMPLETE, s, NULL, status);
        }
        return;
    }
    if (prefix(s, "+QIRD:")) {
        long n = - 1;
        const char *p = strchr(s, ':');
        if (p) sscanf(p + 1, " %ld", & n);
        if (n >= 0) {
            m->raw_capture_expected = (uint16_t)((n > 65535)?65535: n);
            m->raw_capture_got = 0;
            m->raw_capture_overflow = (n > M66_RAW_DATA_SIZE);
            m->raw_capture_mode = (n > 0)?1u: 0u;
        }
    }
    if (prefix(s, "+CREG:") || prefix(s, "+CGREG:")) {
        const char *p = strchr(s, ':');
        int a = - 1, b = - 1, stat = - 1;
        if (p) {
            p++;
            if (sscanf(p, " %d , %d", & a, & b) == 2) stat = b;
            else if (sscanf(p, " %d", & a) == 1) stat = a;
        }
        if (stat == 1 || stat == 5) push(m, M66_EV_REGISTERED, s, NULL, stat);
        else if (stat >= 0) push(m, M66_EV_REGISTRATION_LOST, s, NULL, stat);
    }
    if (prefix(s, "+QIURC: \"closed\"")) {
        push(m, M66_EV_SOCKET_CLOSED, s, NULL, 0);
        return;
    }
    if (prefix(s, "+QIURC: \"recv\"")) {
        int id = 0;
        const char *p = strstr(s, "recv");
        if (p) {
            p = strchr(p, ',');
            if (p) id = atoi(p + 1);
        }
        push(m, M66_EV_SOCKET_DATA, s, NULL, id);
        return;
    }
    if (m->sms_header_pending) {
        /*SMS body line follows +CMGR header. Sender/body are parsed by SMSRead transaction. */
        m->sms_header_pending = 0;
    }
    if (m->collect_response) {
        size_t n = strlen(s);
        if ((size_t) m->response_len + n + 2u < sizeof(m->response)) {
            memcpy(m->response + m->response_len, s, n);
            m->response_len += (uint16_t) n;
            m->response[m->response_len++] = '\n';
            m->response[m->response_len] = 0;
        } else m->stats.parser_errors++;
    }
    push(m, M66_EV_RAW_LINE, s, NULL, 0);
}
static void feed(M66_Handle *m, const uint8_t *p, uint16_t n) {
    for (uint16_t i = 0; i < n; i++) {
        uint8_t c = p[i];
        m->stats.rx_bytes++;
        if (m->raw_capture_mode) {
            if (m->raw_capture_mode == 2u) {
                if (m->raw_capture_total < M66_RAW_DATA_SIZE) m->raw_capture[m->raw_capture_total] = c;
                m->raw_tail[m->raw_tail_pos] = c;
                m->raw_tail_pos = (uint8_t)((m->raw_tail_pos + 1u) % 6u);
                if (m->raw_tail_count < 6u) m->raw_tail_count++;
                m->raw_capture_total++;
                if (m->raw_tail_count == 6u && m->raw_tail[m->raw_tail_pos] == '\r' && m->raw_tail[(m->raw_tail_pos + 1u) % 6u] == '\n' && m->raw_tail[(m->raw_tail_pos + 2u) % 6u] == 'O' && m->raw_tail[(m->raw_tail_pos + 3u) % 6u] == 'K' && m->raw_tail[(m->raw_tail_pos + 4u) % 6u] == '\r' && m->raw_tail[(m->raw_tail_pos + 5u) % 6u] == '\n') {
                    m->raw_capture_mode = 0;
                    m->raw_len = (uint16_t)((m->raw_capture_total >= 6u)?((m->raw_capture_total - 6u > M66_RAW_DATA_SIZE)?M66_RAW_DATA_SIZE: m->raw_capture_total - 6u): 0u);
                    if (m->raw_capture_total >= 6u && m->raw_capture_total - 6u > M66_RAW_DATA_SIZE) m->raw_capture_overflow = 1;
                    m->raw_capture_got = m->raw_len;
                    m->command_result = M66_OK;
                    m->command_done = 1;
                } else if (m->raw_capture_total > M66_RAW_DATA_SIZE + 6u) m->raw_capture_overflow = 1;
            } else {
                if (m->raw_capture_got < M66_RAW_DATA_SIZE) m->raw_capture[m->raw_capture_got] = c;
                else m->raw_capture_overflow = 1;
                if (m->raw_capture_got < UINT16_MAX) m->raw_capture_got++;
                m->raw_capture_total++;
                if (m->raw_capture_got >= m->raw_capture_expected) {
                    m->raw_capture_mode = 0;
                    m->raw_len = (m->raw_capture_got > M66_RAW_DATA_SIZE)?M66_RAW_DATA_SIZE: m->raw_capture_got;
                }
            }
            continue;
        }
        if (c == '>' && m->line_len == 0) {
            m->command_prompt = 1;
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (m->line_len && !m->line_discard) {
                m->line[m->line_len] = 0;
                parse_line(m, (char *) m->line);
            }
            m->line_len = 0;
            m->line_discard = 0;
            continue;
        }
        if (!m->line_discard) {
            if (m->line_len + 1u < sizeof(m->line)) m->line[m->line_len++] = c;
            else {
                m->line_discard = 1;
                m->stats.parser_errors++;
            }
        }
    }
}
M66_Status M66_Init(M66_Handle *m, const M66_Config *c) {
    if (!m || !c || !c->huart || c->rx_dma_size < 64 || c->rx_dma_size > M66_RX_DMA_SIZE || c->tx_queue_size < 128 || c->tx_queue_size > M66_TX_QUEUE_SIZE) return M66_E_ARG;
    memset(m, 0, sizeof(*m));
    m->cfg = *c;
    m->huart = c->huart;
    if (c->hdma_rx) m->huart->hdmarx = c->hdma_rx;
    if (c->hdma_tx) m->huart->hdmatx = c->hdma_tx;
    if (!m->huart->hdmarx) return M66_E_ARG;
    if (c->rx_mode == M66_RX_CIRCULAR) {
        /*Configure RX DMA as circular before starting. CubeMX DMA channel/stream,
     request, alignment and IRQ configuration must already be correct. */
        m->huart->hdmarx->Init.Mode = DMA_CIRCULAR;
        if (HAL_DMA_Init(m->huart->hdmarx) != HAL_OK) return M66_E_UART;
        if (HAL_UART_Receive_DMA(m->huart, m->rx_dma, c->rx_dma_size) != HAL_OK) return M66_E_UART;
        if (!c->enable_rx_half_callback) __HAL_DMA_DISABLE_IT(m->huart->hdmarx, DMA_IT_HT);
    } else {
        if (HAL_UARTEx_ReceiveToIdle_DMA(m->huart, m->rx_dma, c->rx_dma_size) != HAL_OK) return M66_E_UART;
        if (!c->enable_rx_half_callback) __HAL_DMA_DISABLE_IT(m->huart->hdmarx, DMA_IT_HT);
    }
    m->initialized = 1;
    return M66_OK;
}
/*Drain bytes produced by circular DMA. Called both from M66_Process() and
 *optional DMA half/full callbacks. NDTR polling is portable across F4 parts;
 *it cannot detect multiple complete wraps if the consumer is stalled too long. */
static void drain_circular_dma(M66_Handle *m) {
    if (!m || !m->huart || !m->huart->hdmarx || m->cfg.rx_mode != M66_RX_CIRCULAR) return;
    uint16_t pos = (uint16_t)(m->cfg.rx_dma_size - __HAL_DMA_GET_COUNTER(m->huart->hdmarx));
    if (pos >= m->cfg.rx_dma_size) pos = 0;
    uint16_t last = m->rx_last_pos;
    if (pos >= last) {
        if (pos > last) feed(m, & m->rx_dma[last], (uint16_t)(pos - last));
    }
    else {
        feed(m, & m->rx_dma[last], (uint16_t)(m->cfg.rx_dma_size - last));
        if (pos) feed(m, m->rx_dma, pos);
    }
    m->rx_last_pos = pos;
}
void M66_UART_RxEventCallback(M66_Handle *m, uint16_t size) {
    if (!m || !m->initialized) return;
    if (m->cfg.rx_mode == M66_RX_TO_IDLE) {
        /* ReceiveToIdle is restarted after each callback. The callback is short. */
        if (size > m->cfg.rx_dma_size) size = m->cfg.rx_dma_size;
        feed(m, m->rx_dma, size);
        if (HAL_UARTEx_ReceiveToIdle_DMA(m->huart, m->rx_dma, m->cfg.rx_dma_size) != HAL_OK) {
            m->stats.uart_errors++;
            push(m, M66_EV_UART_ERROR, "RX rearm failed", NULL, HAL_UART_GetError(m->huart));
        }
        if (m->huart->hdmarx && !m->cfg.enable_rx_half_callback) __HAL_DMA_DISABLE_IT(m->huart->hdmarx, DMA_IT_HT);
    } else {
        /*Circular RX is consumed only by M66_Process() polling NDTR. Do not drain
     here too: a callback and task draining concurrently can duplicate/skip bytes. */
        (void) size;
    }
}
void M66_UART_TxCpltCallback(M66_Handle *m) {
    if (!m) return;
    m->stats.tx_bytes += m->tx_active_len;
    m->tx_busy = 0;
    tx_kick(m);
}
void M66_UART_ErrorCallback(M66_Handle *m) {
    if (!m) return;
    m->stats.uart_errors++;
    push(m, M66_EV_UART_ERROR, "UART HAL error", NULL, HAL_UART_GetError(m->huart));
}
void M66_Process(M66_Handle *m) {
    if (!m || !m->initialized) return;
    if (m->cfg.rx_mode == M66_RX_CIRCULAR) drain_circular_dma(m);
    tx_kick(m);
    if (m->command_active && !m->command_done && (int32_t)(tick(m) - m->command_deadline) >= 0) {
        m->command_result = M66_E_TIMEOUT;
        m->command_done = 1;
    }
    while (m->ev_tail != m->ev_head) {
        M66_Event e = m->events[m->ev_tail];
        m->ev_tail = (uint8_t)((m->ev_tail + 1u) % M66_EVENT_QUEUE_SIZE);
        if (m->callback) m->callback(& e, m->callback_user);
    }
}
void M66_RegisterCallback(M66_Handle *m, M66_Callback cb, void *u) {
    if (m) {
        m->callback = cb;
        m->callback_user = u;
    }
}
M66_Status M66_AT(M66_Handle *m, const char *cmd, char *out, size_t cap, uint32_t timeout) {
    if (!m || !m->initialized || !cmd) return M66_E_ARG;
    if (m->command_active) return M66_E_BUSY;
    size_t n = strlen(cmd);
    if (n + 2 >= M66_CMD_SIZE) return M66_E_ARG;
    m->command_active = 1;
    m->command_done = 0;
    m->command_prompt = 0;
    m->collect_response = 1;
    m->response_len = 0;
    m->response[0] = 0;
    m->command_result = M66_E_TIMEOUT;
    m->command_deadline = tick(m) + (timeout?timeout: m->cfg.default_timeout_ms);
    M66_Status q = tx_enqueue(m, (const uint8_t *) cmd, (uint16_t) n);
    if (q != M66_OK) {
        m->command_active = 0;
        return q;
    }
    uint8_t cr = '\r';
    q = tx_enqueue(m, & cr, 1);
    if (q != M66_OK) {
        m->command_active = 0;
        return q;
    }
    while (!m->command_done) {
        M66_Process(m);
        if ((int32_t)(tick(m) - m->command_deadline) >= 0) break;
    }
    M66_Status result = m->command_done?m->command_result: M66_E_TIMEOUT;
    if (out && cap) copystr(out, cap, m->response);
    m->command_active = 0;
    m->collect_response = 0;
    if (result == M66_OK) m->stats.commands_ok++;
    else m->stats.commands_failed++;
    return result;
}
static M66_Status atf(M66_Handle *m, const char *c, char *out, size_t cap, uint32_t t) {
    return M66_AT(m, c, out, cap, t);
}
M66_Status M66_Test(M66_Handle *m) {
    return atf(m, "AT", NULL, 0, 1000);
}
M66_Status M66_Echo(M66_Handle *m, uint8_t e) {
    return atf(m, e?"ATE1": "ATE0", NULL, 0, 1000);
}
M66_Status M66_GetManufacturer(M66_Handle *m, char *o, size_t n) {
    return atf(m, "AT+GMI", o, n, 2000);
}
M66_Status M66_GetModel(M66_Handle *m, char *o, size_t n) {
    return atf(m, "AT+GMM", o, n, 2000);
}
M66_Status M66_GetFirmware(M66_Handle *m, char *o, size_t n) {
    return atf(m, "AT+GMR", o, n, 2000);
}
M66_Status M66_GetIMEI(M66_Handle *m, char *o, size_t n) {
    return atf(m, "AT+GSN", o, n, 2000);
}
M66_Status M66_GetIMSI(M66_Handle *m, char *o, size_t n) {
    return atf(m, "AT+CIMI", o, n, 3000);
}
M66_Status M66_GetICCID(M66_Handle *m, char *o, size_t n) {
    /*Preferred for many M66 firmware builds: AT+QCCID. If unsupported, try
    AT+CCID (some firmware/revisions expose the generic command instead). */
    M66_Status s = atf(m, "AT+QCCID", o, n, 3000);
    if (s != M66_OK) s = atf(m, "AT+CCID", o, n, 3000);
    return s;
}
M66_Status M66_GetSIMReady(M66_Handle *m, char *o, size_t n) {
    return atf(m, "AT+CPIN?", o, n, 2000);
}
M66_Status M66_GetSignalQuality(M66_Handle *m, int *rssi, int *ber) {
    char r[M66_LINE_SIZE];
    M66_Status s = atf(m, "AT+CSQ", r, sizeof(r), 2000);
    if (s != M66_OK) return s;
    char *p = strstr(r, "+CSQ:");
    if (!p) return M66_E_PARSE;
    p = strchr(p, ':') + 1;
    int a = - 1, b = - 1;
    if (sscanf(p, " %d , %d", & a, & b) != 2) return M66_E_PARSE;
    if (rssi) *rssi = a;
    if (ber) *ber = b;
    return M66_OK;
}
M66_Status M66_GetNetworkRegistration(M66_Handle *m, char *o, size_t n) {
    return atf(m, "AT+CREG?", o, n, 2000);
}
M66_Status M66_GetOperator(M66_Handle *m, char *o, size_t n) {
    return atf(m, "AT+COPS?", o, n, 10000);
}
M66_Status M66_GetClock(M66_Handle *m, char *o, size_t n) {
    return atf(m, "AT+CCLK?", o, n, 2000);
}
M66_Status M66_SetBaud(M66_Handle *m, uint32_t b, uint8_t save) {
    char c[40];
    if (!(b == 4800 || b == 9600 || b == 19200 || b == 38400 || b == 57600 || b == 115200)) return M66_E_ARG;
    snprintf(c, sizeof(c), "AT+IPR=%lu", (unsigned long) b);
    M66_Status s = atf(m, c, NULL, 0, 2000);
    if (s != M66_OK) return s;
    return save?atf(m, "AT&W", NULL, 0, 2000): M66_OK;
}
M66_Status M66_SaveProfile(M66_Handle *m) {
    return atf(m, "AT&W", NULL, 0, 2000);
}
M66_Status M66_RestoreProfile(M66_Handle *m) {
    return atf(m, "ATZ", NULL, 0, 3000);
}
M66_Status M66_FactoryReset(M66_Handle *m) {
    return atf(m, "AT&F", NULL, 0, 3000);
}
M66_Status M66_SoftReset(M66_Handle *m) {
    return atf(m, "AT+CFUN=1,1", NULL, 0, 10000);
}
M66_Status M66_PowerDown(M66_Handle *m) {
    /*M66 commonly supports AT+QPOWD. Alternative (not equivalent on every
    revision): AT+CFUN=0; use only after checking the hardware design/manual. */
    return atf(m, "AT+QPOWD", NULL, 0, 10000);
}
M66_Status M66_SetPIN(M66_Handle *m, const char *p) {
    char c[48];
    if (!p) return M66_E_ARG;
    snprintf(c, sizeof(c), "AT+CPIN=\"%s\"", p);
    return atf(m, c, NULL, 0, 5000);
}
M66_Status M66_GetBattery(M66_Handle *m, char *o, size_t n) {
    return atf(m, "AT+CBC", o, n, 2000);
}
M66_Status M66_GetTemperature(M66_Handle *m, char *o, size_t n) {
    return atf(m, "AT+QTEMP", o, n, 3000);
}
M66_Status M66_SetFunctionality(M66_Handle *m, uint8_t f) {
    char c[24];
    if (f > 1) return M66_E_ARG;
    snprintf(c, sizeof(c), "AT+CFUN=%u", f);
    return atf(m, c, NULL, 0, 10000);
}
M66_Status M66_GetErrorReportMode(M66_Handle *m, uint8_t *x) {
    char r[M66_LINE_SIZE];
    M66_Status s = atf(m, "AT+CMEE?", r, sizeof(r), 2000);
    if (s == M66_OK && x) {
        char *p = strstr(r, "+CMEE:");
        if (p) *x = (uint8_t) atoi(strchr(p, ':') + 1);
    }
    return s;
}
M66_Status M66_SetErrorReportMode(M66_Handle *m, uint8_t x) {
    char c[20];
    if (x > 2) return M66_E_ARG;
    snprintf(c, sizeof(c), "AT+CMEE=%u", x);
    return atf(m, c, NULL, 0, 2000);
}
M66_Status M66_SetCLIP(M66_Handle *m, uint8_t x) {
    char c[24];
    snprintf(c, sizeof(c), "AT+CLIP=%u", !!x);
    return atf(m, c, NULL, 0, 2000);
}
M66_Status M66_SetCallerIDPresentation(M66_Handle *m, uint8_t x) {
    char c[24];
    snprintf(c, sizeof(c), "AT+COLP=%u", !!x);
    return atf(m, c, NULL, 0, 2000);
}
M66_Status M66_SetSMSDeliveryReport(M66_Handle *m, uint8_t x) {
    char c[24];
    snprintf(c, sizeof(c), "AT+CNMI=2,1,0,0,%u", !!x);
    return atf(m, c, NULL, 0, 2000);
}
M66_Status M66_SMSInit(M66_Handle *m, uint8_t text) {
    M66_Status s = atf(m, text?"AT+CMGF=1": "AT+CMGF=0", NULL, 0, 2000);
    if (s != M66_OK) return s;
    s = atf(m, "AT+CSCS=\"GSM\"", NULL, 0, 2000);
    if (s != M66_OK) return s;
    return atf(m, "AT+CNMI=2,1,0,0,0", NULL, 0, 2000);
}
M66_Status M66_SMSSend(M66_Handle *m, const char *num, const char *txt, uint32_t timeout) {
    if (!m || !num || !txt) return M66_E_ARG;
    /*Prompt-aware raw transaction: this implementation waits for prompt then writes
    text+Ctrl-Z directly. UART RX parser records '>' without requiring CR/LF. */
    if (m->command_active) return M66_E_BUSY;
    char c[64];
    if (snprintf(c, sizeof(c), "AT+CMGS=\"%s\"", num) >= (int) sizeof(c)) return M66_E_ARG;
    m->command_active = 1;
    m->command_done = 0;
    m->command_prompt = 0;
    m->command_result = M66_E_TIMEOUT;
    m->command_deadline = tick(m) + (timeout?timeout: m->cfg.prompt_timeout_ms);
    M66_Status q = tx_enqueue(m, (uint8_t *) c, (uint16_t) strlen(c));
    uint8_t cr = '\r';
    if (q == M66_OK) q = tx_enqueue(m, & cr, 1);
    if (q != M66_OK) {
        m->command_active = 0;
        return q;
    }
    while (!m->command_prompt && !m->command_done) {
        M66_Process(m);
        if ((int32_t)(tick(m) - m->command_deadline) >= 0) break;
    }
    if (!m->command_prompt) {
        m->command_active = 0;
        return M66_E_TIMEOUT;
    }
    size_t n = strlen(txt);
    if (n > M66_TX_QUEUE_SIZE - 2) {
        m->command_active = 0;
        return M66_E_ARG;
    }
    /*Raw bytes must not be CR-terminated. Queue currently stores bytes; use direct DMA chunks. */
    while (m->tx_busy) M66_Process(m);
    uint16_t off = 0;
    while (off < n) {
        uint16_t chunk = (uint16_t)((n - off) > M66_TX_CHUNK_SIZE?M66_TX_CHUNK_SIZE: (n - off));
        while (m->tx_busy) M66_Process(m);
        memcpy(m->tx_active, txt + off, chunk);
        m->tx_active_len = chunk;
        m->tx_busy = 1;
        if (HAL_UART_Transmit_DMA(m->huart, m->tx_active, chunk) != HAL_OK) {
            m->tx_busy = 0;
            m->command_active = 0;
            return M66_E_UART;
        }
        off += chunk;
    }
    while (m->tx_busy) M66_Process(m);
    uint8_t end = 0x1A;
    m->tx_active[0] = end;
    m->tx_active_len = 1;
    m->tx_busy = 1;
    if (HAL_UART_Transmit_DMA(m->huart, m->tx_active, 1) != HAL_OK) {
        m->tx_busy = 0;
        m->command_active = 0;
        return M66_E_UART;
    }
    m->command_deadline = tick(m) + (timeout?timeout: 60000);
    while (!m->command_done) {
        M66_Process(m);
        if ((int32_t)(tick(m) - m->command_deadline) >= 0) break;
    }
    M66_Status result = m->command_done?m->command_result: M66_E_TIMEOUT;
    m->command_active = 0;
    if (result == M66_OK) push(m, M66_EV_SMS_SENT, "SMS sent", num, 0);
    return result;
}
M66_Status M66_SMSRead(M66_Handle *m, uint32_t idx, char *sender, size_t sc, char *body, size_t bc, uint32_t t) {
    char c[40], r[M66_LINE_SIZE];
    snprintf(c, sizeof(c), "AT+CMGR=%lu", (unsigned long) idx);
    M66_Status s = atf(m, c, r, sizeof(r), t?t: 5000);
    if (s != M66_OK) return s;
    /*Response parser keeps text lines. Extract quoted sender from +CMGR header. */
    char *p = strstr(r, "+CMGR:");
    if (!p) return M66_E_PARSE;
    char *q = strchr(p, '"');
    if (!q) return M66_E_PARSE;
    q = strchr(q + 1, '"');
    if (!q) return M66_E_PARSE;
    q = strchr(q + 1, '"');
    if (!q) return M66_E_PARSE;
    char *e = strchr(q + 1, '"');
    if (!e) return M66_E_PARSE;
    size_t n = (size_t)(e - q - 1);
    if (sc) {
        if (n >= sc) n = sc - 1;
        memcpy(sender, q + 1, n);
        sender[n] = 0;
    }
    char *nl = strchr(e, '\n');
    if (!nl) return M66_E_PARSE;
    while (*nl == '\n' || *nl == '\r') nl++;
    copystr(body, bc, nl);
    char *z = strpbrk(body, "\r\n");
    if (z) *z = 0;
    return M66_OK;
}
M66_Status M66_SMSDelete(M66_Handle *m, uint32_t i) {
    char c[40];
    snprintf(c, sizeof(c), "AT+CMGD=%lu", (unsigned long) i);
    return atf(m, c, NULL, 0, 3000);
}
M66_Status M66_SMSList(M66_Handle *m, char *o, size_t n) {
    return atf(m, "AT+CMGL=\"ALL\"", o, n, 10000);
}
M66_Status M66_SMSSetStorage(M66_Handle *m, const char *r, const char *w, const char *s) {
    char c[80];
    if (!r || !w || !s) return M66_E_ARG;
    snprintf(c, sizeof(c), "AT+CPMS=\"%s\",\"%s\",\"%s\"", r, w, s);
    return atf(m, c, NULL, 0, 3000);
}
M66_Status M66_Call(M66_Handle *m, const char *n) {
    char c[64];
    if (!n) return M66_E_ARG;
    snprintf(c, sizeof(c), "ATD%s;", n);
    return atf(m, c, NULL, 0, 10000);
}
M66_Status M66_Answer(M66_Handle *m) {
    return atf(m, "ATA", NULL, 0, 5000);
}
M66_Status M66_Hangup(M66_Handle *m) {
    return atf(m, "ATH", NULL, 0, 5000);
}
M66_Status M66_DTMFEnable(M66_Handle *m, uint8_t e) {
    char c[24];
    snprintf(c, sizeof(c), "AT+DDET=%u", !!e);
    return atf(m, c, NULL, 0, 2000);
}
M66_Status M66_DTMFSend(M66_Handle *m, char d, uint16_t ms) {
    char c[32];
    if (!((d >= '0' && d <= '9') || d == '*' || d == '#' || (d >= 'A' && d <= 'D'))) return M66_E_ARG;
    snprintf(c, sizeof(c), "AT+VTS=%c,%u", d, ms);
    return atf(m, c, NULL, 0, 3000);
}
M66_Status M66_GPRSSetAPN(M66_Handle *m, const char *a, const char *u, const char *p) {
    char c[192];
    if (!m || !a || strpbrk(a, "\r\n\"") || (u && strpbrk(u, "\r\n\"")) || (p && strpbrk(p, "\r\n\""))) return M66_E_ARG;
    /*M66 Series AT manual: AT+QICSGP=1,"APN","USER","PASSWORD".
    Older sample code sometimes puts credentials in AT+QIREGAPP instead; if
    this firmware rejects QICSGP, verify AT+QICSGP=? and the exact manual. */
    int k = snprintf(c, sizeof(c), "AT+QICSGP=1,\"%s\",\"%s\",\"%s\"", a, u?u: "", p?p: "");
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    return atf(m, c, NULL, 0, 3000);
}
M66_Status M66_GPRSActivate(M66_Handle *m, uint32_t t) {
    M66_Status s = atf(m, "AT+QIREGAPP", NULL, 0, 5000);
    if (s != M66_OK) return s;
    return atf(m, "AT+QIACT", NULL, 0, t?t: 85000);
}
M66_Status M66_GPRSDeactivate(M66_Handle *m) {
    return atf(m, "AT+QIDEACT", NULL, 0, 40000);
}
M66_Status M66_GPRSGetIP(M66_Handle *m, char *o, size_t n) {
    return atf(m, "AT+QILOCIP", o, n, 5000);
}
M66_Status M66_GPRSSetDNS(M66_Handle *m, const char *a, const char *b) {
    /*Firmware-specific: try AT+QIDNSCFG=1,"primary","secondary" on
    revisions implementing QIDNSCFG. Other revisions obtain DNS from PDP.
    This is deliberately not sent blindly until the exact syntax is confirmed. */
    if (!m || !a || !b) return M66_E_ARG;
    char c[128];
    snprintf(c, sizeof(c), "AT+QIDNSCFG=1,\"%s\",\"%s\"", a, b);
    return atf(m, c, NULL, 0, 3000);
}
M66_Status M66_SocketOpen(M66_Handle *m, uint8_t id, const char *type, const char *host, uint16_t port, uint32_t t) {
    char c[192];
    M66_Status s;
    if (!m || id > 5 || !type || !host || !port || strpbrk(host, "\r\n\"") || !(strcmp(type, "TCP") == 0 || strcmp(type, "UDP") == 0)) return M66_E_ARG;
    /*M66 Series manual syntax (multiplexing enabled): AT+QIOPEN=<index>,"TCP","host",<port>.
    Single-socket alternative after AT+QIMUX=0: AT+QIOPEN="TCP","host",<port>.
    Do not use newer Quectel QIOPEN syntax with context-id/keepalive flags on M66. */
    s = atf(m, "AT+QIMUX=1", NULL, 0, 2000);
    if (s != M66_OK) return s;
    s = atf(m, "AT+QIMODE=0", NULL, 0, 2000);
    if (s != M66_OK) return s;
    s = atf(m, "AT+QIDNSIP=1", NULL, 0, 2000);
    if (s != M66_OK) return s;
    m->socket_connected = 0;
    m->socket_open_failed = 0;
    int k = snprintf(c, sizeof(c), "AT+QIOPEN=%u,\"%s\",\"%s\",%u", id, type, host, port);
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    s = atf(m, c, NULL, 0, 10000);
    if (s != M66_OK && s != M66_E_TIMEOUT) return s;
    uint32_t deadline = tick(m) + (t?t: 75000u);
    while (!m->socket_connected && !m->socket_open_failed && (int32_t)(tick(m) - deadline) < 0) M66_Process(m);
    if (m->socket_connected) return M66_OK;
    return m->socket_open_failed?M66_E_NETWORK: M66_E_TIMEOUT;
}
M66_Status M66_SocketSend(M66_Handle *m, uint8_t id, const uint8_t *d, uint16_t n, uint32_t t) {
    if (!m || id > 5 || !d || !n) return M66_E_ARG;
    char c[48];
    snprintf(c, sizeof(c), "AT+QISEND=%u,%u", id, n);
    if (m->command_active) return M66_E_BUSY;
    m->command_active = 1;
    m->command_done = 0;
    m->command_prompt = 0;
    m->command_result = M66_E_TIMEOUT;
    m->command_deadline = tick(m) + (t?t: m->cfg.prompt_timeout_ms);
    M66_Status q = tx_enqueue(m, (uint8_t *) c, (uint16_t) strlen(c));
    uint8_t cr = '\r';
    if (q == M66_OK) q = tx_enqueue(m, & cr, 1);
    if (q != M66_OK) {
        m->command_active = 0;
        return q;
    }
    while (!m->command_prompt && !m->command_done) {
        M66_Process(m);
        if ((int32_t)(tick(m) - m->command_deadline) >= 0) break;
    }
    if (!m->command_prompt) {
        m->command_active = 0;
        return M66_E_TIMEOUT;
    }
    while (m->tx_busy) M66_Process(m);
    uint16_t off = 0;
    while (off < n) {
        uint16_t k = (uint16_t)((n - off) > M66_TX_CHUNK_SIZE?M66_TX_CHUNK_SIZE: (n - off));
        while (m->tx_busy) M66_Process(m);
        memcpy(m->tx_active, d + off, k);
        m->tx_active_len = k;
        m->tx_busy = 1;
        if (HAL_UART_Transmit_DMA(m->huart, m->tx_active, k) != HAL_OK) {
            m->tx_busy = 0;
            m->command_active = 0;
            return M66_E_UART;
        }
        off += k;
    }
    while (m->tx_busy) M66_Process(m);
    m->command_deadline = tick(m) + (t?t: 30000);
    while (!m->command_done) {
        M66_Process(m);
        if ((int32_t)(tick(m) - m->command_deadline) >= 0) break;
    }
    M66_Status s = m->command_done?m->command_result: M66_E_TIMEOUT;
    m->command_active = 0;
    return s;
}
M66_Status M66_SocketReadData(M66_Handle *m, uint8_t id, uint16_t n, uint8_t *out, size_t cap, size_t *received, uint32_t t) {
    char c[48], r[M66_LINE_SIZE];
    if (!m || id > 5 || !n || !out || !cap || !received) return M66_E_ARG;
    *received = 0;
    if (m->command_active) return M66_E_BUSY;
    m->raw_capture_mode = 0;
    m->raw_capture_expected = 0;
    m->raw_capture_got = 0;
    m->raw_len = 0;
    m->raw_capture_overflow = 0;
    snprintf(c, sizeof(c), "AT+QIRD=%u,%u", id, n);
    M66_Status s = atf(m, c, r, sizeof(r), t?t: 10000);
    if (s != M66_OK) return s;
    size_t got = m->raw_len;
    if (got > cap) got = cap;
    memcpy(out, m->raw_capture, got);
    *received = got;
    if (m->raw_capture_overflow || m->raw_capture_expected > M66_RAW_DATA_SIZE || m->raw_len > cap) return M66_E_OVERFLOW;
    return M66_OK;
}
M66_Status M66_SocketRead(M66_Handle *m, uint8_t id, uint16_t n, uint32_t t) {
    uint8_t sink[M66_RAW_DATA_SIZE];
    size_t got = 0;
    return M66_SocketReadData(m, id, n, sink, sizeof(sink), & got, t);
}
M66_Status M66_SocketClose(M66_Handle *m, uint8_t id) {
    char c[32];
    if (id > 5) return M66_E_ARG;
    snprintf(c, sizeof(c), "AT+QICLOSE=%u", id);
    return atf(m, c, NULL, 0, 10000);
}
/*Prompt/raw helpers. They keep the command transaction active across CONNECT/>. */
static M66_Status wait_for_prompt(M66_Handle *m, const char *cmd, uint32_t timeout) {
    if (!m || !m->initialized || !cmd) return M66_E_ARG;
    if (m->command_active) return M66_E_BUSY;
    size_t n = strlen(cmd);
    if (n + 2u >= M66_CMD_SIZE) return M66_E_ARG;
    m->command_active = 1;
    m->command_done = 0;
    m->command_prompt = 0;
    m->collect_response = 1;
    m->response_len = 0;
    m->response[0] = 0;
    m->command_result = M66_E_TIMEOUT;
    m->command_deadline = tick(m) + (timeout?timeout: m->cfg.prompt_timeout_ms);
    M66_Status q = tx_enqueue(m, (const uint8_t *) cmd, (uint16_t) n);
    if (q == M66_OK) {
        uint8_t cr = '\r';
        q = tx_enqueue(m, & cr, 1);
    }
    if (q != M66_OK) {
        m->command_active = 0;
        return q;
    }
    while (!m->command_prompt && !m->command_done) {
        M66_Process(m);
        if ((int32_t)(tick(m) - m->command_deadline) >= 0) break;
    }
    if (m->command_prompt) return M66_OK;
    M66_Status result = m->command_done?m->command_result: M66_E_TIMEOUT;
    m->command_active = 0;
    m->collect_response = 0;
    return result;
}
static M66_Status wait_command_final(M66_Handle *m, uint32_t timeout) {
    if (!m) return M66_E_ARG;
    m->command_deadline = tick(m) + (timeout?timeout: m->cfg.default_timeout_ms);
    while (!m->command_done) {
        M66_Process(m);
        if ((int32_t)(tick(m) - m->command_deadline) >= 0) break;
    }
    M66_Status r = m->command_done?m->command_result: M66_E_TIMEOUT;
    m->command_active = 0;
    m->collect_response = 0;
    if (r == M66_OK) m->stats.commands_ok++;
    else m->stats.commands_failed++;
    return r;
}
static M66_Status raw_tx(M66_Handle *m, const uint8_t *data, size_t len, uint32_t timeout) {
    if (!m ||(!data && len)) return M66_E_ARG;
    uint32_t deadline = tick(m) + (timeout?timeout: 10000u);
    size_t off = 0;
    while (off < len) {
        while (m->tx_busy || m->tx_tail != m->tx_head) {
            M66_Process(m);
            if ((int32_t)(tick(m) - deadline) >= 0) return M66_E_TIMEOUT;
        }
        size_t n = len - off;
        if (n > M66_TX_CHUNK_SIZE) n = M66_TX_CHUNK_SIZE;
        memcpy(m->tx_active, data + off, n);
        m->tx_active_len = (uint16_t) n;
        m->tx_busy = 1;
        if (HAL_UART_Transmit_DMA(m->huart, m->tx_active, (uint16_t) n) != HAL_OK) {
            m->tx_busy = 0;
            m->stats.uart_errors++;
            return M66_E_UART;
        }
        off += n;
    }
    while (m->tx_busy) {
        M66_Process(m);
        if ((int32_t)(tick(m) - deadline) >= 0) return M66_E_TIMEOUT;
    }
    return M66_OK;
}
static int safe_text(const char *s, int allow_quote) {
    if (!s) return 0;
    for (; *s; s++) {
        if (*s == '\r' || *s == '\n' ||(!allow_quote && *s == '"')) return 0;
    }
    return 1;
}
M66_Status M66_USSD(M66_Handle *m, const char *code, uint32_t timeout) {
    char c[192];
    if (!m || !code || !safe_text(code, 0) || strlen(code) > 140) return M66_E_ARG;
    m->ussd_text[0] = 0;
    int k = snprintf(c, sizeof(c), "AT+CUSD=1,\"%s\"", code);
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    M66_Status s = atf(m, c, NULL, 0, 3000);
    if (s != M66_OK) return s;
    uint32_t deadline = tick(m) + (timeout?timeout: 120000u);
    while (!m->ussd_text[0] && (int32_t)(tick(m) - deadline) < 0) M66_Process(m);
    return m->ussd_text[0]?M66_OK: M66_E_TIMEOUT;
}
M66_Status M66_USSDCancel(M66_Handle *m) {
    return atf(m, "AT+CUSD=2", NULL, 0, 3000);
}
M66_Status M66_HTTPSetURL(M66_Handle *m, const char *url, uint32_t timeout) {
    char c[64];
    if (!m || !url || !safe_text(url, 0)) return M66_E_ARG;
    size_t n = strlen(url);
    if (n == 0 || n > 450u) return M66_E_ARG;
    int k = snprintf(c, sizeof(c), "AT+QHTTPURL=%lu,%lu", (unsigned long) n, (unsigned long)(timeout?timeout / 1000u: 30u));
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    M66_Status s = wait_for_prompt(m, c, timeout?timeout: 30000u);
    if (s != M66_OK) return s;
    s = raw_tx(m, (const uint8_t *) url, n, timeout?timeout: 30000u);
    if (s != M66_OK) {
        m->command_active = 0;
        return s;
    }
    return wait_command_final(m, timeout?timeout: 30000u);
}
M66_Status M66_HTTPSetParameter(M66_Handle *m, const char *name, const char *value) {
    char c[192];
    if (!m || !name || !value || !safe_text(name, 0) || !safe_text(value, 0) || strlen(name) > 40 || strlen(value) > 100) return M66_E_ARG;
    /*M66-specific firmware uses QHTTPCFG on some revisions, e.g.
    AT+QHTTPCFG="requestheader",1. This API accepts a raw simple value such as 0/1.
    Arbitrary custom headers are not supported by every M66 HTTP firmware. */
    int k = snprintf(c, sizeof(c), "AT+QHTTPCFG=\"%s\",%s", name, value);
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    return atf(m, c, NULL, 0, 3000);
}
M66_Status M66_HTTPSetSSL(M66_Handle *m, uint8_t enable) {
    /*M66 HTTP firmware may not support TLS configuration. This is a best-effort
    command for revisions that implement QHTTPCFG; plain HTTP remains separate. */
    return atf(m, enable?"AT+QHTTPCFG=\"ssl\",1": "AT+QHTTPCFG=\"ssl\",0", NULL, 0, 3000);
}
M66_Status M66_HTTPGet(M66_Handle *m, uint32_t timeout, int *status, int32_t *length) {
    if (!m) return M66_E_ARG;
    m->http_done = 0;
    m->http_status = - 1;
    m->http_content_length = - 1;
    char c[40];
    snprintf(c, sizeof(c), "AT+QHTTPGET=%lu", (unsigned long)(timeout?timeout / 1000u: 60u));
    M66_Status s = atf(m, c, NULL, 0, 10000);
    if (s != M66_OK) return s;
    /*Classic M66 HTTP manual returns OK here and exposes no HTTP status/length URC.
    Newer firmware may emit +QHTTPGET: err,status,length asynchronously. */
    uint32_t short_deadline = tick(m) + 300u;
    while (!m->http_done && (int32_t)(tick(m) - short_deadline) < 0) M66_Process(m);
    if (status) *status = m->http_done?m->http_status: - 1;
    if (length) *length = m->http_done?m->http_content_length: - 1;
    if (m->http_done && (m->http_error != 0 || m->http_status >= 400)) return M66_E_NETWORK;
    return M66_OK;
}
M66_Status M66_HTTPPost(M66_Handle *m, const char *url, const uint8_t *body, size_t len, uint32_t timeout, int *status, int32_t *response_length) {
    char c[64];
    if (!m || !url ||(!body && len) || len == 0 || len > 29696u) return M66_E_ARG;
    M66_Status s = M66_HTTPSetURL(m, url, timeout);
    if (s != M66_OK) return s;
    m->http_done = 0;
    m->http_status = - 1;
    m->http_content_length = - 1;
    int k = snprintf(c, sizeof(c), "AT+QHTTPPOST=%lu,%lu,%lu", (unsigned long) len, (unsigned long)(timeout?timeout / 1000u: 60u), (unsigned long)(timeout?timeout / 1000u: 60u));
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    s = wait_for_prompt(m, c, timeout?timeout: 60000u);
    if (s != M66_OK) return s;
    s = raw_tx(m, body, len, timeout?timeout: 60000u);
    if (s != M66_OK) {
        m->command_active = 0;
        return s;
    }
    s = wait_command_final(m, timeout?timeout: 60000u);
    if (s != M66_OK) return s;
    uint32_t short_deadline = tick(m) + 300u;
    while (!m->http_done && (int32_t)(tick(m) - short_deadline) < 0) M66_Process(m);
    if (status) *status = m->http_done?m->http_status: - 1;
    if (response_length) *response_length = m->http_done?m->http_content_length: - 1;
    if (m->http_done && (m->http_error != 0 || m->http_status >= 400)) return M66_E_NETWORK;
    return M66_OK;
}
M66_Status M66_HTTPRead(M66_Handle *m, uint8_t *out, size_t cap, size_t *received, uint32_t timeout) {
    if (!m || !out || !cap || !received) return M66_E_ARG;
    *received = 0;
    if (m->http_content_length > (int32_t) M66_RAW_DATA_SIZE) return M66_E_OVERFLOW;
    m->raw_capture_mode = 0;
    m->raw_capture_expected = 0;
    m->raw_capture_got = 0;
    m->raw_capture_total = 0;
    m->raw_len = 0;
    m->raw_capture_overflow = 0;
    m->http_read_armed = 1;
    char c[40];
    snprintf(c, sizeof(c), "AT+QHTTPREAD=%lu", (unsigned long)(timeout?timeout / 1000u: 60u));
    M66_Status s = atf(m, c, NULL, 0, timeout?timeout: 70000u);
    m->http_read_armed = 0;
    if (s != M66_OK) return s;
    size_t n = m->raw_len;
    if (n > cap) n = cap;
    memcpy(out, m->raw_capture, n);
    *received = n;
    if (m->raw_capture_overflow || m->raw_len > cap || (m->http_content_length >= 0 && m->raw_len < (size_t) m->http_content_length)) return M66_E_OVERFLOW;
    return M66_OK;
}
M66_Status M66_HTTPStop(M66_Handle *m) {
    /*QHTTPSTOP is not part of the classic M66 GSM HTTP AT command manual. */
    (void) m;
    return M66_E_UNSUPPORTED;
}
M66_Status M66_HTTPDownloadToModemFS(M66_Handle *m, const char *path, uint32_t max_bytes, char *out, size_t cap, uint32_t timeout) {
    char c[192];
    if (!m || !path || !safe_text(path, 0) || max_bytes == 0) return M66_E_ARG;
    if (strchr(path, '\\') || strlen(path) > 100u) return M66_E_ARG;
    /*M66 GSM HTTP manual: AT+QHTTPDL="RAM:1.TXT",<max_size>[,<wait_time>].
    The file is stored inside the modem, not directly on the STM32 SD card. */
    int k = snprintf(c, sizeof(c), "AT+QHTTPDL=\"%s\",%lu,%lu", path, (unsigned long) max_bytes, (unsigned long)(timeout?timeout / 1000u: 60u));
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    return atf(m, c, out, cap, timeout?timeout: 90000u);
}
M66_Status M66_FTPSetUser(M66_Handle *m, const char *u) {
    char c[80];
    if (!m || !u || !safe_text(u, 0) || strlen(u) > 30) return M66_E_ARG;
    int k = snprintf(c, sizeof(c), "AT+QFTPUSER=\"%s\"", u);
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    return atf(m, c, NULL, 0, 5000);
}
M66_Status M66_FTPSetPassword(M66_Handle *m, const char *p) {
    char c[80];
    if (!m || !p || !safe_text(p, 0) || strlen(p) > 30) return M66_E_ARG;
    int k = snprintf(c, sizeof(c), "AT+QFTPPASS=\"%s\"", p);
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    return atf(m, c, NULL, 0, 5000);
}
M66_Status M66_FTPSetPath(M66_Handle *m, const char *p) {
    char c[140];
    if (!m || !p || !safe_text(p, 0) || strlen(p) > 100) return M66_E_ARG;
    int k = snprintf(c, sizeof(c), "AT+QFTPPATH=\"%s\"", p);
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    return atf(m, c, NULL, 0, 5000);
}
M66_Status M66_FTPSetConfig(M66_Handle *m, uint8_t type, const char *value) {
    char c[160];
    if (!m || !value || type < 1 || type > 4 || !safe_text(value, 0) || strlen(value) > 100) return M66_E_ARG;
    int k;
    if (type == 4) k = snprintf(c, sizeof(c), "AT+QFTPCFG=4,\"%s\"", value);
    else {
        char *end = 0;
        long v = strtol(value, & end, 10);
        if (!end || *end || v < 0 || v > UINT32_MAX || ((type == 1u || type == 2u) && v > 1)) return M66_E_ARG;
        k = snprintf(c, sizeof(c), "AT+QFTPCFG=%u,%lu", type, (unsigned long) v);
    }
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    return atf(m, c, NULL, 0, 5000);
}
static M66_Status ftp_wait(M66_Handle *m, uint32_t timeout) {
    uint32_t deadline = tick(m) + (timeout?timeout: 120000u);
    while (!m->ftp_done && (int32_t)(tick(m) - deadline) < 0) M66_Process(m);
    if (!m->ftp_done) return M66_E_TIMEOUT;
    return m->ftp_last_result >= 0?M66_OK: M66_E_NETWORK;
}
M66_Status M66_FTPOpen(M66_Handle *m, const char *host, uint16_t port, uint32_t timeout) {
    char c[160];
    if (!m || !host || !safe_text(host, 0) || strlen(host) > 100 || !port) return M66_E_ARG;
    int k = snprintf(c, sizeof(c), "AT+QFTPOPEN=\"%s\",%u", host, port);
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    m->ftp_done = 0;
    m->ftp_last_result = - 999;
    M66_Status s = atf(m, c, NULL, 0, 10000);
    if (s != M66_OK) return s;
    return ftp_wait(m, timeout?timeout: 120000u);
}
M66_Status M66_FTPGet(M66_Handle *m, const char *remote, uint32_t max_bytes, uint32_t timeout) {
    char c[160];
    if (!m || !remote || !safe_text(remote, 0) || strlen(remote) > 50) return M66_E_ARG;
    int k;
    if (max_bytes) k = snprintf(c, sizeof(c), "AT+QFTPGET=\"%s\",%lu", remote, (unsigned long) max_bytes);
    else k = snprintf(c, sizeof(c), "AT+QFTPGET=\"%s\"", remote);
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    m->ftp_done = 0;
    m->ftp_last_result = - 999;
    M66_Status s = atf(m, c, NULL, 0, 10000);
    if (s != M66_OK) return s;
    return ftp_wait(m, timeout?timeout: 120000u);
}
M66_Status M66_FTPPut(M66_Handle *m, const char *remote, uint32_t file_size, uint32_t timeout) {
    char c[160];
    if (!m || !remote || !safe_text(remote, 0) || strlen(remote) > 50) return M66_E_ARG;
    int k = snprintf(c, sizeof(c), "AT+QFTPPUT=\"%s\",%lu,%lu", remote, (unsigned long) file_size, (unsigned long)(timeout?timeout / 1000u: 900u));
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    m->ftp_done = 0;
    m->ftp_last_result = - 999;
    M66_Status s = atf(m, c, NULL, 0, 10000);
    if (s != M66_OK) return s;
    return ftp_wait(m, timeout?timeout: 120000u);
}
M66_Status M66_FTPClose(M66_Handle *m) {
    m->ftp_done = 0;
    m->ftp_last_result = - 999;
    M66_Status s = atf(m, "AT+QFTPCLOSE", NULL, 0, 5000);
    if (s != M66_OK) return s;
    return ftp_wait(m, 15000);
}
M66_Status M66_FTPStatus(M66_Handle *m, char *out, size_t cap) {
    return atf(m, "AT+QFTPSTAT", out, cap, 5000);
}
M66_Status M66_FTPLength(M66_Handle *m, char *out, size_t cap) {
    return atf(m, "AT+QFTPLEN", out, cap, 5000);
}
M66_Status M66_GPRSInit(M66_Handle *m, const char *apn, const char *user, const char *pass, uint32_t timeout) {
    if (!m || !apn) return M66_E_ARG;
    M66_Status s = atf(m, "AT+QIFGCNT=0", NULL, 0, 3000);
    if (s != M66_OK) return s;
    s = M66_GPRSSetAPN(m, apn, user, pass);
    if (s != M66_OK) return s;
    s = atf(m, "AT+QIREGAPP", NULL, 0, 5000);
    if (s != M66_OK) {
        /*Alternative documented M66 form sets credentials directly on QIREGAPP. */
        char c[192];
        int k = snprintf(c, sizeof(c), "AT+QIREGAPP=\"%s\",\"%s\",\"%s\"", apn, user?user: "", pass?pass: "");
        if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
        s = atf(m, c, NULL, 0, 5000);
        if (s != M66_OK) return s;
    }
    return atf(m, "AT+QIACT", NULL, 0, timeout?timeout: 85000u);
}
M66_Status M66_GetGPRSRegistration(M66_Handle *m, char *out, size_t cap) {
    return atf(m, "AT+CGREG?", out, cap, 3000);
}
M66_Status M66_GetNetworkMode(M66_Handle *m, char *out, size_t cap) {
    return atf(m, "AT+QNWINFO", out, cap, 3000);
}
M66_Status M66_GetIMEIAlt(M66_Handle *m, char *out, size_t cap) {
    M66_Status s = atf(m, "AT+GSN", out, cap, 2000);
    if (s != M66_OK) s = atf(m, "AT+CGSN", out, cap, 2000);
    return s;
}
static M66_Status mqtt_wait(M66_Handle *m, uint32_t timeout) {
    uint32_t deadline = tick(m) + (timeout?timeout: 30000u);
    while (!m->mqtt_done && (int32_t)(tick(m) - deadline) < 0) M66_Process(m);
    if (!m->mqtt_done) return M66_E_TIMEOUT;
    return (m->mqtt_last_result == 0)?M66_OK: M66_E_NETWORK;
}
M66_Status M66_MQTTOpen(M66_Handle *m, uint8_t id, const char *host, uint16_t port, uint32_t timeout) {
    char c[180];
    if (!m || id > 5 || !host || !safe_text(host, 0) || strlen(host) > 100 || !port) return M66_E_ARG;
    m->mqtt_done = 0;
    m->mqtt_last_result = - 99;
    int k = snprintf(c, sizeof(c), "AT+QMTOPEN=%u,\\\"%s\\\",%u", id, host, port);
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    M66_Status s = atf(m, c, NULL, 0, 10000);
    if (s != M66_OK) return s;
    return mqtt_wait(m, timeout?timeout: 120000u);
}
M66_Status M66_MQTTConnect(M66_Handle *m, uint8_t id, const char *client, const char *user, const char *pass, uint32_t timeout) {
    char c[240];
    if (!m || id > 5 || !client || !safe_text(client, 0) || strlen(client) > 80 || (user && !safe_text(user, 0)) || (pass && !safe_text(pass, 0))) return M66_E_ARG;
    int k;
    if (user && pass) k = snprintf(c, sizeof(c), "AT+QMTCONN=%u,\\\"%s\\\",\\\"%s\\\",\\\"%s\\\"", id, client, user, pass);
    else k = snprintf(c, sizeof(c), "AT+QMTCONN=%u,\\\"%s\\\"", id, client);
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    m->mqtt_done = 0;
    m->mqtt_last_result = - 99;
    M66_Status s = atf(m, c, NULL, 0, 10000);
    if (s != M66_OK) return s;
    return mqtt_wait(m, timeout?timeout: 60000u);
}
M66_Status M66_MQTTSubscribe(M66_Handle *m, uint8_t id, uint16_t msgid, const char *topic, uint8_t qos, uint32_t timeout) {
    char c[200];
    if (!m || id > 5 || !topic || !safe_text(topic, 0) || strlen(topic) > 100 || qos > 2) return M66_E_ARG;
    int k = snprintf(c, sizeof(c), "AT+QMTSUB=%u,%u,\\\"%s\\\",%u", id, msgid, topic, qos);
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    m->mqtt_done = 0;
    m->mqtt_last_result = - 99;
    M66_Status s = atf(m, c, NULL, 0, 10000);
    if (s != M66_OK) return s;
    return mqtt_wait(m, timeout?timeout: 60000u);
}
M66_Status M66_MQTTUnsubscribe(M66_Handle *m, uint8_t id, uint16_t msgid, const char *topic, uint32_t timeout) {
    char c[200];
    if (!m || id > 5 || !topic || !safe_text(topic, 0) || strlen(topic) > 100) return M66_E_ARG;
    int k = snprintf(c, sizeof(c), "AT+QMTUNS=%u,%u,\\\"%s\\\"", id, msgid, topic);
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    m->mqtt_done = 0;
    m->mqtt_last_result = - 99;
    M66_Status s = atf(m, c, NULL, 0, 10000);
    if (s != M66_OK) return s;
    return mqtt_wait(m, timeout?timeout: 60000u);
}
M66_Status M66_MQTTPublish(M66_Handle *m, uint8_t id, uint16_t msgid, uint8_t qos, uint8_t retain, const char *topic, const uint8_t *payload, size_t len, uint32_t timeout) {
    char c[220];
    if (!m || id > 5 || qos > 2 || retain > 1 || !topic || !safe_text(topic, 0) || strlen(topic) > 100 ||(!payload && len) || len == 0 || len > 1548u) return M66_E_ARG;
    /*Fixed-length M66 syntax supports binary payloads: AT+QMTPUB=id,msgid,qos,retain,"topic",size. */
    int k = snprintf(c, sizeof(c), "AT+QMTPUB=%u,%u,%u,%u,\\\"%s\\\",%lu", id, msgid, qos, retain, topic, (unsigned long) len);
    if (k < 0 || (size_t) k >= sizeof(c)) return M66_E_ARG;
    m->mqtt_done = 0;
    m->mqtt_last_result = - 99;
    M66_Status s = wait_for_prompt(m, c, timeout?timeout: 30000u);
    if (s != M66_OK) {
        m->command_active = 0;
        return s;
    }
    s = raw_tx(m, payload, len, timeout?timeout: 30000u);
    if (s != M66_OK) {
        m->command_active = 0;
        return s;
    }
    s = wait_command_final(m, timeout?timeout: 30000u);
    if (s != M66_OK) return s;
    return mqtt_wait(m, timeout?timeout: 60000u);
}
M66_Status M66_MQTTDisconnect(M66_Handle *m, uint8_t id, uint32_t timeout) {
    char c[32];
    if (!m || id > 5) return M66_E_ARG;
    snprintf(c, sizeof(c), "AT+QMTDISC=%u", id);
    m->mqtt_done = 0;
    m->mqtt_last_result = - 99;
    M66_Status s = atf(m, c, NULL, 0, 10000);
    if (s != M66_OK) return s;
    return mqtt_wait(m, timeout?timeout: 30000u);
}
M66_Status M66_MQTTClose(M66_Handle *m, uint8_t id, uint32_t timeout) {
    char c[32];
    if (!m || id > 5) return M66_E_ARG;
    snprintf(c, sizeof(c), "AT+QMTCLOSE=%u", id);
    m->mqtt_done = 0;
    m->mqtt_last_result = - 99;
    M66_Status s = atf(m, c, NULL, 0, 10000);
    if (s != M66_OK) return s;
    return mqtt_wait(m, timeout?timeout: 30000u);
}
M66_Status M66_MQTTSetKeepAlive(M66_Handle *m, uint8_t id, uint16_t seconds) {
    char c[64];
    if (!m || id > 5 || seconds < 5 || seconds > 3600) return M66_E_ARG;
    snprintf(c, sizeof(c), "AT+QMTCFG=\"keepalive\",%u,%u", id, seconds);
    return atf(m, c, NULL, 0, 3000);
}
M66_Status M66_MQTTSetSSL(M66_Handle *m, uint8_t id, uint8_t enable, uint8_t ssl_ctx) {
    char c[64];
    if (!m || id > 5 || enable > 1 || ssl_ctx > 5) return M66_E_ARG;
    snprintf(c, sizeof(c), "AT+QMTCFG=\"SSL\",%u,%u,%u", id, enable, ssl_ctx);
    return atf(m, c, NULL, 0, 3000);
}
void M66_GetStats(const M66_Handle *m, M66_Stats *o) {
    if (m && o) *o = m->stats;
}
void M66_ClearStats(M66_Handle *m) {
    if (m) memset(& m->stats, 0, sizeof(m->stats));
}
