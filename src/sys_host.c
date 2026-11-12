/*
 * This file is part of mod-system-control.
 */

#include "sys_host.h"
#include "cli.h"
#include "reply.h"
#include "serial_rw.h"

#include "../mod-controller-proto/mod-protocol.h"

#define SERVER_MODE
#include "sys_host_impl.h"

#include <pthread.h>
#include <stdlib.h>

static volatile bool sys_host_thread_running = false;
static volatile bool sys_host_values_changed = false;
static int sys_host_shmfd;
static sys_serial_shm_data* sys_host_data;
static pthread_t sys_host_thread;
static int sys_host_has_msgs;
static bool s_debug;

// compressor state
static int compressor_mode = 1;
static float compressor_release = 100.0f;
static float pedalboard_gain = 0.0f;

// noise gate state
static int noisegate_channel = 0;
static float noisegate_decay = 10.0f;
static float noisegate_threshold = -60.0f;

#if defined(_MOD_DEVICE_DUOX)
 #define HMI_NUM_PAGES 6
 #define HMI_NUM_SUBPAGES 1
 #define HMI_NUM_ACTUATORS 14
#elif defined(_MOD_DEVICE_DWARF)
 #define HMI_NUM_PAGES 8
 #define HMI_NUM_SUBPAGES 3
 #define HMI_NUM_ACTUATORS 6
#else
 // fallback, so just it builds for local testing
 #define HMI_NUM_PAGES 1
 #define HMI_NUM_SUBPAGES 1
 #define HMI_NUM_ACTUATORS 2
#endif

// page cache handling
typedef struct HMI_CACHE_T {
    // values match mod-host message size
    char led_blink[32];
    char led_brightness[32];
    char label[24];
    char value[24];
    char unit[24];
    char indicator[32];
} hmi_cache_t;
static hmi_cache_t* hmi_cache[HMI_NUM_PAGES * HMI_NUM_SUBPAGES * HMI_NUM_ACTUATORS];
static int hmi_page = 0;
static int hmi_subpage = 0;
static int hmi_page_or_subpage_changed = 0;
static bool hmi_io_values_requested = false;

static bool read_host_values(void)
{
    char buf[0xff];
    if (! read_file(buf, "/data/audioproc.txt", s_debug))
        return false;

    int cmode = -1, ngchannel = -1;
    float crelease = 0, pgain = 0, ngdecay = 0, ngthreshold = 0;
    sscanf(buf, "%i\n%f\n%f\n%i\n%f\n%f\n",
           &cmode, &crelease, &pgain, &ngchannel, &ngdecay, &ngthreshold);

    // bail out if any value is invalid
    if (cmode < 0 || cmode > 4)
        return false;
    if (crelease < 50.0f || crelease > 500.0f)
        return false;
    if (pgain < -30.0f || pgain > 20.0f)
        return false;
    if (ngchannel < 0 || ngchannel > 3)
        return false;
    if (ngdecay < 1.0f || ngdecay > 500.0f)
        return false;
    if (ngthreshold < -70.0f || ngthreshold > -10.0f)
        return false;

    if (s_debug)
    {
        printf("%s success, values: %i, %f, %f, %i, %f, %f\n",
               __func__, cmode, crelease, pgain, ngchannel, ngdecay, ngthreshold);
        fflush(stdout);
    }

    // all good!
    compressor_mode = cmode;
    compressor_release = crelease;
    noisegate_channel = ngchannel;
    noisegate_decay = ngdecay;
    noisegate_threshold = ngthreshold;
    pedalboard_gain = pgain;
    return true;
}

static void write_host_values(void)
{
    const int cmode = compressor_mode;
    const float crelease = compressor_release;
    const float pgain = pedalboard_gain;
    const int ngchannel = noisegate_channel;
    const float ngdecay = noisegate_decay;
    const float ngthreshold = noisegate_threshold;

    char buf[0xff];
    snprintf(buf, sizeof(buf), "%i\n%f\n%f\n%i\n%f\n%f\n",
             cmode, crelease, pgain, ngchannel, ngdecay, ngthreshold);
    buf[sizeof(buf)-1] = '\0';
    write_file(buf, "/data/audioproc.txt", s_debug);
}

static void* sys_host_thread_run(void* const arg)
{
    while (sys_host_thread_running)
    {
        if (sys_host_values_changed)
        {
            write_host_values();
            sys_host_values_changed = false;
        }

        if (sem_timedwait_secs(&sys_host_data->server.sem, 5))
            continue;

        if (! sys_host_thread_running)
            break;

        sys_host_has_msgs = 1;
    }

    return NULL;

    // unused
    (void)arg;
}

static bool hmi_command_cache_add(const uint8_t page,
                                  uint8_t subpage,
                                  const sys_serial_event_type etype,
                                  char msg[SYS_SERIAL_SHM_DATA_SIZE])
{
    if (page >= HMI_NUM_PAGES)
        return false;
    if (subpage >= HMI_NUM_SUBPAGES)
        return false;

    char actuator[8];
    memset(actuator, 0, sizeof(actuator));
    for (uint8_t i=0; i<sizeof(actuator) && msg[i] != '\0'; ++i)
    {
        if ((actuator[i] = msg[i]) == ' ')
        {
            actuator[i] = '\0';
            break;
        }
    }

    // sanity check
    for (uint8_t i=0; i < sizeof(actuator) && actuator[i] != '\0'; ++i)
    {
        if (actuator[i] < '0' || actuator[i] > '9')
        {
            if (s_debug)
            {
                printf("%s: invalid initial byte at %d %d:%c, args were: %u, %u, %02x:%s, '%s'\n",
                       __func__, i, actuator[i], actuator[i],
                       page, subpage, etype, sys_serial_event_type_to_str(etype), msg);
                fflush(stdout);
            }
            return false;
        }
    }

    if (actuator[sizeof(actuator)-1] != '\0')
    {
        if (s_debug)
        {
            printf("%s: invalid last byte %d:%c, args were: %u, %u, %02x:%s, '%s'\n",
                   __func__, actuator[sizeof(actuator)-1], actuator[sizeof(actuator)-1],
                   page, subpage, etype, sys_serial_event_type_to_str(etype), msg);
            fflush(stdout);
        }
        return false;
    }

    const int actuatorId = atoi(actuator);

    if (actuatorId >= HMI_NUM_ACTUATORS)
    {
        if (s_debug)
        {
            printf("%s: out of bounds actuatorId %d\n", __func__, actuatorId);
            fflush(stdout);
        }
        return false;
    }

    bool matching_subpage;
#ifdef _MOD_DEVICE_DWARF
    // special exception for dwarf
    if (actuatorId >= 3)
    {
        subpage = 0;
        matching_subpage = true;
    }
    else
#endif
    {
        matching_subpage = subpage == hmi_subpage;
    }

    const size_t index = page * HMI_NUM_SUBPAGES * HMI_NUM_ACTUATORS + subpage * HMI_NUM_ACTUATORS + actuatorId;
    hmi_cache_t* cache = hmi_cache[index];

    if (cache == NULL)
    {
        hmi_cache[index] = cache = calloc(1, sizeof(hmi_cache_t));

        if (cache == NULL)
        {
            if (s_debug) {
                printf("%s: failed to allocate memory\n", __func__);
                fflush(stdout);
            }
            return false;
        }
    }

    const bool match_pages = page == hmi_page && matching_subpage;
    bool match_content = false;

    switch (etype)
    {
    case sys_serial_event_type_led_blink:
        if (strncmp(cache->led_blink, msg, sizeof(cache->led_blink)-1) != 0)
            strncpy(cache->led_blink, msg, sizeof(cache->led_blink)-1);
        else
            match_content = true;
        break;
    case sys_serial_event_type_led_brightness:
        if (strncmp(cache->led_brightness, msg, sizeof(cache->led_brightness)-1) != 0)
            strncpy(cache->led_brightness, msg, sizeof(cache->led_brightness)-1);
        else
            match_content = true;
        break;
    case sys_serial_event_type_name:
        if (strncmp(cache->label, msg, sizeof(cache->label)-1) != 0)
            strncpy(cache->label, msg, sizeof(cache->label)-1);
        else
            match_content = true;
        break;
    case sys_serial_event_type_unit:
        if (strncmp(cache->unit, msg, sizeof(cache->unit)-1) != 0)
            strncpy(cache->unit, msg, sizeof(cache->unit)-1);
        else
            match_content = true;
        break;
    case sys_serial_event_type_value:
        if (strncmp(cache->value, msg, sizeof(cache->value)-1) != 0)
            strncpy(cache->value, msg, sizeof(cache->value)-1);
        else
            match_content = true;
        break;
    case sys_serial_event_type_widget_indicator:
        if (strncmp(cache->indicator, msg, sizeof(cache->indicator)-1) != 0)
            strncpy(cache->indicator, msg, sizeof(cache->indicator)-1);
        else
            match_content = true;
        break;
    case sys_serial_event_type_popup:
        // there is no cache for popups, and we allow them to be repeated
        break;
    default:
        break;
    }

    if (s_debug)
    {
        if (match_pages)
        {
            if (match_content)
                printf("%s: page and subpage %u,%u are currently active but new contents match old, ignoring\n",
                       __func__, page, subpage);
            else
                printf("%s: page and subpage %u,%u are currently active, will trigger HMI now with new content\n",
                       __func__, page, subpage);
        }
        else
        {
            printf("%s: page and subpage %u,%u are NOT currently active, will only cache\n",
                   __func__, page, subpage);
        }
        fflush(stdout);
    }

    return match_pages && !match_content;
}

static void hmi_command_cache_remove(const uint8_t page, uint8_t subpage, char msg[SYS_SERIAL_SHM_DATA_SIZE])
{
    if (s_debug)
    {
        printf("%s called with values: %u, %u, '%s'\n", __func__, page, subpage, msg);
        fflush(stdout);
    }
    if (page >= HMI_NUM_PAGES)
        return;
    if (subpage >= HMI_NUM_SUBPAGES)
        return;

    char actuator[8];
    memset(actuator, 0, sizeof(actuator));
    for (uint8_t i=0; i<sizeof(actuator) && msg[i] != '\0'; ++i)
    {
        if ((actuator[i] = msg[i]) == ' ')
        {
            actuator[i] = '\0';
            break;
        }
    }

    // sanity check
    for (uint8_t i=0; i < sizeof(actuator) && actuator[i] != '\0'; ++i)
    {
        if (actuator[i] < '0' || actuator[i] > '9')
        {
            if (s_debug)
            {
                printf("%s: invalid initial byte at %d %d:%c, args were: %u, %u,'%s'\n",
                       __func__, i, actuator[i], actuator[i], page, subpage, msg);
                fflush(stdout);
            }
            return;
        }
    }

    if (actuator[sizeof(actuator)-1] != '\0')
    {
        if (s_debug)
        {
            printf("%s: invalid last byte %d:%c, args were: %u, %u, '%s'\n",
                   __func__, actuator[sizeof(actuator)-1], actuator[sizeof(actuator)-1], page, subpage, msg);
            fflush(stdout);
        }
        return;
    }

    const int actuatorId = atoi(actuator);

    if (actuatorId >= HMI_NUM_ACTUATORS)
    {
        if (s_debug)
        {
            printf("%s: out of bounds actuatorId %d\n", __func__, actuatorId);
            fflush(stdout);
        }
        return;
    }

#ifdef _MOD_DEVICE_DWARF
    // special exception for dwarf
    if (actuatorId >= 3)
        subpage = 0;
#endif

    const size_t index = page * HMI_NUM_SUBPAGES * HMI_NUM_ACTUATORS + subpage * HMI_NUM_ACTUATORS + actuatorId;

    if (s_debug)
    {
        printf("%s has index %lu and cache pointer %p\n", __func__, index, hmi_cache[index]);
        fflush(stdout);
    }

    if (hmi_cache[index] != NULL)
    {
        free(hmi_cache[index]);
        hmi_cache[index] = NULL;
    }
}

// Whether a message is one of the protocol errors of the HMI ("not found" for a command it does not
// know, a firmware without the HMI widgets say). Without arguments the HMI cuts a message on the system
// line to _CMD_SYS_LENGTH characters ("not fou"): a prefix of the error counts.
static bool is_hmi_protocol_error(const char* const buf)
{
    static const char* const errors[] = {
        RESP_ERR_COMMAND_NOT_FOUND, RESP_ERR_MANY_ARGUMENTS, RESP_ERR_FEW_ARGUMENTS, RESP_ERR_INVALID_ARGUMENT
    };
    const size_t len = strlen(buf);

    for (size_t i = 0; len != 0 && i < sizeof(errors) / sizeof(errors[0]); ++i)
        if (strncmp(buf, errors[i], len) == 0)
            return true;

    return false;
}

// Waits for the HMI to answer a command sent to it. The HMI may send a command of its own in the
// meantime, a page change say, and it may be blocked until that one is answered: such a command is
// handled right here, as the main loop would, instead of being taken for the response and dropped.
// A protocol error is the answer too, reported once per command: waiting on would stall every command
// the HMI does not know, and mod-host behind them.
static void wait_for_hmi_response(struct sp_port* const serialport, const char* const sys_cmd)
{
    char buf[0xff];

    for (;;)
    {
        // generous: the HMI can be busy redrawing, and a response missed here comes late into the main loop
        const int ret = serial_read_message(serialport, buf, 500);

        if (ret == SP_READ_ERROR_NO_DATA || ret == SP_READ_ERROR_IO)
        {
            fprintf(stderr, "%s: no response from the HMI\n", __func__);
            return;
        }

        if (ret < 0)
            continue;

        if (buf[0] == 'r' && buf[1] == ' ')
            return;

        if (is_hmi_protocol_error(buf))
        {
            static char reported[16][_CMD_SYS_LENGTH + 1];
            static unsigned int reported_count;
            bool known = false;

            for (unsigned int i = 0; i < reported_count && ! known; ++i)
                known = strncmp(reported[i], sys_cmd, _CMD_SYS_LENGTH) == 0;

            if (! known)
            {
                fprintf(stderr, "%s: the HMI answers '%s' to %.*s\n", __func__, buf, _CMD_SYS_LENGTH, sys_cmd);
                if (reported_count < sizeof(reported) / sizeof(reported[0]))
                    snprintf(reported[reported_count++], sizeof(reported[0]), "%.*s", _CMD_SYS_LENGTH, sys_cmd);
            }
            return;
        }

        if (strncmp(buf, _CMD_SYS_PREFIX, strlen(_CMD_SYS_PREFIX)) == 0)
        {
            if (s_debug)
            {
                fprintf(stdout, "handling '%s' from the HMI while waiting for a response\n", buf);
                fflush(stdout);
            }
            parse_and_reply_to_message(serialport, buf, s_debug);
            continue;
        }

        fprintf(stderr, "%s: ignoring '%s' while waiting for a response\n", __func__, buf);
    }
}

static void send_command_to_hmi(struct sp_port* const serialport, const char* const sys_cmd,
                                char msg[SYS_SERIAL_SHM_DATA_SIZE], const bool quoted)
{
    const uint8_t len = strlen(msg);
    const size_t msg_offset = _CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 2 /* spaces */ + (quoted ? 1 : 0);

    // move value string
    memmove(msg + msg_offset, msg, len+1);

    // place command at start of msg
    memcpy(msg, sys_cmd, _CMD_SYS_LENGTH);

    // store msg size
    snprintf(msg + (_CMD_SYS_LENGTH + 1), _CMD_SYS_DATA_LENGTH + 1, "%02x", len);

    // spaces
    msg[_CMD_SYS_LENGTH] = ' ';
    msg[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 1] = ' ';

    // quotes, if needed
    if (quoted)
    {
        msg[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 2] = msg[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 3];
        msg[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 3] = ' ';
        msg[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + 4] = '"';
        msg[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + len + 3] = '"';
        msg[_CMD_SYS_LENGTH + _CMD_SYS_DATA_LENGTH + len + 4] = '\0';
    }

    if (s_debug)
    {
        fprintf(stdout, "send_command_to_hmi '%s'\n", msg);
        fflush(stdout);
    }

    // write message
    write_or_close(serialport, msg);

    // response
    wait_for_hmi_response(serialport, sys_cmd);
}

// The 2 hex digits of the sys data length cap each message to the HMI at 255 bytes, far less than a
// widget bitmap can take, so it goes out in bands of whole rows, each a bitmap command of its own.
// Bands are sent in order and each replaces only its own rows, so the result is the same.
static void send_widget_bitmap_to_hmi(struct sp_port* const serialport, const char* const msg)
{
    int area_id, x, y, width, height, offset = 0;

    if (sscanf(msg, "%i %i %i %i %i %n", &area_id, &x, &y, &width, &height, &offset) != 5 || offset == 0)
    {
        fprintf(stderr, "%s: invalid message '%s'\n", __func__, msg);
        return;
    }

    const char* const rows = msg + offset;
    const size_t row_len = (size_t)((width + 7) / 8) * 2;

    if (width <= 0 || height <= 0 || strlen(rows) < row_len * (size_t)height)
    {
        fprintf(stderr, "%s: invalid bitmap '%s'\n", __func__, msg);
        return;
    }

    char band[SYS_SERIAL_SHM_DATA_SIZE];

    for (int row = 0; row < height;)
    {
        // the header with the worst case band height, to know how many rows fit after it
        const int header_len = snprintf(band, sizeof(band), "%i %i %i %i %i ", area_id, x, y + row, width, height);
        const int band_rows = (int)((0xff - header_len) / row_len);

        if (band_rows <= 0)
        {
            fprintf(stderr, "%s: bitmap rows too wide '%s'\n", __func__, msg);
            return;
        }

        const int count = band_rows < height - row ? band_rows : height - row;
        const int len = snprintf(band, sizeof(band), "%i %i %i %i %i ", area_id, x, y + row, width, count);

        memcpy(band + len, rows + row_len * (size_t)row, row_len * (size_t)count);
        band[len + row_len * (size_t)count] = '\0';

        send_command_to_hmi(serialport, CMD_SYS_WIDGET_BITMAP, band, false);

        row += count;
    }
}

static void send_command_to_host(const sys_serial_event_type etype, const char* const value)
{
    if (sys_host_data == NULL)
        return;

    sys_serial_shm_data_channel* const data = &sys_host_data->client;

    if (! sys_serial_write(data, etype, value))
        return;

    if (s_debug)
    {
        fprintf(stdout, "send_command_to_host %02x:%s '%s'\n", etype, sys_serial_event_type_to_str(etype), value);
        fflush(stdout);
    }

    sem_post(&data->sem);
}

static void send_command_to_host_int(const sys_serial_event_type etype, const int value)
{
    char str[24];
    snprintf(str, sizeof(str), "%i", value);
    str[sizeof(str)-1] = '\0';
    send_command_to_host(etype, str);
}

static void send_command_to_host_float(const sys_serial_event_type etype, const float value)
{
    char str[32];
    snprintf(str, sizeof(str), "%f", value);
    str[sizeof(str)-1] = '\0';
    send_command_to_host(etype, str);
}

static void sys_host_resend_hmi(struct sp_port* const serialport)
{
    size_t index;
    hmi_cache_t* cache;
    char msg[SYS_SERIAL_SHM_DATA_SIZE];
    int subpage;

    for (int i=0; i<HMI_NUM_ACTUATORS; ++i)
    {
#ifdef _MOD_DEVICE_DWARF
        // special exception for dwarf
        if (i >= 3)
            subpage = 0;
        else
#endif
            subpage = hmi_subpage;

        index = hmi_page * HMI_NUM_SUBPAGES * HMI_NUM_ACTUATORS + subpage * HMI_NUM_ACTUATORS + i;
        cache = hmi_cache[index];

        if (cache == NULL)
            continue;

        printf("%s: found cache with index %lu %p; page and subpage %u,%u\n",
               __func__, index, cache, hmi_page, subpage);

        if (cache->led_blink[0] != '\0')
        {
            memcpy(msg, cache->led_blink, sizeof(cache->led_blink));
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_LED_BLINK, msg, false);
        }
        if (cache->led_brightness[0] != '\0')
        {
            memcpy(msg, cache->led_brightness, sizeof(cache->led_brightness));
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_LED_BRIGHTNESS, msg, false);
        }
        if (cache->label[0] != '\0')
        {
            memcpy(msg, cache->label, sizeof(cache->label));
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_NAME, msg, true);
        }
        if (cache->unit[0] != '\0')
        {
            memcpy(msg, cache->unit, sizeof(cache->unit));
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_UNIT, msg, true);
        }
        if (cache->value[0] != '\0')
        {
            memcpy(msg, cache->value, sizeof(cache->value));
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_VALUE, msg, true);
        }
        if (cache->indicator[0] != '\0')
        {
            memcpy(msg, cache->indicator, sizeof(cache->indicator));
            send_command_to_hmi(serialport, CMD_SYS_CHANGE_WIDGET_INDICATOR, msg, false);
        }
    }

    if (s_debug)
    {
        fputs("\n", stdout);
        fflush(stdout);
    }
}

static void sys_host_reset(const uint8_t page, const uint8_t subpage)
{
    hmi_page = page;
    hmi_subpage = subpage;
    // pedalboard_gain = 0.0f;

    hmi_cache_t* cache;
    for (int i=0; i<HMI_NUM_PAGES * HMI_NUM_SUBPAGES * HMI_NUM_ACTUATORS; ++i)
    {
        cache = hmi_cache[i];

        if (cache == NULL)
            continue;

        free(hmi_cache[i]);
        hmi_cache[i] = NULL;
    }
}

void sys_host_setup(const bool debug)
{
    s_debug = debug;

    if (! sys_serial_open(&sys_host_shmfd, &sys_host_data))
    {
        fprintf(stderr, "sys_host shared memory failed\n");
        return;
    }

    read_host_values();
    sys_host_thread_running = true;
    pthread_create(&sys_host_thread, NULL, sys_host_thread_run, NULL);
}

void sys_host_process(struct sp_port* const serialport)
{
    if (sys_host_data == NULL)
        return;

    if (hmi_page_or_subpage_changed)
    {
        // NOTE this is racy, workaround for mod-ui side handling messages slower than us
        if (++hmi_page_or_subpage_changed == 10)
        {
            hmi_page_or_subpage_changed = 0;
            sys_host_resend_hmi(serialport);
        }
    }

    if (hmi_io_values_requested)
    {
        hmi_io_values_requested = false;
        send_command_to_host_int(sys_serial_event_type_compressor_mode, compressor_mode);
        send_command_to_host_float(sys_serial_event_type_compressor_release, compressor_release);
        send_command_to_host_int(sys_serial_event_type_noisegate_channel, noisegate_channel);
        send_command_to_host_float(sys_serial_event_type_noisegate_decay, noisegate_decay);
        send_command_to_host_float(sys_serial_event_type_noisegate_threshold, noisegate_threshold);
        send_command_to_host_float(sys_serial_event_type_pedalboard_gain, pedalboard_gain);

        if (s_debug)
        {
            fputs("\n", stdout);
            fflush(stdout);
        }
    }

    if (! __sync_bool_compare_and_swap(&sys_host_has_msgs, 1, 0))
        return;

    sys_serial_shm_data_channel* const data = &sys_host_data->server;

    sys_serial_event_type etype;
    uint8_t page, subpage;
    char msg[SYS_SERIAL_SHM_DATA_SIZE];

    while (data->head != data->tail)
    {
        if (! sys_serial_read(data, &etype, &page, &subpage, msg))
            continue;

        if (s_debug)
        {
            fprintf(stdout, "Received message from host %u %u %02x:%s '%s'\n",
                    page, subpage, etype, sys_serial_event_type_to_str(etype), msg);
            fflush(stdout);
        }

        switch (etype)
        {
        case sys_serial_event_type_special_req:
            if (strcmp(msg, "restart") == 0)
            {
                hmi_io_values_requested = true;
                sys_host_reset(0, 0);
            }
            else if (strcmp(msg, "pages") == 0)
            {
                sys_host_reset(page, subpage);
            }
            break;
        case sys_serial_event_type_unassign:
            hmi_command_cache_remove(page, subpage, msg);
            break;
        case sys_serial_event_type_led_blink:
            if (hmi_command_cache_add(page, subpage, etype, msg))
                send_command_to_hmi(serialport, CMD_SYS_CHANGE_LED_BLINK, msg, false);
            break;
        case sys_serial_event_type_led_brightness:
            if (hmi_command_cache_add(page, subpage, etype, msg))
                send_command_to_hmi(serialport, CMD_SYS_CHANGE_LED_BRIGHTNESS, msg, false);
            break;
        case sys_serial_event_type_name:
            if (hmi_command_cache_add(page, subpage, etype, msg))
                send_command_to_hmi(serialport, CMD_SYS_CHANGE_NAME, msg, true);
            break;
        case sys_serial_event_type_unit:
            if (hmi_command_cache_add(page, subpage, etype, msg))
                send_command_to_hmi(serialport, CMD_SYS_CHANGE_UNIT, msg, true);
            break;
        case sys_serial_event_type_value:
            if (hmi_command_cache_add(page, subpage, etype, msg))
                send_command_to_hmi(serialport, CMD_SYS_CHANGE_VALUE, msg, true);
            break;
        case sys_serial_event_type_widget_indicator:
            if (hmi_command_cache_add(page, subpage, etype, msg))
                send_command_to_hmi(serialport, CMD_SYS_CHANGE_WIDGET_INDICATOR, msg, false);
            break;
        case sys_serial_event_type_popup:
            if (hmi_command_cache_add(page, subpage, etype, msg))
                send_command_to_hmi(serialport, CMD_SYS_LAUNCH_POPUP, msg, false);
            break;
        // widgets are not cached: the plugin redraws its whole area every time it comes into view
        case sys_serial_event_type_widget_bitmap:
            send_widget_bitmap_to_hmi(serialport, msg);
            break;
        case sys_serial_event_type_widget_clear:
            send_command_to_hmi(serialport, CMD_SYS_WIDGET_CLEAR, msg, false);
            break;
        default:
            break;
        }

        if (s_debug)
        {
            fputs("\n", stdout);
            fflush(stdout);
        }
    }
}

void sys_host_destroy(void)
{
    if (sys_host_data == NULL)
        return;

    sys_host_thread_running = false;
    sem_post(&sys_host_data->server.sem);
    pthread_join(sys_host_thread, NULL);

    sys_serial_close(sys_host_shmfd, sys_host_data);
}

int sys_host_get_compressor_mode(void)
{
    return compressor_mode;
}

float sys_host_get_compressor_release(void)
{
    return compressor_release;
}

int sys_host_get_noisegate_channel(void)
{
    return noisegate_channel;
}

float sys_host_get_noisegate_decay(void)
{
    return noisegate_decay;
}

float sys_host_get_noisegate_threshold(void)
{
    return noisegate_threshold;
}

float sys_host_get_pedalboard_gain(void)
{
    return pedalboard_gain;
}

void sys_host_set_compressor_mode(const int mode)
{
    compressor_mode = mode;
    sys_host_values_changed = true;
    send_command_to_host_int(sys_serial_event_type_compressor_mode, mode);
}

void sys_host_set_compressor_release(const float value)
{
    compressor_release = value;
    sys_host_values_changed = true;
    send_command_to_host_float(sys_serial_event_type_compressor_release, value);
}

void sys_host_set_noisegate_channel(const int channel)
{
    noisegate_channel = channel;
    sys_host_values_changed = true;
    send_command_to_host_int(sys_serial_event_type_noisegate_channel, channel);
}

void sys_host_set_noisegate_decay(const float value)
{
    noisegate_decay = value;
    sys_host_values_changed = true;
    send_command_to_host_float(sys_serial_event_type_noisegate_decay, value);
}

void sys_host_set_noisegate_threshold(const float value)
{
    noisegate_threshold = value;
    sys_host_values_changed = true;
    send_command_to_host_float(sys_serial_event_type_noisegate_threshold, value);
}

void sys_host_set_pedalboard_gain(const float value)
{
    pedalboard_gain = value;
    sys_host_values_changed = true;
    send_command_to_host_float(sys_serial_event_type_pedalboard_gain, value);
}

void sys_host_set_hmi_page(const int page)
{
    if (hmi_page == page)
    {
        if (s_debug)
        {
            fprintf(stdout, "active page remains at %d\n", page);
            fflush(stdout);
        }
        return;
    }

    if (s_debug)
    {
        fprintf(stdout, "active page changed to %d, plus subpage resets to 0\n", page);
        fflush(stdout);
    }

    hmi_page = page;
    hmi_subpage = 0;
    hmi_page_or_subpage_changed = 1;
}

void sys_host_set_hmi_subpage(const int subpage)
{
    if (hmi_subpage == subpage)
    {
        if (s_debug)
        {
            fprintf(stdout, "active subpage remains at %d\n", subpage);
            fflush(stdout);
        }
        return;
    }

    if (s_debug)
    {
        fprintf(stdout, "active subpage changed to %d\n", subpage);
        fflush(stdout);
    }

    hmi_subpage = subpage;
    hmi_page_or_subpage_changed = 1;
}
