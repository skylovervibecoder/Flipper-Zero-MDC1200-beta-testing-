#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <gui/elements.h>
#include <gui/view.h>
#include <gui/view_dispatcher.h>
#include <gui/modules/submenu.h>
#include <gui/modules/text_box.h>
#include <notification/notification.h>
#include <notification/notification_messages.h>
#include <storage/storage.h>

// CORRETTO: include per la nuova API SubGhz
#include <lib/subghz/subghz_tx_rx_worker.h>
#include <lib/subghz/devices/devices.h>
#include <lib/subghz/devices/cc1101_int/cc1101_int_interconnect.h>

#include "mdc1200_codec.h"
#include "mdc1200_dsp.h"

#define TAG "Mdc1200Tool"

typedef struct {
    const char* name;
    uint32_t    freq;
} MdcChannel;

static const MdcChannel PMR_CHANNELS[] = {
    {"PMR1",  446006250}, {"PMR2",  446018750},
    {"PMR3",  446031250}, {"PMR4",  446043750},
    {"PMR5",  446056250}, {"PMR6",  446068750},
    {"PMR7",  446081250}, {"PMR8",  446093750},
};
#define PMR_CHANNEL_COUNT (sizeof(PMR_CHANNELS) / sizeof(PMR_CHANNELS[0]))

static const MdcChannel FRS_CHANNELS[] = {
    {"FRS1",  462562500}, {"FRS2",  462587500},
    {"FRS3",  462612500}, {"FRS4",  462637500},
    {"FRS5",  462662500}, {"FRS6",  462687500},
    {"FRS7",  462712500}, {"FRS8",  467562500},
    {"FRS9",  467587500}, {"FRS10", 467612500},
    {"FRS11", 467637500}, {"FRS12", 467662500},
    {"FRS13", 467687500}, {"FRS14", 467712500},
};
#define FRS_CHANNEL_COUNT (sizeof(FRS_CHANNELS) / sizeof(FRS_CHANNELS[0]))

typedef struct {
    const char* name;
    uint8_t     op;
    uint8_t     arg;
} MdcAction;

static const MdcAction ACTIONS[] = {
    {"PTT ID",        0x01, 0x00},
    {"Emergency",     0x00, 0x00},
    {"Call Alert",    0x02, 0x00},
    {"Radio Check",   0x03, 0x00},
    {"Selective Call",0x04, 0x00},
    {"Remote Monitor",0x05, 0x00},
    {"Stun (Inibisci)",  0x2B, 0x00},
    {"Unstun (Riabilita)",0x2B, 0x0C},
};
#define ACTION_COUNT (sizeof(ACTIONS) / sizeof(ACTIONS[0]))

typedef enum {
    MdcBandPmr,
    MdcBandFrs,
} MdcBand;

typedef enum {
    MdcViewMenu,
    MdcViewStatus,
} MdcView;

typedef struct {
    Gui*                  gui;
    ViewDispatcher*       view_dispatcher;
    Submenu*              submenu;
    TextBox*              text_box;
    NotificationApp*      notifications;
    SubGhzTxRxWorker*     subghz_worker;
    // CORRETTO: puntatore al device SubGhz
    const SubGhzDevice*   subghz_device;
    FuriMutex*            mutex;
    FuriString*           buffer;
    Mdc1200Dsp            dsp;

    MdcBand               band;
    uint8_t               channel_idx;
    uint16_t              unit_id;
    uint8_t               action_idx;
    bool                  worker_running;
} Mdc1200App;

#define TX_CHUNK_SIZE 60
#define SAMPLE_RATE   8000

static bool mdc1200_send_chunked(
    SubGhzTxRxWorker* worker,
    const uint8_t* data,
    size_t total_size)
{
    size_t offset = 0;
    while(offset < total_size) {
        size_t chunk = total_size - offset;
        if(chunk > TX_CHUNK_SIZE) chunk = TX_CHUNK_SIZE;
        uint8_t retries = 0;
        while(!subghz_tx_rx_worker_write(worker, (uint8_t*)(data + offset), chunk)) {
            if(++retries > 50) {
                FURI_LOG_E(TAG, "TX timeout @ %zu", offset);
                return false;
            }
            furi_delay_ms(2);
        }
        offset += chunk;
    }
    return true;
}

// CORRETTO: riavvia il worker con la frequenza corrente
static void mdc1200_restart_worker(Mdc1200App* app) {
    if(app->worker_running) {
        subghz_tx_rx_worker_stop(app->subghz_worker);
        app->worker_running = false;
    }
    uint32_t freq = (app->band == MdcBandPmr)
        ? PMR_CHANNELS[app->channel_idx].freq
        : FRS_CHANNELS[app->channel_idx].freq;
    if(subghz_tx_rx_worker_start(app->subghz_worker, app->subghz_device, freq)) {
        app->worker_running = true;
    } else {
        FURI_LOG_E(TAG, "Impossibile avviare worker su %lu Hz", freq);
    }
}

static void mdc1200_transmit(Mdc1200App* app) {
    const MdcAction* act = &ACTIONS[app->action_idx];

    uint8_t frame[MDC1200_FRAME_BYTES];
    mdc1200_build_frame(frame, act->op, act->arg, app->unit_id);

    uint8_t bits[40 + MDC1200_FRAME_BITS];
    size_t bit_count = 0;
    mdc1200_frame_to_bits(frame, bits, &bit_count);

    size_t max_samples = bit_count * (SAMPLE_RATE / MDC1200_BAUD_RATE) + 64;
    int16_t* samples = malloc(max_samples * sizeof(int16_t));
    if(!samples) {
        notification_message(app->notifications, &sequence_error);
        return;
    }
    size_t sample_count = 0;
    mdc1200_bits_to_ffsk(bits, bit_count, samples, &sample_count, SAMPLE_RATE);

    // CORRETTO: non serve fermare/riavviare il worker per TX.
    // Il worker gestisce TX e RX automaticamente.
    bool ok = mdc1200_send_chunked(
        app->subghz_worker, (uint8_t*)samples, sample_count * sizeof(int16_t));

    free(samples);

    if(ok) {
        notification_message(app->notifications, &sequence_success);
    } else {
        notification_message(app->notifications, &sequence_error);
    }
}

static void mdc1200_worker_rx_callback(void* ctx) {
    Mdc1200App* app = ctx;
    uint8_t data[128];
    size_t size = subghz_tx_rx_worker_read(app->subghz_worker, data, sizeof(data));
    if(size < 2) return;

    mdc1200_dsp_feed(&app->dsp, (int16_t*)data, size / 2);

    Mdc1200Decoded decoded;
    if(mdc1200_dsp_get_frame(&app->dsp, &decoded)) {
        notification_message(app->notifications, &sequence_blink_green_100);
        FURI_LOG_I(TAG, "RX MDC1200 ID=%u OP=0x%02X ARG=0x%02X CRC=%s",
            decoded.unit_id, decoded.op, decoded.arg,
            decoded.crc_ok ? "OK" : "ERR");
    }
}

static void mdc1200_update_status(Mdc1200App* app) {
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    const char* band_name = (app->band == MdcBandPmr) ? "PMR" : "FRS";
    const char* ch_name = (app->band == MdcBandPmr)
        ? PMR_CHANNELS[app->channel_idx].name
        : FRS_CHANNELS[app->channel_idx].name;
    uint32_t freq = (app->band == MdcBandPmr)
        ? PMR_CHANNELS[app->channel_idx].freq
        : FRS_CHANNELS[app->channel_idx].freq;
    furi_string_printf(app->buffer,
        "Canale: %s (%s)\n"
        "Freq: %lu Hz\n"
        "Unit ID: %u\n"
        "Azione: %s\n\n"
        "OK: trasmetti\n"
        "UP/DOWN: canale\n"
        "LEFT: banda\n"
        "RIGHT: +1 ID",
        ch_name, band_name, freq,
        app->unit_id,
        ACTIONS[app->action_idx].name);
    text_box_set_text(app->text_box, furi_string_get_cstr(app->buffer));
    furi_mutex_release(app->mutex);
}

static void mdc1200_submenu_callback(void* ctx, uint32_t index) {
    Mdc1200App* app = ctx;
    app->action_idx = index;
    mdc1200_update_status(app);
    view_dispatcher_switch_to_view(app->view_dispatcher, MdcViewStatus);
}

static bool mdc1200_input_callback(InputEvent* event, void* ctx) {
    Mdc1200App* app = ctx;
    bool consumed = false;
    if(event->type != InputTypeShort && event->type != InputTypeRepeat) {
        return false;
    }
    switch(event->key) {
    case InputKeyUp:
        app->channel_idx = (app->channel_idx + 1) %
            ((app->band == MdcBandPmr) ? PMR_CHANNEL_COUNT : FRS_CHANNEL_COUNT);
        mdc1200_restart_worker(app);
        mdc1200_update_status(app);
        consumed = true;
        break;
    case InputKeyDown: {
        uint8_t count = (app->band == MdcBandPmr) ? PMR_CHANNEL_COUNT : FRS_CHANNEL_COUNT;
        app->channel_idx = (app->channel_idx + count - 1) % count;
        mdc1200_restart_worker(app);
        mdc1200_update_status(app);
        consumed = true;
        break;
    }
    case InputKeyLeft:
        app->band = (app->band == MdcBandPmr) ? MdcBandFrs : MdcBandPmr;
        app->channel_idx = 0;
        mdc1200_restart_worker(app);
        mdc1200_update_status(app);
        consumed = true;
        break;
    case InputKeyRight:
        app->unit_id = (app->unit_id + 1) % 65536;
        mdc1200_update_status(app);
        consumed = true;
        break;
    case InputKeyOk:
        mdc1200_transmit(app);
        consumed = true;
        break;
    case InputKeyBack:
        view_dispatcher_switch_to_view(app->view_dispatcher, MdcViewMenu);
        consumed = true;
        break;
    default:
        break;
    }
    return consumed;
}

// CORRETTO: la callback deve restituire bool
static bool mdc1200_nav_callback(void* ctx) {
    UNUSED(ctx);
    return false; // false = chiude l'applicazione
}

int32_t mdc1200_tool_app(void* p) {
    UNUSED(p);

    Mdc1200App* app = malloc(sizeof(Mdc1200App));
    memset(app, 0, sizeof(Mdc1200App));

    app->band = MdcBandPmr;
    app->channel_idx = 0;
    app->unit_id = 1234;
    app->action_idx = 0;
    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->buffer = furi_string_alloc();
    app->notifications = furi_record_open(RECORD_NOTIFICATION);
    mdc1200_dsp_init(&app->dsp, SAMPLE_RATE);

    // CORRETTO: inizializza i device SubGhz e ottieni il device interno
    subghz_devices_init();
    app->subghz_device = subghz_devices_get_by_name(SUBGHZ_DEVICE_CC1101_INT_NAME);
    furi_assert(app->subghz_device);

    app->subghz_worker = subghz_tx_rx_worker_alloc();
    subghz_tx_rx_worker_set_callback_have_read(
        app->subghz_worker, mdc1200_worker_rx_callback, app);

    app->gui = furi_record_open(RECORD_GUI);
    app->view_dispatcher = view_dispatcher_alloc();
    view_dispatcher_set_event_callback_context(app->view_dispatcher, app);
    // CORRETTO: la callback di navigazione ora restituisce bool
    view_dispatcher_set_navigation_event_callback(
        app->view_dispatcher, mdc1200_nav_callback);

    app->submenu = submenu_alloc();
    for(size_t i = 0; i < ACTION_COUNT; i++) {
        submenu_add_item(app->submenu, ACTIONS[i].name, i,
            mdc1200_submenu_callback, app);
    }
    view_dispatcher_add_view(app->view_dispatcher, MdcViewMenu,
        submenu_get_view(app->submenu));

    app->text_box = text_box_alloc();
    text_box_set_font(app->text_box, TextBoxFontText);
    View* tb_view = text_box_get_view(app->text_box);
    view_set_input_callback(tb_view, mdc1200_input_callback);
    view_set_context(tb_view, app);
    view_dispatcher_add_view(app->view_dispatcher, MdcViewStatus, tb_view);

    view_dispatcher_attach_to_gui(app->view_dispatcher, app->gui,
        ViewDispatcherTypeFullscreen);
    view_dispatcher_switch_to_view(app->view_dispatcher, MdcViewMenu);

    // CORRETTO: avvia il worker con device e frequenza corretti
    uint32_t freq = PMR_CHANNELS[0].freq;
    if(subghz_tx_rx_worker_start(app->subghz_worker, app->subghz_device, freq)) {
        app->worker_running = true;
    } else {
        FURI_LOG_E(TAG, "Impossibile avviare il worker SubGhz");
    }

    view_dispatcher_run(app->view_dispatcher);

    if(app->worker_running) {
        subghz_tx_rx_worker_stop(app->subghz_worker);
    }
    subghz_tx_rx_worker_free(app->subghz_worker);
    // CORRETTO: deinit dei device SubGhz
    subghz_devices_deinit();

    view_dispatcher_remove_view(app->view_dispatcher, MdcViewMenu);
    view_dispatcher_remove_view(app->view_dispatcher, MdcViewStatus);
    submenu_free(app->submenu);
    text_box_free(app->text_box);
    view_dispatcher_free(app->view_dispatcher);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_NOTIFICATION);

    furi_string_free(app->buffer);
    furi_mutex_free(app->mutex);
    free(app);
    return 0;
}