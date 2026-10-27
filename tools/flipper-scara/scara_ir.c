// SPDX-License-Identifier: MIT
/*
 * SCARA IR - the Flipper Zero as the remote of `scara ir`.
 *
 * Each arrow sends the code of the matching button of jo's remote, so
 * `scara ir` needs no new codes.  The kernel on the BBB reports them as
 * NEC scancodes 0x86054n, i.e. extended NEC: address bytes 0x86 0x05
 * (sent in that order, so address 0x0586) and command byte n followed by
 * its inverse.  While an arrow is held the worker sends the frame and
 * then NEC repeat codes (~110 ms), like the real remote: `scara ir`
 * moves as long as they come and stops ~200 ms after the arrow is let
 * go.  Back exits.
 */
#include <furi.h>
#include <gui/gui.h>
#include <infrared.h>
#include <infrared_worker.h>
#include <notification/notification_messages.h>

#define SCARA_ADDRESS 0x0586
#define SCARA_TX_MIN_INTERVAL_MS 50 /* as the stock Infrared app */

typedef enum {
    ArrowUp,
    ArrowDown,
    ArrowRight,
    ArrowLeft,
    ArrowCount,
    ArrowNone = ArrowCount,
} Arrow;

/* Command byte of each arrow on jo's remote (scancodes 0x00860542, ...) */
static const uint8_t arrow_command[ArrowCount] = {
    [ArrowUp] = 0x42,
    [ArrowDown] = 0x43,
    [ArrowRight] = 0x15,
    [ArrowLeft] = 0x16,
};

typedef struct {
    FuriMessageQueue* queue;
    ViewPort* view_port;
    Gui* gui;
    NotificationApp* notifications;
    InfraredWorker* worker;
    FuriMutex* mutex;
    Arrow sending; /* the arrow being sent, ArrowNone when idle */
    uint32_t last_stop; /* tick of the last stop */
} ScaraIr;

static Arrow arrow_of(InputKey key) {
    switch(key) {
    case InputKeyUp:
        return ArrowUp;
    case InputKeyDown:
        return ArrowDown;
    case InputKeyRight:
        return ArrowRight;
    case InputKeyLeft:
        return ArrowLeft;
    default:
        return ArrowNone;
    }
}

static void tx_start(ScaraIr* app, Arrow arrow) {
    InfraredMessage message = {
        .protocol = InfraredProtocolNECext,
        .address = SCARA_ADDRESS,
        /* NECext sends 16 command bits: the byte, then its inverse */
        .command = arrow_command[arrow] | (uint8_t)~arrow_command[arrow] << 8,
        .repeat = false,
    };

    if(app->sending != ArrowNone) return;
    /* the worker needs a moment between two transmissions */
    while(furi_get_tick() - app->last_stop < SCARA_TX_MIN_INTERVAL_MS)
        furi_delay_ms(5);

    infrared_worker_set_decoded_signal(app->worker, &message);
    infrared_worker_tx_set_get_signal_callback(
        app->worker, infrared_worker_tx_get_signal_steady_callback, app);
    infrared_worker_tx_start(app->worker);
    notification_message(app->notifications, &sequence_blink_start_magenta);
    app->sending = arrow;
}

static void tx_stop(ScaraIr* app) {
    if(app->sending == ArrowNone) return;
    infrared_worker_tx_stop(app->worker);
    infrared_worker_tx_set_get_signal_callback(app->worker, NULL, NULL);
    notification_message(app->notifications, &sequence_blink_stop);
    app->sending = ArrowNone;
    app->last_stop = furi_get_tick();
}

/* One arrow of the d-pad, centred on cx, cy; filled while it is sent */
static void draw_arrow(Canvas* canvas, int32_t cx, int32_t cy, CanvasDirection dir, bool on) {
    if(on) {
        canvas_draw_rbox(canvas, cx - 8, cy - 8, 17, 17, 3);
        canvas_set_color(canvas, ColorWhite);
    } else {
        canvas_draw_rframe(canvas, cx - 8, cy - 8, 17, 17, 3);
    }
    switch(dir) {
    case CanvasDirectionBottomToTop:
        canvas_draw_triangle(canvas, cx, cy + 3, 9, 6, dir);
        break;
    case CanvasDirectionTopToBottom:
        canvas_draw_triangle(canvas, cx, cy - 3, 9, 6, dir);
        break;
    case CanvasDirectionLeftToRight:
        canvas_draw_triangle(canvas, cx - 3, cy, 9, 6, dir);
        break;
    case CanvasDirectionRightToLeft:
        canvas_draw_triangle(canvas, cx + 3, cy, 9, 6, dir);
        break;
    }
    canvas_set_color(canvas, ColorBlack);
}

static void draw_callback(Canvas* canvas, void* ctx) {
    ScaraIr* app = ctx;
    Arrow sending;

    furi_mutex_acquire(app->mutex, FuriWaitForever);
    sending = app->sending;
    furi_mutex_release(app->mutex);

    canvas_clear(canvas);
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 2, 12, "SCARA IR");
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 2, 26, "Tieni premuta");
    canvas_draw_str(canvas, 2, 36, "una freccia");
    canvas_draw_str(canvas, 2, 50, sending == ArrowNone ? "Fermo" : "Invio...");
    canvas_draw_str(canvas, 2, 62, "Indietro: esci");

    draw_arrow(canvas, 96, 13, CanvasDirectionBottomToTop, sending == ArrowUp);
    draw_arrow(canvas, 96, 50, CanvasDirectionTopToBottom, sending == ArrowDown);
    draw_arrow(canvas, 77, 32, CanvasDirectionRightToLeft, sending == ArrowLeft);
    draw_arrow(canvas, 115, 32, CanvasDirectionLeftToRight, sending == ArrowRight);
}

static void input_callback(InputEvent* event, void* ctx) {
    ScaraIr* app = ctx;
    furi_message_queue_put(app->queue, event, FuriWaitForever);
}

int32_t scara_ir_app(void* p) {
    UNUSED(p);
    ScaraIr* app = malloc(sizeof(ScaraIr));
    InputEvent event;
    bool running = true;

    app->queue = furi_message_queue_alloc(8, sizeof(InputEvent));
    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->worker = infrared_worker_alloc();
    app->notifications = furi_record_open(RECORD_NOTIFICATION);
    app->sending = ArrowNone;
    app->last_stop = furi_get_tick() - SCARA_TX_MIN_INTERVAL_MS;

    app->view_port = view_port_alloc();
    view_port_draw_callback_set(app->view_port, draw_callback, app);
    view_port_input_callback_set(app->view_port, input_callback, app);
    app->gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);

    while(running) {
        if(furi_message_queue_get(app->queue, &event, FuriWaitForever) != FuriStatusOk) continue;

        Arrow arrow = arrow_of(event.key);
        furi_mutex_acquire(app->mutex, FuriWaitForever);
        if(event.key == InputKeyBack) {
            if(event.type == InputTypeShort || event.type == InputTypeLong) running = false;
        } else if(arrow != ArrowNone) {
            /* one arrow at a time: the others wait for it to be let go */
            if(event.type == InputTypePress)
                tx_start(app, arrow);
            else if(event.type == InputTypeRelease && app->sending == arrow)
                tx_stop(app);
        }
        furi_mutex_release(app->mutex);
        view_port_update(app->view_port);
    }

    tx_stop(app);
    gui_remove_view_port(app->gui, app->view_port);
    view_port_free(app->view_port);
    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_NOTIFICATION);
    infrared_worker_free(app->worker);
    furi_mutex_free(app->mutex);
    furi_message_queue_free(app->queue);
    free(app);
    return 0;
}
