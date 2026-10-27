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
 *
 * On the left a little SCARA, seen from above, follows the arrows and
 * leaves a light trail while they are sent; left alone it wanders about.
 */
#include <furi.h>
#include <gui/gui.h>
#include <infrared.h>
#include <infrared_worker.h>
#include <notification/notification_messages.h>

#define SCARA_ADDRESS 0x0586
#define SCARA_TX_MIN_INTERVAL_MS 50 /* as the stock Infrared app */

/* The little arm on screen: base, link lengths (px) and frame period */
#define ARM_BASE_X 33
#define ARM_BASE_Y 54
#define ARM_L1 18
#define ARM_L2 15
#define ARM_FRAME_MS 40
#define ARM_IDLE_FRAMES 75 /* 3 s without arrows: it wanders by itself */
#define ARM_TRAIL 32

/* Angles in steps of 1/128 turn; sin * 1024 for a quarter turn */
static const int16_t sin_q[33] = {
    0,   50,  100, 150, 200, 249, 297, 345, 392, 438, 483,
    526, 569, 610, 650, 688, 724, 759, 792, 822, 851, 878,
    903, 926, 946, 964, 980, 993, 1004, 1013, 1019, 1023, 1024,
};

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
    FuriTimer* timer;
    /* the arm: joint angles, where its tip is going, the trail it left */
    int shoulder, elbow; /* elbow relative to link 1 */
    float tx, ty;
    uint32_t frame, idle;
    int8_t trail_x[ARM_TRAIL], trail_y[ARM_TRAIL];
    uint8_t trail_n, trail_head;
} ScaraIr;

static float arm_sin(int a) {
    a &= 127;
    if(a <= 32) return sin_q[a] / 1024.0f;
    if(a <= 64) return sin_q[64 - a] / 1024.0f;
    if(a <= 96) return -sin_q[a - 64] / 1024.0f;
    return -sin_q[128 - a] / 1024.0f;
}

static float arm_cos(int a) {
    return arm_sin(a + 32);
}

/* Elbow and tip on screen for the given angles (0 = right, 32 = up) */
static void arm_fk(int sh, int el, float* ex, float* ey, float* px, float* py) {
    *ex = ARM_BASE_X + ARM_L1 * arm_cos(sh);
    *ey = ARM_BASE_Y - ARM_L1 * arm_sin(sh);
    *px = *ex + ARM_L2 * arm_cos(sh + el);
    *py = *ey - ARM_L2 * arm_sin(sh + el);
}

/* Is x, y a place the tip can go: within reach and above the base */
static bool arm_can_reach(float x, float y) {
    float dx = x - ARM_BASE_X, dy = y - ARM_BASE_Y, r2 = dx * dx + dy * dy;
    return r2 >= 8 * 8 && r2 <= (ARM_L1 + ARM_L2 - 1) * (ARM_L1 + ARM_L2 - 1) &&
           y <= ARM_BASE_Y - 4;
}

/* One frame: move the target, then each joint one step toward it */
static void arm_step(ScaraIr* app) {
    static const int8_t dx[ArrowCount] = {[ArrowRight] = 1, [ArrowLeft] = -1};
    static const int8_t dy[ArrowCount] = {[ArrowUp] = -1, [ArrowDown] = 1};
    float best = 1e9f, ex, ey, px, py;
    int bs = 0, be = 0;

    app->frame++;
    if(app->sending != ArrowNone) {
        float nx = app->tx + dx[app->sending], ny = app->ty + dy[app->sending];
        if(arm_can_reach(nx, ny)) {
            app->tx = nx;
            app->ty = ny;
        }
        app->idle = 0;
    } else if(++app->idle > ARM_IDLE_FRAMES) {
        /* a slow figure eight over the table */
        app->tx = ARM_BASE_X + 17 * arm_sin(app->frame / 3);
        app->ty = ARM_BASE_Y - 20 - 6 * arm_sin(app->frame / 3 * 2);
    }

    /* the angles that put the tip nearest the target (shoulder over the
     * top half, elbow bent one way only: no flips), then one step there */
    for(int sh = 0; sh <= 64; sh++)
        for(int el = 6; el <= 58; el++) {
            arm_fk(sh, el, &ex, &ey, &px, &py);
            float d = (px - app->tx) * (px - app->tx) + (py - app->ty) * (py - app->ty);
            if(d < best) {
                best = d;
                bs = sh;
                be = el;
            }
        }
    bs = app->shoulder + (bs > app->shoulder) - (bs < app->shoulder);
    be = app->elbow + (be > app->elbow) - (be < app->elbow);
    app->shoulder = bs;
    app->elbow = be;

    /* the LED leaves a trail while an arrow is sent, which then fades */
    if(app->sending != ArrowNone) {
        arm_fk(app->shoulder, app->elbow, &ex, &ey, &px, &py);
        app->trail_head = (app->trail_head + 1) % ARM_TRAIL;
        app->trail_x[app->trail_head] = (int8_t)(px + 0.5f);
        app->trail_y[app->trail_head] = (int8_t)(py + 0.5f);
        if(app->trail_n < ARM_TRAIL) app->trail_n++;
    } else if(app->trail_n && app->frame % 2) {
        app->trail_n--;
    }
}

static void timer_callback(void* ctx) {
    ScaraIr* app = ctx;

    furi_mutex_acquire(app->mutex, FuriWaitForever);
    arm_step(app);
    furi_mutex_release(app->mutex);
    view_port_update(app->view_port);
}

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

    /* the arm on screen goes on from where its tip is now */
    float ex, ey;
    arm_fk(app->shoulder, app->elbow, &ex, &ey, &app->tx, &app->ty);
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

/* A link two pixels wide */
static void draw_link(Canvas* canvas, float x0, float y0, float x1, float y1) {
    int ax = x0 + 0.5f, ay = y0 + 0.5f, bx = x1 + 0.5f, by = y1 + 0.5f;
    canvas_draw_line(canvas, ax, ay, bx, by);
    canvas_draw_line(canvas, ax + 1, ay, bx + 1, by);
    canvas_draw_line(canvas, ax, ay + 1, bx, by + 1);
}

static void draw_arm(Canvas* canvas, ScaraIr* app) {
    float ex, ey, px, py;

    arm_fk(app->shoulder, app->elbow, &ex, &ey, &px, &py);

    /* the table, the light trail (oldest first), the base */
    for(int x = 2; x < 64; x += 3)
        canvas_draw_dot(canvas, x, ARM_BASE_Y + 7);
    for(int i = 0; i < app->trail_n; i++) {
        int j = (app->trail_head + ARM_TRAIL - i) % ARM_TRAIL;
        canvas_draw_dot(canvas, app->trail_x[j], app->trail_y[j]);
    }
    canvas_draw_rbox(canvas, ARM_BASE_X - 6, ARM_BASE_Y - 1, 13, 7, 2);

    draw_link(canvas, ARM_BASE_X, ARM_BASE_Y, ex, ey);
    draw_link(canvas, ex, ey, px, py);
    canvas_draw_disc(canvas, ARM_BASE_X, ARM_BASE_Y, 3);
    canvas_set_color(canvas, ColorWhite);
    canvas_draw_dot(canvas, ARM_BASE_X, ARM_BASE_Y);
    canvas_draw_disc(canvas, ex + 0.5f, ey + 0.5f, 2);
    canvas_set_color(canvas, ColorBlack);
    canvas_draw_circle(canvas, ex + 0.5f, ey + 0.5f, 2);

    /* the LED: lit and glowing while sending */
    if(app->sending != ArrowNone) {
        canvas_draw_disc(canvas, px + 0.5f, py + 0.5f, 2);
        if(app->frame & 2) canvas_draw_circle(canvas, px + 0.5f, py + 0.5f, 4);
    } else {
        canvas_draw_circle(canvas, px + 0.5f, py + 0.5f, 1);
    }
}

static void draw_callback(Canvas* canvas, void* ctx) {
    ScaraIr* app = ctx;

    furi_mutex_acquire(app->mutex, FuriWaitForever);
    canvas_clear(canvas);
    canvas_set_font(canvas, FontSecondary);
    canvas_draw_str(canvas, 2, 8, "SCARA IR");
    draw_arm(canvas, app);

    draw_arrow(canvas, 96, 13, CanvasDirectionBottomToTop, app->sending == ArrowUp);
    draw_arrow(canvas, 96, 50, CanvasDirectionTopToBottom, app->sending == ArrowDown);
    draw_arrow(canvas, 77, 32, CanvasDirectionRightToLeft, app->sending == ArrowLeft);
    draw_arrow(canvas, 115, 32, CanvasDirectionLeftToRight, app->sending == ArrowRight);
    furi_mutex_release(app->mutex);
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
    app->shoulder = 40;
    app->elbow = 40;
    app->tx = ARM_BASE_X;
    app->ty = ARM_BASE_Y - 22;
    app->idle = ARM_IDLE_FRAMES; /* start wandering */

    app->view_port = view_port_alloc();
    view_port_draw_callback_set(app->view_port, draw_callback, app);
    view_port_input_callback_set(app->view_port, input_callback, app);
    app->gui = furi_record_open(RECORD_GUI);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);
    app->timer = furi_timer_alloc(timer_callback, FuriTimerTypePeriodic, app);
    furi_timer_start(app->timer, furi_ms_to_ticks(ARM_FRAME_MS));

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

    furi_timer_stop(app->timer);
    furi_timer_free(app->timer);
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
