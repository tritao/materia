#ifndef PENDANT_VIRTUAL_H
#define PENDANT_VIRTUAL_H

#include <stddef.h>
#include <stdint.h>

/* The pendant firmware's hardware-independent core (machinekit/pendant_core), driven from a simulator.
 * Time is the caller's millisecond clock; it wraps at 2^32 like the firmware's. Every call returns zero for success
 * (tick: 1 when a frame is due), or a negative PV_ERROR_*. */

/* Haxeon FFI annotations are inert for ordinary C/C++ consumers. */
#if defined(__clang__)
#define PV_OUT __attribute__((annotate("hxi:out")))
#define PV_INOUT __attribute__((annotate("hxi:inout")))
#define PV_HANDLE __attribute__((annotate("hxi:handle")))
#define PV_HANDLE_DESTROY(symbol) __attribute__((annotate("hxi:handle_destroy")))
#define PV_OWNED __attribute__((annotate("hxi:owned")))
#define PV_IN_ARRAY(count) __attribute__((annotate("hxi:in_array")))
#define PV_OUT_BUFFER(size) __attribute__((annotate("hxi:out_buffer")))
#define PV_INITIAL_CAPACITY(n) __attribute__((annotate("hxi:initial_capacity=" #n)))
#else
#define PV_OUT
#define PV_INOUT
#define PV_HANDLE
#define PV_HANDLE_DESTROY(symbol)
#define PV_OWNED
#define PV_IN_ARRAY(count)
#define PV_OUT_BUFFER(size)
#define PV_INITIAL_CAPACITY(n)
#endif

#if defined(_WIN32)
#define PV_API __declspec(dllexport)
#else
#define PV_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define PENDANT_VIRTUAL_WIDTH 128
#define PENDANT_VIRTUAL_HEIGHT 64
#define PENDANT_VIRTUAL_PIXELS 8192
#define PENDANT_VIRTUAL_STATE_BYTES 8

#define PENDANT_VIRTUAL_POWER_ON 0
#define PENDANT_VIRTUAL_POWER_DIM 1
#define PENDANT_VIRTUAL_POWER_OFF 2

#define PENDANT_VIRTUAL_ERROR_HANDLE -1
#define PENDANT_VIRTUAL_ERROR_ARGUMENT -2
#define PENDANT_VIRTUAL_ERROR_FRAME -3

typedef struct pendant_virtual_handle { uint32_t id; } pendant_virtual_handle PV_HANDLE PV_HANDLE_DESTROY(pendant_virtual_destroy);

PV_API int32_t pendant_virtual_create(uint32_t now_ms, pendant_virtual_handle *out_pendant PV_OUT PV_OWNED);
PV_API void pendant_virtual_destroy(pendant_virtual_handle pendant);

/* Advances the pendant to now_ms with the controls as given. The pendant runs on its own timeline, one PENDANT_STATE frame every
 * 10 ms: the time since the last call is replayed at that period with the controls as they were then (at most five seconds), so a
 * caller that ticks irregularly, such as once per rendered frame, still gives the controller the heartbeat it needs to hold the lease.
 * position: 0 OFF, 1..11; step: 0..2 for x1, x10, x100; keys: bits 0..5 Start, Hold, Stop, F1, F2, F3; wheel_counts is the
 * free-running quadrature count. Returns 1 when a frame was due at now_ms (read it with pendant_virtual_frame), else 0. */
PV_API int32_t pendant_virtual_tick(pendant_virtual_handle pendant, uint32_t now_ms, uint16_t wheel_counts,
    uint8_t position, uint8_t step, uint8_t keys, uint8_t wheel_enable, uint8_t chain_closed);

/* The PENDANT_STATE payload (8 bytes) the last tick that returned 1 produced. With too little capacity it reports the size
 * needed in `inout_frame_size` and writes nothing. */
PV_API int32_t pendant_virtual_frame(pendant_virtual_handle pendant,
    uint8_t *frame PV_OUT_BUFFER(inout_frame_size) PV_INITIAL_CAPACITY(8), uint32_t *inout_frame_size PV_INOUT);

/* Attaches the cabinet-node operator service as the pendant's controller, with the host's part played here: an allowing state
 * every 100 ms and the profile's labels. `profile` is a compiled profile record; `reasons` and `labels` are its refusal-text and
 * label records, each preceded by one length byte. PV_ERROR_ARGUMENT when the profile or a record is invalid. After this the
 * pendant's screen and LEDs follow the controller. */
PV_API int32_t pendant_virtual_attach_controller(pendant_virtual_handle pendant, uint32_t now_ms,
    const uint8_t *profile PV_IN_ARRAY(profile_length), uint32_t profile_length,
    const uint8_t *reasons PV_IN_ARRAY(reasons_length), uint32_t reasons_length,
    const uint8_t *labels PV_IN_ARRAY(labels_length), uint32_t labels_length);
/* The condition mask the controller judges jogs with (the profile's conditions: bit 0 the gearbox, bit 3 a program running...). */
PV_API int32_t pendant_virtual_set_conditions(pendant_virtual_handle pendant, uint32_t conditions);
/* The oldest jog the controller forwarded and the host has not taken: the profile's axis index and the distance in thousandths of a
   unit. With none pending out_axis is 255 and out_distance 0. */
PV_API int32_t pendant_virtual_take_jog(pendant_virtual_handle pendant, uint8_t *out_axis PV_OUT, int32_t *out_distance PV_OUT);

/* A frame from the controller (id is the 11-bit CAN id). PV_ERROR_FRAME when the payload is malformed. */
PV_API int32_t pendant_virtual_on_frame(pendant_virtual_handle pendant, uint32_t now_ms, uint32_t id,
    const uint8_t *bytes PV_IN_ARRAY(length), uint32_t length);

/* The key LEDs, in button order. */
PV_API int32_t pendant_virtual_leds(pendant_virtual_handle pendant, uint32_t now_ms, uint8_t *out_leds PV_OUT);
/* PENDANT_VIRTUAL_POWER_*. */
PV_API int32_t pendant_virtual_display_power(pendant_virtual_handle pendant, uint32_t now_ms, uint8_t *out_power PV_OUT);
/* The OLED as one byte per pixel (0 or 255), row-major, 128 wide, top row first, PENDANT_VIRTUAL_PIXELS bytes; a blanked
 * display is all zero. With too little capacity it reports the size needed in `inout_pixels_size` and writes nothing. */
PV_API int32_t pendant_virtual_pixels(pendant_virtual_handle pendant, uint32_t now_ms,
    uint8_t *pixels PV_OUT_BUFFER(inout_pixels_size) PV_INITIAL_CAPACITY(8192), uint32_t *inout_pixels_size PV_INOUT);
/* FNV-1a over the frame as the display chip would receive it, for golden tests. */
PV_API int32_t pendant_virtual_checksum(pendant_virtual_handle pendant, uint32_t now_ms, uint32_t *out_checksum PV_OUT);

#ifdef __cplusplus
}
#endif

#endif
