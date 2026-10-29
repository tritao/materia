//! C ABI over `pendant-core`, for the simulator. The header is include/pendant_virtual.h.

use pendant_core::screen::{HEIGHT, WIDTH};
use pendant_core::{DisplayPower, Inputs, PendantCore};
use std::slice;
use std::sync::Mutex;

pub const PIXELS: usize = WIDTH * HEIGHT;

pub struct PendantVirtual {
    core: PendantCore,
    frame: [u8; 8],
}

impl PendantVirtual {
    fn pixels(&self, now_ms: u32, out: &mut [u8]) {
        if self.core.display_power(now_ms) == DisplayPower::Off {
            out.fill(0);
            return;
        }
        let frame = self.core.screen(now_ms).render();
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                out[y * WIDTH + x] = if frame.pixel(x, y) { 255 } else { 0 };
            }
        }
    }
}

const OK: i32 = 0;
const ERROR_HANDLE: i32 = -1;
const ERROR_ARGUMENT: i32 = -2;
const ERROR_FRAME: i32 = -3;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Handle {
    id: u32,
}

/// Live pendants by handle id minus one; a destroyed slot stays empty so a stale handle is refused, not reused.
static PENDANTS: Mutex<Vec<Option<PendantVirtual>>> = Mutex::new(Vec::new());

fn with<R>(handle: Handle, f: impl FnOnce(&mut PendantVirtual) -> R) -> Result<R, i32> {
    let mut pendants = PENDANTS.lock().map_err(|_| ERROR_HANDLE)?;
    let slot = (handle.id as usize).checked_sub(1).and_then(|i| pendants.get_mut(i)).and_then(|p| p.as_mut());
    slot.map(f).ok_or(ERROR_HANDLE)
}

fn code(result: Result<i32, i32>) -> i32 {
    result.unwrap_or_else(|e| e)
}

/// # Safety
/// `out_pendant` is writable.
#[no_mangle]
pub unsafe extern "C" fn pendant_virtual_create(now_ms: u32, out_pendant: *mut Handle) -> i32 {
    if out_pendant.is_null() {
        return ERROR_ARGUMENT;
    }
    let Ok(mut pendants) = PENDANTS.lock() else { return ERROR_HANDLE };
    pendants.push(Some(PendantVirtual { core: PendantCore::new(now_ms), frame: [0; 8] }));
    *out_pendant = Handle { id: pendants.len() as u32 };
    OK
}

#[no_mangle]
pub extern "C" fn pendant_virtual_destroy(pendant: Handle) {
    if let Ok(mut pendants) = PENDANTS.lock() {
        if let Some(slot) = (pendant.id as usize).checked_sub(1).and_then(|i| pendants.get_mut(i)) {
            *slot = None;
        }
    }
}

#[no_mangle]
pub extern "C" fn pendant_virtual_tick(
    pendant: Handle, now_ms: u32, wheel_counts: u16, position: u8, step: u8, keys: u8, wheel_enable: u8, chain_closed: u8,
) -> i32 {
    let inputs = Inputs { wheel_counts, position, step, keys, wheel_enable: wheel_enable != 0, chain_closed: chain_closed != 0 };
    code(with(pendant, |p| match p.core.tick(now_ms, &inputs) {
        Some(payload) => {
            p.frame = payload;
            1
        }
        None => 0,
    }))
}

/// Copies `bytes` out when `*inout_size` has room; otherwise reports the size needed.
///
/// # Safety
/// `out` has `*inout_size` writable bytes, or is null with `*inout_size` zero.
unsafe fn write_out(bytes: &[u8], out: *mut u8, inout_size: *mut u32) -> i32 {
    if inout_size.is_null() {
        return ERROR_ARGUMENT;
    }
    if out.is_null() || (*inout_size as usize) < bytes.len() {
        *inout_size = bytes.len() as u32;
        return OK;
    }
    slice::from_raw_parts_mut(out, bytes.len()).copy_from_slice(bytes);
    *inout_size = bytes.len() as u32;
    OK
}

/// # Safety
/// `frame` has `*inout_frame_size` writable bytes, or is null with it zero.
#[no_mangle]
pub unsafe extern "C" fn pendant_virtual_frame(pendant: Handle, frame: *mut u8, inout_frame_size: *mut u32) -> i32 {
    code(with(pendant, |p| write_out(&p.frame, frame, inout_frame_size)))
}

/// # Safety
/// `bytes` points at `length` readable bytes.
#[no_mangle]
pub unsafe extern "C" fn pendant_virtual_on_frame(pendant: Handle, now_ms: u32, id: u32, bytes: *const u8, length: u32) -> i32 {
    if bytes.is_null() && length != 0 {
        return ERROR_ARGUMENT;
    }
    let payload = if length == 0 { &[][..] } else { slice::from_raw_parts(bytes, length as usize) };
    code(with(pendant, |p| match p.core.on_frame(now_ms, id, payload) {
        Ok(()) => OK,
        Err(_) => ERROR_FRAME,
    }))
}

/// # Safety
/// `out_leds` is writable.
#[no_mangle]
pub unsafe extern "C" fn pendant_virtual_leds(pendant: Handle, now_ms: u32, out_leds: *mut u8) -> i32 {
    if out_leds.is_null() {
        return ERROR_ARGUMENT;
    }
    code(with(pendant, |p| {
        *out_leds = p.core.leds(now_ms);
        OK
    }))
}

/// # Safety
/// `out_power` is writable.
#[no_mangle]
pub unsafe extern "C" fn pendant_virtual_display_power(pendant: Handle, now_ms: u32, out_power: *mut u8) -> i32 {
    if out_power.is_null() {
        return ERROR_ARGUMENT;
    }
    code(with(pendant, |p| {
        *out_power = match p.core.display_power(now_ms) {
            DisplayPower::On => 0,
            DisplayPower::Dim => 1,
            DisplayPower::Off => 2,
        };
        OK
    }))
}

/// # Safety
/// `pixels` has `*inout_pixels_size` writable bytes, or is null with it zero.
#[no_mangle]
pub unsafe extern "C" fn pendant_virtual_pixels(pendant: Handle, now_ms: u32, pixels: *mut u8, inout_pixels_size: *mut u32) -> i32 {
    code(with(pendant, |p| {
        let mut frame = vec![0u8; PIXELS];
        p.pixels(now_ms, &mut frame);
        write_out(&frame, pixels, inout_pixels_size)
    }))
}

/// # Safety
/// `out_checksum` is writable.
#[no_mangle]
pub unsafe extern "C" fn pendant_virtual_checksum(pendant: Handle, now_ms: u32, out_checksum: *mut u32) -> i32 {
    if out_checksum.is_null() {
        return ERROR_ARGUMENT;
    }
    code(with(pendant, |p| {
        *out_checksum = p.core.screen(now_ms).render().checksum();
        OK
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use operator_link::{CONFIG_ID, OUTPUT_ID};

    fn create() -> Handle {
        let mut h = Handle { id: 0 };
        assert_eq!(unsafe { pendant_virtual_create(0, &mut h) }, OK);
        h
    }

    fn pixels(h: Handle, now: u32) -> Vec<u8> {
        let mut px = vec![0u8; PIXELS];
        let mut size = PIXELS as u32;
        assert_eq!(unsafe { pendant_virtual_pixels(h, now, px.as_mut_ptr(), &mut size) }, OK);
        assert_eq!(size as usize, PIXELS);
        px
    }

    #[test]
    fn pixels_follow_the_core_and_blank_when_idle() {
        let p = create();
        let mut small = 10u32;
        let mut one = [0u8; 10];
        assert_eq!(unsafe { pendant_virtual_pixels(p, 0, one.as_mut_ptr(), &mut small) }, OK);
        assert_eq!(small as usize, PIXELS, "too little capacity reports the size needed");
        assert!(pixels(p, 0).iter().any(|&b| b == 255), "NO LINK is drawn");
        let mut power = 9u8;
        unsafe { pendant_virtual_display_power(p, 0, &mut power) };
        assert_eq!(power, 0);
        assert!(pixels(p, 70_000).iter().all(|&b| b == 0), "blanked after a minute idle");
        unsafe { pendant_virtual_display_power(p, 70_000, &mut power) };
        assert_eq!(power, 2);
        pendant_virtual_destroy(p);
    }

    #[test]
    fn ticks_emit_a_frame_every_ten_ms_and_bad_frames_are_refused() {
        let p = create();
        let mut frame = [0u8; 8];
        let mut size = 8u32;
        unsafe {
            assert_eq!(pendant_virtual_tick(p, 10, 0, 3, 1, 0, 1, 1), 1);
            assert_eq!(pendant_virtual_frame(p, frame.as_mut_ptr(), &mut size), OK);
            assert_eq!(size, 8);
            assert!(frame.iter().any(|&b| b != 0), "position 3 is in the frame");
            assert_eq!(pendant_virtual_tick(p, 11, 0, 3, 1, 0, 1, 1), 0);
            assert_eq!(pendant_virtual_on_frame(p, 12, OUTPUT_ID, [1u8].as_ptr(), 1), ERROR_FRAME);
            assert_eq!(pendant_virtual_on_frame(p, 12, CONFIG_ID, [].as_ptr(), 0), ERROR_FRAME);
            let mut leds = 7u8;
            assert_eq!(pendant_virtual_leds(p, 12, &mut leds), OK);
            assert_eq!(leds, 0);
        }
        pendant_virtual_destroy(p);
    }

    #[test]
    fn a_destroyed_handle_is_refused() {
        let p = create();
        pendant_virtual_destroy(p);
        let mut leds = 0u8;
        assert_eq!(unsafe { pendant_virtual_leds(p, 0, &mut leds) }, ERROR_HANDLE);
        assert_eq!(unsafe { pendant_virtual_leds(Handle { id: 0 }, 0, &mut leds) }, ERROR_HANDLE);
    }
}
