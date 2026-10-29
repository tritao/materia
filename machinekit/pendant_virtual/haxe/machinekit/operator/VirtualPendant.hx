package machinekit.operator;

import MachineKitPendant;
import haxe.io.Bytes;

/**
	The pendant firmware's own core (machinekit/pendant_core), running in the simulator: feed it the controls and the
	controller's frames, and read back the frames it would send, its LEDs and the OLED. Time is the caller's millisecond
	clock.
**/
class VirtualPendant {
  public static inline var WIDTH = 128;
  public static inline var HEIGHT = 64;
  public static inline var POWER_ON = 0;
  public static inline var POWER_DIM = 1;
  public static inline var POWER_OFF = 2;

  final owner:Ownedpendant_virtual_handle;

  public function new(nowMs:Int) {
    var created = MachineKitPendant.pendant_virtual_create(nowMs);
    check(created.status, "create");
    owner = created.out_pendant;
  }

  /** Reads the controls; true when a PENDANT_STATE frame is due (every 10 ms), to be read with `frame`. **/
  public function tick(nowMs:Int,
    wheelCounts:Int, position:Int, step:Int, keys:Int, wheelEnable:Bool, chainClosed:Bool):Bool {
    var due = MachineKitPendant.pendant_virtual_tick(
      owner.borrow(),
      nowMs,
      wheelCounts & 0xFFFF,
      position,
      step,
      keys,
      wheelEnable ? 1 : 0,
      chainClosed ? 1 : 0
    );
    if (due < 0) check(due, "tick");
    return due == 1;
  }

  /** The PENDANT_STATE payload of the last tick that was due. **/
  public function frame():Bytes {
    var result = MachineKitPendant.pendant_virtual_frame(owner.borrow());
    check(result.status, "frame");
    return result.frame;
  }

  /**
	 * Attaches the cabinet-node operator service as the pendant's controller, with the host's part played inside: an allowing
	 * state and the profile's labels. `profile`, `reasons` and `labels` are a compiled profile's records (`CompiledPendant`).
	 * From then on the screen and the LEDs follow the controller.
	 **/
  public function attachController(nowMs:Int, profile:Bytes, reasons:Array<Bytes>, labels:Array<Bytes>):Void {
    var status = MachineKitPendant.pendant_virtual_attach_controller(
      owner.borrow(),
      nowMs,
      profile,
      pack(reasons),
      pack(labels)
    );
    check(status, "attach_controller");
  }

  /** The condition mask the controller judges jogs with. **/
  public function setConditions(conditions:Int):Void {
    check(MachineKitPendant.pendant_virtual_set_conditions(owner.borrow(), conditions), "set_conditions");
  }

  /**
   * The oldest jog the controller forwarded and the host has not taken: the profile's axis index and the distance in thousandths of
   * a unit; null when none is pending.
   **/
  public function takeJog():Null<{axis:Int, distance:Int}> {
    var result = MachineKitPendant.pendant_virtual_take_jog(owner.borrow());
    check(result.status, "take_jog");
    return result.out_axis == 255 ? null : {axis: result.out_axis, distance: result.out_distance};
  }

  static function pack(records:Array<Bytes>):Bytes {
    var out = new haxe.io.BytesBuffer();
    for (record in records) {
      out.addByte(record.length);
      out.add(record);
    }
    return out.getBytes();
  }

  /** A frame from the controller; false when its payload is malformed. **/
  public function onFrame(nowMs:Int, id:Int, payload:Bytes):Bool {
    var status = MachineKitPendant.pendant_virtual_on_frame_slice(owner.borrow(),
      nowMs, id, payload, 0, payload.length);
    if (status == -3) return false;
    check(status, "on_frame");
    return true;
  }

  public function leds(nowMs:Int):Int {
    var result = MachineKitPendant.pendant_virtual_leds(owner.borrow(), nowMs);
    check(result.status, "leds");
    return result.out_leds;
  }

  public function displayPower(nowMs:Int):Int {
    var result = MachineKitPendant.pendant_virtual_display_power(owner.borrow(), nowMs);
    check(result.status, "display_power");
    return result.out_power;
  }

  /** The OLED, one byte per pixel (0 or 255), row-major from the top-left; all zero when blanked. **/
  public function pixels(nowMs:Int):Bytes {
    var result = MachineKitPendant.pendant_virtual_pixels(owner.borrow(), nowMs);
    check(result.status, "pixels");
    return result.pixels;
  }

  public function checksum(nowMs:Int):Int {
    var result = MachineKitPendant.pendant_virtual_checksum(owner.borrow(), nowMs);
    check(result.status, "checksum");
    return result.out_checksum;
  }

  public function close():Void {
    owner.close();
  }

  static function check(status:Int, what:String):Void {
    if (status < 0) throw 'Virtual pendant $what failed with status $status';
  }
}
