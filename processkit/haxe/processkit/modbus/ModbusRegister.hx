package processkit.modbus;

/** One unsigned holding/input register, with engineering units = raw * scale + offset. */
class ModbusRegister {
  public final address:Int;
  public final scale:Float;
  public final offset:Float;

  public function new(address:Int, scale:Float = 1.0, offset:Float = 0.0) {
    if (address < 0 || address > 65535 || !Math.isFinite(scale) || scale <= 0.0 || !Math.isFinite(offset))
      throw "Invalid Modbus register mapping";
    this.address = address; this.scale = scale; this.offset = offset;
  }
  public function encode(value:Float):Int {
    if (!Math.isFinite(value)) throw "Modbus setpoint must be finite";
    var scaled = (value - offset) / scale;
    if (!Math.isFinite(scaled) || scaled < 0.0 || scaled > 65535.0) throw "Modbus setpoint exceeds register range";
    return Math.round(scaled);
  }
  public function decode(raw:Int):Float {
    if (raw < 0 || raw > 65535) throw "Modbus value must fit an unsigned register";
    return raw * scale + offset;
  }
}
