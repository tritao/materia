package processkit.modbus;

/** Vendor register addresses/scales and bit assignments; no addresses belong in the driver. */
class ModbusWelderMap {
  public final unit:Int;
  public final arcAddress:Int;
  public final arcMask:Int;
  public final wire:ModbusRegister;
  public final voltage:ModbusRegister;
  public final job:Null<ModbusRegister>;
  public final statusAddress:Int;
  public final establishedMask:Int;
  public final touchMask:Int;
  public final faultMask:Int;
  public final current:ModbusRegister;
  public final measuredVoltage:ModbusRegister;
  public final power:ModbusRegister;
  /** The device expires its output lease even if TCP cannot deliver a final arc-off write. */
  public final leaseAddress:Int;
  public final leaseMs:Int;

  public function new(unit:Int, arcAddress:Int, arcMask:Int, wire:ModbusRegister, voltage:ModbusRegister,
      statusAddress:Int, establishedMask:Int, touchMask:Int, faultMask:Int, current:ModbusRegister,
      measuredVoltage:ModbusRegister, power:ModbusRegister, leaseAddress:Int, leaseMs:Int,
      job:Null<ModbusRegister> = null) {
    if (unit < 1 || unit > 247 || wire == null || voltage == null || current == null || measuredVoltage == null || power == null)
      throw "Invalid Modbus welder map";
    for (address in [arcAddress, statusAddress, leaseAddress])
      if (address < 0 || address > 65535) throw "Invalid Modbus welder address";
    for (mask in [arcMask, establishedMask, touchMask, faultMask])
      if (mask <= 0 || mask > 65535) throw "Invalid Modbus welder bit mask";
    if ((establishedMask & touchMask) != 0 || (establishedMask & faultMask) != 0 || (touchMask & faultMask) != 0)
      throw "Modbus feedback masks must be distinct";
    if (leaseMs < 20 || leaseMs > 1000) throw "Modbus welder needs a bounded device lease";
    var writes = [arcAddress, wire.address, voltage.address, leaseAddress];
    if (job != null) writes.push(job.address);
    for (i in 0...writes.length) for (j in 0...i)
      if (writes[i] == writes[j]) throw "Modbus output registers must be distinct";
    this.unit = unit; this.arcAddress = arcAddress; this.arcMask = arcMask;
    this.wire = wire; this.voltage = voltage; this.job = job; this.statusAddress = statusAddress;
    this.establishedMask = establishedMask; this.touchMask = touchMask; this.faultMask = faultMask;
    this.current = current; this.measuredVoltage = measuredVoltage; this.power = power;
    this.leaseAddress = leaseAddress; this.leaseMs = leaseMs;
  }
}
