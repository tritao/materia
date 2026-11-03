package machinekit.motion;

/**
 * What a part needs from the DC bus that feeds it: the voltage band it works in and the power it draws. A
 * drive with its amplifier built in states this itself, where a separate `MotorDriver` carries it otherwise. All
 * values are DC bus quantities at the part's `port`.
 */
typedef DcBusRequirement = {
  var nominalVolts:Float;
  var minimumVolts:Float;
  var maximumVolts:Float;
  /** Continuous electrical power at the rated operating point, W. */
  var ratedPower:Float;
  /** True when braking energy returns to the bus, so the source must absorb it (`ElectricalSupplyRating.regenerativePower`). */
  var regenerates:Bool;
}

interface DcBusConsumer {
  function busRequirement(port:String):DcBusRequirement;
}
