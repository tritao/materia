package robotkit.model;

import robotkit.model.Transmission;

class Actuator {
  public final id:String;
  /** Limits are in the actuator's effort and coordinate units, not joint units. */
  public var maxEffort:Float;
  public var maxRate:Float;
  public var transmission:Transmission;

  public function new(id:String, maxEffort:Float, maxRate:Float,
      transmission:Transmission) {
    if (id == null || StringTools.trim(id).length == 0)
      throw "Actuator ID must be non-empty";
    if (!Math.isFinite(maxEffort) || maxEffort < 0.0 ||
        !Math.isFinite(maxRate) || maxRate < 0.0)
      throw "Actuator limits must be finite and non-negative";
    if (transmission == null) throw "Actuator transmission is required";
    switch transmission {
      case SimpleTransmission(jointId, ratio, offset):
        if (jointId == null || StringTools.trim(jointId).length == 0 ||
            !Math.isFinite(ratio) || ratio == 0.0 || !Math.isFinite(offset))
          throw "Simple transmission needs a joint ID, nonzero finite ratio and finite offset";
    }
    this.id = id;
    this.maxEffort = maxEffort;
    this.maxRate = maxRate;
    this.transmission = transmission;
  }
}
