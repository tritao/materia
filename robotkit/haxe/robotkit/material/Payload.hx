package robotkit.material;

/** Physical payload dimensions and center of mass, expressed in metres and kilograms. */
class Payload {
  public final massKg:Float;
  public final lengthMeters:Float;
  public final widthMeters:Float;
  public final heightMeters:Float;
  public final centerOfMassForwardMeters:Float;
  public final centerOfMassLateralMeters:Float;
  public final centerOfMassAboveBaseMeters:Float;

  public function new(massKg:Float, lengthMeters:Float, widthMeters:Float,
      heightMeters:Float, centerOfMassForwardMeters:Float,
      ?centerOfMassLateralMeters:Float = 0.0,
      ?centerOfMassAboveBaseMeters:Float = 0.0) {
    for (value in [massKg, lengthMeters, widthMeters, heightMeters,
        centerOfMassForwardMeters, centerOfMassLateralMeters,
        centerOfMassAboveBaseMeters])
      if (!Math.isFinite(value)) throw "Payload values must be finite";
    if (massKg <= 0.0 || lengthMeters <= 0.0 || widthMeters <= 0.0 || heightMeters <= 0.0 ||
        centerOfMassForwardMeters < 0.0 || centerOfMassAboveBaseMeters < 0.0)
      throw "Payload mass, dimensions, and center of mass must be physically valid";
    this.massKg = massKg;
    this.lengthMeters = lengthMeters;
    this.widthMeters = widthMeters;
    this.heightMeters = heightMeters;
    this.centerOfMassForwardMeters = centerOfMassForwardMeters;
    this.centerOfMassLateralMeters = centerOfMassLateralMeters;
    this.centerOfMassAboveBaseMeters = centerOfMassAboveBaseMeters;
  }
}
