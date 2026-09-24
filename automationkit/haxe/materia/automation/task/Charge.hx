package materia.automation.task;

/** Charge a robot at a named charger to a minimum state of charge. */
class Charge extends Task {
  public final chargerId:String;
  public final targetChargeFraction:Float;

  public function new(id:String, chargerId:String, targetChargeFraction:Float) {
    super(id, TaskKind.Charge);
    if (chargerId == null || chargerId.length == 0 || !Math.isFinite(targetChargeFraction) ||
        targetChargeFraction <= 0.0 || targetChargeFraction > 1.0)
      throw "Charge requires a charger and a target fraction in (0, 1]";
    this.chargerId = chargerId;
    this.targetChargeFraction = targetChargeFraction;
  }
}
