package processkit.screwdriving;

import materia.project.SceneScrewDefinition;
import materia.project.SceneScrewDefinition.TighteningSettings;

/** A qualified program. A torque table alone cannot supply its angle or stroke windows. */
class TighteningProgram {
  public final settings:TighteningSettings;
  /** A snug pass stops at the head seat; final qualification remains unchanged. */
  public final stopAtSeat:Bool;
  public function new(settings:TighteningSettings, stopAtSeat:Bool = false) {
    this.settings = SceneScrewDefinition.copyProgram(settings);
    this.stopAtSeat = stopAtSeat;
  }
}
