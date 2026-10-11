package processkit.screwdriving;

import materia.project.SceneScrewDefinition;
import materia.project.SceneScrewDefinition.TighteningSettings;

/** A qualified program. A torque table alone cannot supply its angle or stroke windows. */
class TighteningProgram {
  public final settings:TighteningSettings;
  public function new(settings:TighteningSettings) {
    this.settings = SceneScrewDefinition.copyProgram(settings);
  }
}
