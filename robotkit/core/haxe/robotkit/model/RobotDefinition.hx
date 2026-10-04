package robotkit.model;

import robotkit.profile.RobotProfile;

/** A mechanical model and its separately authored interpretation, passed together by consumers. */
class RobotDefinition {
  public final model:RobotModel;
  public final profile:RobotProfile;
  public function new(model:RobotModel, profile:RobotProfile) {
    if (model == null || profile == null) throw "Robot definition requires a model and profile";
    this.model = model;
    this.profile = profile;
  }
}
