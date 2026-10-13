package robotkit.collision;

import collisionkit.CollisionDescription;
import collisionkit.kinematics.ModelBodies;

/** The description plus its mechanical carrier mapping; policy stays in the description. */
class CollisionClearanceLayout {
  public final description:CollisionDescription;
  public final bodies:ModelBodies;

  public function new(description:CollisionDescription, bodies:ModelBodies) {
    if (description == null || bodies == null) throw "Described clearance needs geometry and kinematics";
    this.description = description; this.bodies = bodies;
  }

}
