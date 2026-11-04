package machinekit.component;

import cadkit.InertiaTensor;
import cadkit.modeling.Vector;

/** Volume in mm³, centroid in mm, and centroidal inertia at unit density (mm⁵) of a solid in its own frame. */
class VolumeProperties {
  public final volume:Float;
  public final centre:Vector;
  public final inertia:InertiaTensor;

  public function new(volume:Float, centre:Vector, inertia:InertiaTensor) {
    if (!Math.isFinite(volume) || volume <= 0) throw "Volume properties need a positive volume";
    this.volume = volume;
    this.centre = centre;
    this.inertia = inertia;
  }
}
