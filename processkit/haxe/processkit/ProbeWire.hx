package processkit;

import collisionkit.CollisionGeometry;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import robotkit.collision.CollisionSceneBuilder;
import robotkit.collision.CollisionSceneBuilder.CollisionOwner;
import robotkit.manipulation.Manipulator;
import robotkit.model.LinkId;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

/** The probe's wire in the shared collision description, with dimensions for stopping bounds. */
class ProbeWire {
  public static inline var NAME:String = "contact sensing wire";
  public final arm:Manipulator;
  /** Extra reach beyond the exact flat-ended cut-wire profile. */
  public final envelopeExcess:Float;
  public final object:Int;
  final link:LinkId;
  final centre:Vec3;
  final extent:Float;

  function new(arm:Manipulator, link:LinkId, centre:Vec3, radius:Float, halfLength:Float, object:Int) {
    this.arm = arm; this.link = link; this.centre = centre; this.object = object;
    envelopeExcess = 0.0;
    extent = Math.sqrt(halfLength * halfLength + radius * radius);
  }

  /** The welder cuts a flat end; cylinder length and diameter equal the declared CAD wire. */
  public static function describe(scene:CollisionSceneBuilder, arm:Manipulator, link:LinkId, tip:Transform3,
      diameter:Float, stickout:Float, toolObjects:Array<Int>, workObjects:Array<Int>,
      airMargin:Float = WeldPathPlanner.AIR_MARGIN):ProbeWire {
    if (arm == null || scene.arm != arm || tip == null || !Math.isFinite(diameter) || diameter <= 0 ||
        !Math.isFinite(stickout) || stickout <= 0 || !Math.isFinite(airMargin) || airMargin <= 0)
      throw "Probe wire needs a matching description, CAD dimensions and its link-local tip";
    var radius = diameter / 2, halfLength = stickout / 2;
    var offset = tip.compose(new Transform3(new Vec3(0, 0, -stickout / 2), Quat.identity()));
    var name = NAME;
    scene.addGeometry(name, Mechanism(scene.owner, link), CollisionGeometry.Cylinder(radius, halfLength), offset, true);
    var index = scene.object(name);
    for (tool in toolObjects)
      scene.pair(name, scene.description.objects[tool].name, CollisionPairRule.Allow, CollisionPairStatus.Rigid);
    for (work in workObjects)
      scene.pair(name, scene.description.objects[work].name, CollisionPairRule.Touch, CollisionPairStatus.Checked, airMargin, 0.0);
    scene.description.note("Contact sensing wire: exact flat-ended cylinder matches the cut wire diameter and stickout, including the distal rim; no rounded-end assumption or box approximation");
    return new ProbeWire(arm, link, offset.translation, radius, halfLength, index);
  }

  /** Distal lever bound of the same flat-ended cylinder used by native point and continuous collision queries. */
  public function extentFrom(q:Array<Float>, origin:Vec3):Float {
    var pose = arm.linkPoses(q, [link])[0];
    return pose.transformPoint(centre).sub(origin).norm() + extent;
  }
}
