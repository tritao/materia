import collisionkit.CollisionGeometry;
import collisionkit.CollisionPose;
import collisionkit.CollisionMarginClass;
import robotkit.collision.CollisionSceneBuilder;
import robotkit.collision.CollisionSceneBuilder.CollisionOwner;
import robotkit.spatial.Transform3;
import robotkit.collision.CollisionClearance;
import robotkit.collision.CollisionClearanceLayout;
import robotkit.collision.RobotCollision.RobotLinkShape;
import robotkit.manipulation.KinematicGroup;

/** Standalone fixtures author mechanisms and work/obstacles explicitly through the kit adapters. */
class CollisionFixture {
  final arm:KinematicGroup;
  final reference:Array<Float>;
  final margin:Float;
  final contactMargin:Float;
  final hulls:Array<{hull:RobotLinkShape, tool:Bool}> = [];
  final fixtures:Array<{name:String, vertices:Array<Float>, pose:CollisionPose, work:Bool}> = [];

  public function new(arm:KinematicGroup, reference:Array<Float>, margin:Float = 0.005, contactMargin:Float = 0.0005) {
    this.arm = arm; this.reference = reference.copy(); this.margin = margin; this.contactMargin = contactMargin;
  }

  public function mechanism(name:String, link:String, vertices:Array<Float>, tool:Bool = false):Void {
    var index = -1;
    for (i in 0...arm.robot.links.length) if (arm.robot.links[i].id == link) index = i;
    if (index < 0) throw 'Fixture hull "$name" has unknown link "$link"';
    hulls.push({hull: new RobotLinkShape(index, name, new robotkit.model.CollisionShape(CollisionGeometry.Convex(vertices)), tool ? CollisionMarginClass.Tool : CollisionMarginClass.Installed), tool: tool});
  }

  public function fixture(name:String, vertices:Array<Float>, work:Bool, ?pose:CollisionPose):Void
    fixtures.push({name: name, vertices: vertices, pose: pose == null ? CollisionPose.identity() : pose, work: work});

  public function layout():CollisionClearanceLayout {
    var scene = new CollisionSceneBuilder(arm, reference, "fixture", margin, arm.stateOf(reference),
      [for (entry in hulls) entry.hull]);
    for (entry in hulls)
      scene.identify(entry.hull.name, Mechanism("fixture", arm.robot.links[entry.hull.link].id));
    for (part in fixtures) {
      var at = part.pose;
      scene.addGeometry(part.name, part.work ? Loose : Fixture, CollisionGeometry.Convex(part.vertices),
        Transform3.fromArrays([at.x, at.y, at.z], [at.qx, at.qy, at.qz, at.qw]));
    }
    scene.description.setGroupMargins(CollisionMarginClass.Tool, CollisionMarginClass.Work, margin, contactMargin);
    var articulation = scene.bodies.declare();
    return new CollisionClearanceLayout(scene.description, scene.bodies);
  }

  public function clearance():CollisionClearance
    {
      var layout = layout();
      return new CollisionClearance(arm, layout.description, reference, () -> new collisionkit.native.NativeCollisionWorld(), layout.bodies);
    }
}
