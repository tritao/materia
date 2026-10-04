package robotkit.runtime;

import robotkit.model.LinkId;
import robotkit.model.JointId;
import robotkit.model.SensorId;
import robotkit.model.FrameId;
import robotkit.model.CollisionApproximation;

/** Immutable semantic-ID mapping for one compiled model revision.
 * Sensor indices identify model slots, not SensorKit backend registrations.
 */
class RobotRuntimeIdentity {
  final links:Array<LinkId>;
  final joints:Array<JointId>;
  final sensors:Array<SensorId>;
  final frames:Array<FrameId>;
  final frameLinks:Array<LinkId>;
  final visuals:Array<Null<String>>;
  final collisions:Array<Null<String>>;
  public final collisionApproximation:CollisionApproximation;

  public function new(links:Array<LinkId>, joints:Array<JointId>, sensors:Array<SensorId>,
      ?frames:Array<FrameId>, ?frameLinks:Array<LinkId>,
      ?visuals:Array<Null<String>>, ?collisions:Array<Null<String>>,
      ?collisionApproximation:CollisionApproximation) {
    this.links = links.copy();
    this.joints = joints.copy();
    this.sensors = sensors.copy();
    this.frames = frames == null ? [] : frames.copy();
    this.frameLinks = frameLinks == null ? [] : frameLinks.copy();
    this.visuals = visuals == null ? [] : visuals.copy();
    this.collisions = collisions == null ? [] : collisions.copy();
    this.collisionApproximation = collisionApproximation == null ? CollisionApproximation.BoundsBox : collisionApproximation;
  }

  public function linkId(index:Int):Null<LinkId>
    return index < 0 || index >= links.length ? null : links[index];
  public function jointId(index:Int):Null<JointId>
    return index < 0 || index >= joints.length ? null : joints[index];
  public function sensorId(index:Int):Null<SensorId>
    return index < 0 || index >= sensors.length ? null : sensors[index];

  public function linkIndex(id:LinkId):Int return links.indexOf(id);
  public function jointIndex(id:JointId):Int return joints.indexOf(id);
  public function sensorIndex(id:SensorId):Int return sensors.indexOf(id);
  public function frameIndex(id:FrameId):Int return frames.indexOf(id);
  public function frameId(index:Int):Null<FrameId>
    return index < 0 || index >= frames.length ? null : frames[index];
  public function frameLinkId(index:Int):Null<LinkId>
    return index < 0 || index >= frameLinks.length ? null : frameLinks[index];
  public function visualGeometry(index:Int):Null<String>
    return index < 0 || index >= visuals.length ? null : visuals[index];
  public function collisionGeometry(index:Int):Null<String>
    return index < 0 || index >= collisions.length ? null : collisions[index];
}
