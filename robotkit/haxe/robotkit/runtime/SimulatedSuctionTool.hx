package robotkit.runtime;

import haxe.Int64;
import nativekit.sim.MotionType;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;
import robotkit.spatial.Vec3;
import robotkit.world.ProcessEventValue;

/**
 * A suction tool on one link of a simulated robot, worked by its control channel as the valve of a
 * real ejector is: once the channel is on, it seals on the free object touching the link and the
 * simulation carries that object on the link; once it is off, it lets go. With nothing under it the
 * vacuum finds no seal and holds nothing. A tool with a vacuum sensor publishes the vacuum it pulls
 * there, `sealedKpa` while it holds and none otherwise, so skills tell a grip as they would on a real
 * robot. Add it to the simulation as a step observer; it acts after each step.
 *
 * A tool with no channel is worked by `actuate` instead, for scripted playback of a robot that runs
 * no trajectory plans to carry channel events.
 */
class SimulatedSuctionTool implements SimulationStepObserver {
  /** How far from the link an object may be for the vacuum to seal on it, in metres. */
  public static inline var REACH:Float = 0.004;
  /**
   * The gap left between the cup and what it holds. A held object follows its carrier and cannot
   * yield, so a contact between them would push back on the robot; a gap far above the contact's own
   * tolerance keeps that force at zero and is invisible at this scale.
   */
  public static inline var CLEARANCE:Float = 0.001;

  public final simulation:Simulation;
  public final runtime:RobotRuntime;
  public final robotIndex:Int;
  public final linkIndex:Int;
  public final channel:Null<String>;
  public final sensorId:Null<String>;
  public final sealedKpa:Float;
  /** What the tool holds, or null. */
  public var held(default, null):Null<SimObject> = null;

  final objects:Array<SimObject>;
  var sequence:Int = 0;
  var scripted = false;

  public function new(simulation:Simulation, runtime:RobotRuntime, robotIndex:Int, linkIndex:Int, channel:Null<String>,
      objects:Array<SimObject>, ?sensorId:String, sealedKpa:Float = 80.0) {
    if (simulation == null || runtime == null || robotIndex < 0 || linkIndex < 0 || objects == null ||
        !(sealedKpa > 0))
      throw "Simulated suction tool needs a simulation, robot, link, channel, objects and a positive vacuum";
    this.simulation = simulation;
    this.runtime = runtime;
    this.robotIndex = robotIndex;
    this.linkIndex = linkIndex;
    this.channel = channel;
    this.objects = objects;
    this.sensorId = sensorId;
    this.sealedKpa = sealedKpa;
  }

  /** Turns a tool with no channel on or off; it seals or lets go after the next step. */
  public function actuate(on:Bool):Void {
    if (channel != null) throw "A suction tool with a control channel is worked through it";
    scripted = on;
  }

  public function afterSimulationStep(sourceTimestampNs:Int64):Void {
    var control = channel;
    var on = control == null ? scripted : switch runtime.channelValue(control) {
      case Digital(value): value;
      case _: throw 'Suction channel "$control" is not digital';
    };
    if (on && held == null) seal();
    else if (!on && held != null) release();
    var sensor = sensorId;
    if (sensor != null)
      runtime.publishSensorFrame(sensor, [held == null ? 0.0 : sealedKpa], Int64.ofInt(++sequence), sourceTimestampNs,
        "robotkit.simulation");
  }

  /** The session reset restored every object and the robot's channels: forget what was held. */
  public function reset():Void {
    held = null;
    scripted = false;
  }

  function seal():Void {
    var touched = candidate();
    if (touched == null) return;
    var link = simulation.linkPose(robotIndex, linkIndex);
    var pose = simulation.objectPose(touched.object);
    // Ease the object to the clearance along the contact normal, away from the cup.
    var away = touched.normal;
    var toObject = new Vec3(pose.x - link.position[0], pose.y - link.position[1], pose.z - link.position[2]);
    if (away.dot(toObject) < 0.0) away = new Vec3(-away.x, -away.y, -away.z);
    var shift = Math.max(0.0, CLEARANCE - touched.distance);
    var seated = new SimPose(pose.x + away.x * shift, pose.y + away.y * shift, pose.z + away.z * shift,
      pose.qx, pose.qy, pose.qz, pose.qw);
    simulation.holdObjectOnLink(touched.object, robotIndex, linkIndex, relativePose(link.position, link.rotation, seated));
    held = touched.object;
  }

  function release():Void {
    var object = held;
    held = null;
    if (object != null) simulation.releaseObject(object);
  }

  /** The free object nearest the link within reach. */
  function candidate():Null<{object:SimObject, normal:Vec3, distance:Float}> {
    var best:Null<{object:SimObject, normal:Vec3, distance:Float}> = null;
    var bestDistance = REACH;
    for (contact in simulation.robotContacts(runtime)) {
      if (contact.linkIndex != linkIndex || contact.otherKind != RobotContactOtherKind.Object ||
          contact.distance > bestDistance) continue;
      for (object in objects) {
        if (object.handle.rawValue() != contact.otherObject || object.motion != MotionType.Dynamic) continue;
        best = {object: object, normal: contact.normal, distance: contact.distance};
        bestDistance = contact.distance;
      }
    }
    return best;
  }

  /** The object's pose in the frame of the link that carries it. */
  static function relativePose(linkPosition:Array<Float>, linkRotation:Array<Float>, object:SimPose):SimPose {
    var x = -linkRotation[0], y = -linkRotation[1], z = -linkRotation[2], w = linkRotation[3];
    var v = [object.x - linkPosition[0], object.y - linkPosition[1], object.z - linkPosition[2]];
    var tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
    return new SimPose(v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx,
      w * object.qx + x * object.qw + y * object.qz - z * object.qy,
      w * object.qy - x * object.qz + y * object.qw + z * object.qx,
      w * object.qz + x * object.qy - y * object.qx + z * object.qw,
      w * object.qw - x * object.qx - y * object.qy - z * object.qz);
  }
}
