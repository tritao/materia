package processkit.simulation;

import robotkit.runtime.*;

import haxe.Int64;
import processkit.tool.WeldArcModel.WeldArcConfig;
import processkit.tool.WeldArcModel.WeldArcInput;
import processkit.tool.WeldArcModel;
import processkit.tool.WeldSensor.WeldReading;
import processkit.tool.WeldSensor;
import processkit.tool.WeldWork;
import robotkit.execution.ProcessEventValue;

/**
 * A MIG/MAG welder on one link of a simulated robot, worked by its channels as a real supply is: the arc
 * (digital), the wire speed (metres per minute) and the voltage (volts). After each step it finds where the wire
 * tip is, asks the grounded work how far away that is, lets `WeldArcModel` say what the circuit does (see there for
 * the arc, the current and voltage, touch and faults) and publishes the result as one `tool_weld` frame on its
 * sensor, as the suction tool publishes its vacuum.
 *
 * **Geometry.** The physics engine reports contacts of the robot's collision shapes, and the wire is not one of
 * them: it is a few millimetres of metal beyond the nozzle, and the arc strikes across a gap, which a contact
 * would not report. So the welder asks the work itself, in the world frame: the distance of the wire tip from
 * the grounded solids (`WeldWork.distance`) and a ray along the wire (`WeldWork.ray`). The work is convex solids
 * on the links that carry them (`GroundedWork`), so it follows them when they move.
 *
 * The tip is the tool point: `tipPosition` and `wireDirection` are in the frame of the link that carries the torch
 * (metres), the direction being the torch's +Z, along the wire out of the nozzle.
 *
 * Add it to the simulation as a step observer; it acts after each step. Its arc channel is not a `keepOnStop`
 * channel, so a stop or a fault takes it to its safe value, off, and the arc goes out.
 */
class SimulatedWelder implements SimulationStepObserver {
  public final simulation:Simulation;
  public final runtime:RobotRuntime;
  public final robotIndex:Int;
  public final linkIndex:Int;
  public final arcChannel:String;
  public final wireSpeedChannel:String;
  public final voltageChannel:String;
  public final sensorId:String;
  public final model:WeldArcModel;
  /** The supply is on and healthy; clear it to model a dropout of the mains or the gas. */
  public var supplyReady:Bool = true;
  /** Optional device supply; its real feedback also controls the bead model. */
  public var supply:Null<SimulationWelderSupply> = null;
  var deviceReading:WeldReading = {arc:false, currentA:0.0, voltageV:0.0, touch:false, fault:0, powerW:0.0};

  final work:processkit.tool.GroundedWork;
  final tipPosition:Array<Float>;
  final wireDirection:Array<Float>;
  var sequence:Int = 0;
  var lastTimestampNs:Null<Int64> = null;

  public function new(simulation:Simulation, runtime:RobotRuntime, robotIndex:Int, linkIndex:Int, tipPosition:Array<Float>,
      wireDirection:Array<Float>, work:WeldWork, arcChannel:String, wireSpeedChannel:String, voltageChannel:String,
      sensorId:String, config:WeldArcConfig) {
    if (simulation == null || runtime == null || robotIndex < 0 || linkIndex < 0 || work == null ||
        tipPosition == null || tipPosition.length != 3 || wireDirection == null || wireDirection.length != 3 ||
        arcChannel == null || wireSpeedChannel == null || voltageChannel == null || sensorId == null)
      throw "Simulated welder needs a simulation, robot, link, tip, wire direction, work, channels and sensor";
    var length = Math.sqrt(wireDirection[0] * wireDirection[0] + wireDirection[1] * wireDirection[1] +
      wireDirection[2] * wireDirection[2]);
    if (!(length > 0)) throw "Simulated welder needs a wire direction";
    this.simulation = simulation;
    this.runtime = runtime;
    this.robotIndex = robotIndex;
    this.linkIndex = linkIndex;
    this.work = new processkit.tool.GroundedWork().addWork(work);
    this.tipPosition = tipPosition.copy();
    this.wireDirection = [for (axis in wireDirection) axis / length];
    this.arcChannel = arcChannel;
    this.wireSpeedChannel = wireSpeedChannel;
    this.voltageChannel = voltageChannel;
    this.sensorId = sensorId;
    this.model = new WeldArcModel(config);
  }

  /** The latest reading. */
  public function reading():WeldReading return supply == null ? model.reading : deviceReading;

  /** Metal deposited on a grounded workpiece extends its electrical contact geometry. */
  public function addWork(work:WeldWork):Void this.work.addWork(work);

  /** Where the wire tip is now, in the world frame, in metres. */
  public function tip():Array<Float> {
    var link = simulation.linkPose(robotIndex, linkIndex);
    return place(link.position, link.rotation, tipPosition);
  }

  /** The wire speed the channel commands now, in metres per minute. */
  public function wireSpeed():Float return Math.max(0.0, analog(wireSpeedChannel));

  public function afterSimulationStep(sourceTimestampNs:Int64):Void {
    var previous = lastTimestampNs;
    lastTimestampNs = sourceTimestampNs;
    // Source time is monotonic across resets; a session reset (which calls `reset`) starts over from zero.
    var dt = previous == null ? 0.0 : Math.max(0.0, Int64.toFloat(Int64.sub(sourceTimestampNs, previous)) * 1.0e-9);
    var link = simulation.linkPose(robotIndex, linkIndex);
    var tip = place(link.position, link.rotation, tipPosition);
    var wire = turn(link.rotation, wireDirection);
    if (supply != null) {
      var touching = work.distance(tip[0], tip[1], tip[2]) <= WeldArcModel.STRIKE_REACH;
      var ahead = work.ray(tip[0], tip[1], tip[2], wire[0], wire[1], wire[2], WeldArcModel.STRIKE_REACH);
      deviceReading = supply.observe(dt, sourceTimestampNs, supplyReady && (touching || ahead <= WeldArcModel.STRIKE_REACH));
      return;
    }
    var reading = model.step(dt, {
      arcCommanded: digital(arcChannel),
      wireSpeed: Math.max(0.0, analog(wireSpeedChannel)),
      voltageSet: analog(voltageChannel),
      supplyReady: supplyReady,
      tipDistance: work.distance(tip[0], tip[1], tip[2]),
      wireDistance: work.ray(tip[0], tip[1], tip[2], wire[0], wire[1], wire[2], WeldArcModel.STRIKE_REACH)
    });
    runtime.publishSensorFrame(sensorId, WeldSensor.values(reading), Int64.ofInt(++sequence), sourceTimestampNs,
      "robotkit.simulation");
  }

  /** The session reset restored the robot's channels: forget the arc and any fault. */
  public function reset():Void {
    model.reset();
    if (supply != null) supply.reset();
    lastTimestampNs = null;
  }

  function digital(channel:String):Bool
    return switch runtime.channelValue(channel) {
      case Digital(value): value;
      case _: throw 'Welder channel "$channel" is not digital';
    };

  function analog(channel:String):Float
    return switch runtime.channelValue(channel) {
      case Analog(value): value;
      case _: throw 'Welder channel "$channel" is not analog';
    };

  static function place(position:Array<Float>, rotation:Array<Float>, local:Array<Float>):Array<Float> {
    var moved = turn(rotation, local);
    return [position[0] + moved[0], position[1] + moved[1], position[2] + moved[2]];
  }

  /** `v` turned by the xyzw quaternion `q`. */
  static function turn(q:Array<Float>, v:Array<Float>):Array<Float> {
    var x = q[0], y = q[1], z = q[2], w = q[3];
    var tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
    return [v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx];
  }
}
