package robotkit.localization;

import robotkit.mobile.PlanarMath;
import robotkit.mobile.Pose2;
import robotkit.runtime.Simulation;
import robotkit.world.RobotSnapshot;

/** Projects an owned Simulation robot base pose into a map-frame estimate. */
class SimulationTruthLocalization implements Localization {
  public final simulation:Simulation;
  public final robotIndex:Int;
  public final referenceFrame:String;
  public final bodyFrame:String;
  var currentState:Null<LocalizationState> = null;

  public function new(simulation:Simulation, robotIndex:Int,
      ?referenceFrame:String = "map", ?bodyFrame:String = "base") {
    if (simulation == null || robotIndex < 0)
      throw "Simulation truth localization requires a simulation and valid robot index";
    this.simulation = simulation;
    this.robotIndex = robotIndex;
    this.referenceFrame = referenceFrame;
    this.bodyFrame = bodyFrame;
  }

  public function update(snapshot:RobotSnapshot):LocalizationState {
    if (snapshot == null) throw "Simulation truth localization requires a robot snapshot";
    var value = simulation.robotPose(robotIndex);
    if (value.position.length != 3 || value.rotation.length != 4)
      throw "Simulation returned an invalid base pose";
    var qx = value.rotation[0];
    var qy = value.rotation[1];
    var qz = value.rotation[2];
    var qw = value.rotation[3];
    var norm = qx * qx + qy * qy + qz * qz + qw * qw;
    if (!Math.isFinite(norm) || norm <= 1e-12)
      throw "Simulation returned an invalid base orientation";
    var scale = 1.0 / Math.pow(norm, 0.5);
    qx *= scale;
    qy *= scale;
    qz *= scale;
    qw *= scale;
    var sinYaw = 2.0 * (qw * qz + qx * qy);
    var cosYaw = 1.0 - 2.0 * (qy * qy + qz * qz);
    var pose = new Pose2(value.position[0], value.position[1],
      PlanarMath.atan2(sinYaw, cosYaw));
    currentState = new LocalizationState(snapshot.sourceSequence, pose,
      referenceFrame, bodyFrame, PoseCovariance2.zero(), LocalizationQuality.Good,
      snapshot.sourceTimestampNs, snapshot.receivedTimestampNs,
      snapshot.sourceClockId, snapshot.receivedClockId);
    return currentState;
  }

  public function state():Null<LocalizationState> return currentState;

  public function reset(?pose:Pose2):Void {
    // Simulation owns truth; a requested estimate cannot teleport or rewrite it.
    if (pose != null)
      throw "Simulation truth reset cannot override the simulation pose";
    currentState = null;
  }
}
