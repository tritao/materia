package cadbridge;

import cadkit.modeling.AssemblyState;
import cadbridge.EndEffectorRuntimeBridge.EndEffectorRuntimeBundle;
import haxe.Int64;
import machinekit.pneumatic.SuctionCup;
import machinekit.robotics.EndEffectorSet;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.Simulation;
import robotkit.runtime.SimulationStepObserver;
import robotkit.tool.SimulatedVacuum;
import robotkit.tool.ToolCollisionShape;
import robotkit.tool.ToolRuntimeSelection;

/** Deterministic pressure feedback from simulation contact on one cup piece.
 * Geometric touch is a seal proxy; this model has no leakage or evacuation time.
 */
class EndEffectorVacuumFeedback implements SimulationStepObserver {
  final selection:ToolRuntimeSelection;
  final bundle:EndEffectorRuntimeBundle;
  final robot:RobotRuntime;
  final cupPieceIndex:Int;
  final sealedVacuumKpa:Float;
  final adapter:robotkit.tool.SimulatedToolSensorAdapter;
  var sequence:Int64 = Int64.ofInt(0);
  var lastTimestampNs:Null<Int64>;
  var attached:Null<Simulation>;
  var observerId:Null<Int>;

  public function new(set:EndEffectorSet, configurationId:String,
      selection:ToolRuntimeSelection, bundle:EndEffectorRuntimeBundle,
      robot:RobotRuntime, sealedVacuumKpa:Float = 60.0, ?state:AssemblyState) {
    if (set == null || selection == null || bundle == null || robot == null ||
        bundle.bindings.vacuumSensorId == null || bundle.runtime.vacuum == null)
      throw "Simulation vacuum feedback needs a coupled pressure sensor and vacuum runtime";
    if (!Math.isFinite(sealedVacuumKpa) || sealedVacuumKpa <= 0 || sealedVacuumKpa > 101.325)
      throw "Simulated sealed vacuum must be within ambient pressure";
    if (bundle.runtime.tool.id.indexOf(configurationId + "/") != 0)
      throw "Vacuum feedback configuration does not match the mounted tool";
    if (!robot.hasExternalSensor(bundle.bindings.vacuumSensorId, "tool_vacuum_kpa"))
      throw "Robot blueprint needs the authored EOAT pressure sensor";
    var configuration = set.configuration(configurationId);
    var collision = EndEffectorCollision.pieces(configuration, state);
    var cups:Array<String> = [];
    for (member in configuration.components())
      if (Std.isOfType(member.component, SuctionCup)) cups.push(member.id);
    if (cups.length != 1)
      throw "Simulation vacuum feedback needs exactly one suction cup";
    var cupIndex = -1;
    for (index in 0...collision.pieces.length)
      if (collision.pieces[index].memberIds.indexOf(cups[0]) >= 0) {
        if (collision.pieces[index].memberIds.length != 1)
          throw "Suction cup collision piece is merged with another member";
        cupIndex = index;
      }
    if (cupIndex < 0) throw "Suction cup has no collision piece";
    switch bundle.runtime.tool.collision {
      case Hulls(pieces, _):
        if (pieces.length != collision.pieces.length)
          throw "Mounted tool collision does not match the coupled configuration";
      case _: throw "Simulation vacuum feedback needs per-member tool hulls";
    }
    this.selection = selection;
    this.bundle = bundle;
    this.robot = robot;
    this.cupPieceIndex = cupIndex;
    this.sealedVacuumKpa = sealedVacuumKpa;
    adapter = EndEffectorRuntimeBridge.bindSensors(selection, bundle);
  }

  /** Register for every explicit private or joined-session step. */
  public function attach(simulation:Simulation):Void {
    if (simulation == null || attached != null)
      throw "Vacuum feedback needs one simulation attachment";
    if (!simulation.ownsRobot(robot))
      throw "Vacuum feedback robot does not belong to this simulation";
    observerId = simulation.addStepObserver(this);
    attached = simulation;
  }

  public function detach():Void {
    if (attached == null) return;
    if (observerId != null) attached.removeStepObserver(observerId);
    attached = null;
    observerId = null;
  }

  public function afterSimulationStep(sourceTimestampNs:Int64):Void {
    sample(sourceTimestampNs);
  }

  /** Publish one authored sensor frame and update the selected tool. */
  public function sample(timestampNs:Int64):Bool {
    if (timestampNs == null || Int64.compare(timestampNs, Int64.ofInt(0)) < 0 ||
        (lastTimestampNs != null && Int64.compare(timestampNs, lastTimestampNs) <= 0))
      throw "Vacuum feedback needs increasing non-negative simulation timestamps";
    if (selection.active() == bundle.runtime &&
        selection.selectedClockId() != "robotkit.simulation")
      throw "Vacuum feedback selection needs the robotkit.simulation source clock";
    var vacuum:SimulatedVacuum = cast bundle.runtime.vacuum;
    var sealed = false;
    if (selection.active() == bundle.runtime && vacuum.isEnabled())
      for (contact in robot.contacts())
        if (contact.toolPieceIndex == cupPieceIndex && contact.distance <= 0.0) {
          sealed = true;
          break;
        }
    var nextSequence = Int64.add(sequence, Int64.ofInt(1));
    var frame = robot.publishSensorFrameAndGet(bundle.bindings.vacuumSensorId,
      [sealed ? sealedVacuumKpa : 0.0], nextSequence, timestampNs,
      "robotkit.simulation");
    sequence = nextSequence;
    lastTimestampNs = timestampNs;
    return adapter.apply(frame);
  }
}
