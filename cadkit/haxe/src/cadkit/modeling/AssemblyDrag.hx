package cadkit.modeling;

import kinematicskit.ClosureTask;
import kinematicskit.DampedLeastSquares;
import kinematicskit.FrameOrientation;
import kinematicskit.FrameTask;
import kinematicskit.KinematicProblem;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import kinematicskit.KinematicStatus;
import kinematicskit.SolverWorkspace;
import kinematicskit.Transform;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.units.LengthUnit;

/** Why a drag update stopped where it did. */
enum abstract AssemblyDragStatus(String) to String {
  /** The grabbed frame is on its target (and every closure is closed). */
  var Following = "following";
  /** The target is beyond what the movable joints can reach from here. */
  var OutOfReach = "out-of-reach";
  /** A movable joint sits on its limit with the target still off. */
  var Limited = "limited";
  /** A closure (a pinned linkage) cannot stay closed at this target. */
  var ClosureBroken = "closure-broken";
}

/** One drag update: the preview coordinates and, when the frame is not on target, why. */
class AssemblyDragResult {
  public final status:AssemblyDragStatus;
  /** A sentence for the user, e.g. `Joint "j3" is at its limit`. */
  public final message:String;
  /** Preview coordinate of every movable joint (the same order as `AssemblyDrag.movableJoints`). */
  public final coordinates:Array<Float>;
  /** Distance of the grabbed frame from the target, in the assembly's length unit. */
  public final positionError:Float;
  /** Angle between the grabbed frame and the target, in radians (0 when orientation is free). */
  public final orientationError:Float;
  public final limitedJoints:Array<String>;
  public final brokenClosures:Array<String>;

  public function new(status:AssemblyDragStatus, message:String, coordinates:Array<Float>, positionError:Float,
      orientationError:Float, limitedJoints:Array<String>, brokenClosures:Array<String>) {
    this.status = status;
    this.message = message;
    this.coordinates = coordinates;
    this.positionError = positionError;
    this.orientationError = orientationError;
    this.limitedJoints = limitedJoints;
    this.brokenClosures = brokenClosures;
  }
}

/**
 * Drags a frame of an assembly (an occurrence's origin, or one of its
 * connectors) towards world targets by moving joints, for an editor IK gizmo.
 *
 * The movable joints are the driving joints from the root to the grabbed
 * occurrence (a coupled joint contributes its leader), plus any `dependent`
 * joints — the ones the editor marks to keep closures closed. Every closure
 * of the assembly is part of the problem, so a pinned linkage stays together
 * while its end is dragged.
 *
 * Each update continues from the previous preview with damped least squares
 * (tracking: small, smooth joint motion), within the joints' limits. The
 * `AssemblyState` given is never modified: read the preview from
 * `previewPose`/`result.coordinates`, and call `commit` for the record to
 * apply as one undoable edit.
 */
class AssemblyDrag {
  public final state:AssemblyState;
  public final occurrence:String;
  public final connector:Null<String>;
  /** Joint IDs whose coordinates the drag may change, in the order of `AssemblyDragResult.coordinates`. */
  public final movableJoints:Array<String>;
  public final keepOrientation:Bool;

  final kinematics:AssemblyKinematics;
  final problem:KinematicProblem;
  final task:FrameTask;
  final dofs:Array<Int>;
  final preview:KinematicState;
  final snapshot:KinematicSnapshot;
  final workspace = new SolverWorkspace();
  final positionTolerance:Float;
  /** One metre in the assembly's length unit (caps prismatic steps like a tenth of a radian caps revolute ones). */
  final metre:Float;

  /**
   * `keepOrientation` holds the grabbed frame's orientation to the target's
   * (a tool keeps pointing the same way); otherwise only its position follows.
   */
  public function new(state:AssemblyState, occurrence:String, ?connector:String, ?dependent:Array<String>,
      ?keepOrientation:Bool = true) {
    if (state == null) throw "Assembly drag needs a state";
    this.state = state;
    this.occurrence = occurrence;
    this.connector = connector;
    this.keepOrientation = keepOrientation;
    kinematics = state.kinematicModel();
    var model = kinematics.model;
    var body = kinematics.body(occurrence);

    dofs = [];
    for (joint in model.bodyChain[body]) addDof(model.jointDof[joint]);
    if (dependent != null) for (id in dependent) {
      var dof = model.dofIndex(id);
      if (dof < 0) throw 'Dependent assembly joint "$id" is not a driving tree joint';
      addDof(dof);
    }
    if (dofs.length == 0) throw 'Nothing moves "$occurrence": no driving joint lies between it and its root';
    movableJoints = [for (dof in dofs) model.dofId(dof)];

    var metresPerUnit = LengthUnit.metresPerUnit(state.definition.lengthUnit == null ? "mm" : state.definition.lengthUnit);
    // A micrometre and a hundredth of a degree: far below what a viewport shows.
    positionTolerance = 1e-6 / metresPerUnit;
    metre = 1.0 / metresPerUnit;
    var orientationTolerance = 1.7e-4;
    preview = state.kinematicSeed();
    snapshot = new KinematicSnapshot(model);
    snapshot.evaluate(preview);
    var start = connector == null ? snapshot.bodyPose(body) : snapshot.framePose(kinematics.connectorFrame(occurrence, connector));
    var orientation = keepOrientation ? FrameOrientation.Full : FrameOrientation.Free;
    task = connector == null
      ? new FrameTask(model, body, null, start, positionTolerance, orientationTolerance, FrameTask.ALL_AXES, orientation, occurrence)
      : FrameTask.atFrame(model, kinematics.connectorFrame(occurrence, connector), start, positionTolerance,
        orientationTolerance, null, FrameTask.ALL_AXES, orientation);
    // Weigh position in metres whatever the drawing unit, so the damping means the same for a
    // millimetre assembly as for a robot in metres (otherwise a far target gets undamped steps).
    task.positionWeight = metresPerUnit;
    problem = new KinematicProblem(model).setActiveDofs(dofs).add(task);
    for (closure in 0...model.closureCount())
      problem.add(new ClosureTask(model, closure, positionTolerance, orientationTolerance));
  }

  /** The grabbed frame's world pose in the current preview. */
  public function grabbedPose():AssemblyFrame return AssemblyKinematics.toFrame(task.currentPose(snapshot));

  /** An occurrence's world pose in the current preview, for rendering it. */
  public function previewPose(occurrenceId:String):AssemblyFrame
    return AssemblyKinematics.toFrame(snapshot.bodyPose(kinematics.body(occurrenceId)));

  /** Moves the grabbed frame towards `target` (a world frame, in the assembly's length unit). */
  public function drag(target:AssemblyFrame, ?maxIterations:Int = 60):AssemblyDragResult {
    task.setTarget(AssemblyKinematics.fromFrame(target));
    // Capped steps: a target far outside the reach would otherwise swing joints by radians per iteration.
    var solution = DampedLeastSquares.solve(problem, preview, maxIterations, 0.02, 1e-8, workspace, 0.1, metre);
    for (dof in dofs) preview.q[dof] = solution.state.q[dof];
    snapshot.evaluate(preview);
    var model = kinematics.model;
    var grabbed = solution.tasks[0];
    var broken = [for (i in 1...solution.tasks.length) if (!solution.tasks[i].satisfied) solution.tasks[i].label];
    var limited = [for (dof in solution.limitHits) model.dofId(dof)];
    var coordinates = [for (dof in dofs) preview.q[dof]];
    var status:AssemblyDragStatus, message:String;
    if (solution.status == KinematicStatus.Converged) {
      status = Following;
      message = "Following";
    } else if (broken.length > 0) {
      status = ClosureBroken;
      message = broken.length == 1 ? 'Linkage "${broken[0]}" cannot stay closed here'
        : 'Linkages ${broken.join(", ")} cannot stay closed here';
    } else if (limited.length > 0) {
      status = Limited;
      message = limited.length == 1 ? 'Joint "${limited[0]}" is at its limit' : 'Joints ${limited.join(", ")} are at their limits';
    } else {
      status = OutOfReach;
      message = 'Out of reach: ${formatDistance(grabbed.positionError)} from the target';
    }
    return new AssemblyDragResult(status, message, coordinates, grabbed.positionError,
      keepOrientation ? grabbed.orientationError : 0.0, limited, broken);
  }

  /**
   * The state with the preview applied: the record to store as one undoable
   * edit. Coupled joints follow their leaders.
   */
  public function commit():AssemblyStateRecord {
    var result = new AssemblyState(state.definition, state.record());
    var model = kinematics.model;
    for (dof in dofs) result.setJoint(model.dofId(dof), preview.q[dof]);
    return result.record();
  }

  function addDof(dof:Int):Void {
    if (dof >= 0 && dofs.indexOf(dof) < 0) dofs.push(dof);
  }

  function formatDistance(value:Float):String {
    var unit = state.definition.lengthUnit == null ? "mm" : state.definition.lengthUnit;
    return '${Math.round(value * 10) / 10} $unit';
  }
}
