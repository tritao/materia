package robotkit.manipulation;

import robotkit.spatial.Transform3;
import robotkit.spatial.Quat;
import kinematicskit.FrameOrientation;

/** How `KinematicGroup.solve` runs. */
enum abstract IkMethod(String) to String {
  /** Damped least squares: small, smooth steps from a nearby seed (tracking, dragging). */
  var Tracking = "tracking";
  /** Levenberg-Marquardt: reaches far targets from a distant seed. */
  var Reaching = "reaching";
  /**
   * Hard tasks (the tool target, an exact swivel) as equalities, soft ones
   * (a posture, a swivel preference) as well as they allow: kinematicskit's
   * `PrioritizedSolver`. Chosen whenever a posture is preferred.
   */
  var Prioritized = "prioritized";
}

/**
 * Settings for one `KinematicGroup.solve`. The defaults suit tracking a
 * tool-centre-point target from a nearby seed.
 */
class IkOptions {
  /** Position axes in the target frame; 7 selects X, Y and Z. */
  public var positionAxes:Int = 7;
  /** Hard orientation rows; an axis leaves spin about that local axis free. */
  public var orientation:FrameOrientation = Full;
  /** Soft tool orientation, optimized only after the selected hard rows. */
  public var orientationPreference:Null<Quat> = null;
  public var orientationPreferenceWeight:Float = 0.05;
  public var positionTolerance:Float;
  public var orientationTolerance:Float;
  public var maxIterations:Int;
  /** Damping of the DLS step (tracking) or the initial LM damping (reaching). */
  public var damping:Float;
  public var method:IkMethod = Tracking;
  /** The target is the flange's pose, not the tool centre point's. */
  public var atFlange:Bool = false;
  /** A swivel angle to solve at (radians; see `ArmSwivel`); needs a swivel. */
  public var swivel:Null<Float> = null;
  /** Hold `swivel` exactly; otherwise it is a preference the tool target wins over. */
  public var swivelExact:Bool = true;
  public var swivelTolerance:Float = 1e-4;
  /**
   * A posture the group is drawn towards (`q` order; external axes ignored):
   * the arm stays comfortable and the external axes bring the work to it.
   * Solved prioritized, so it shapes the configuration only; the tool target
   * is still met exactly.
   */
  public var posture:Null<Array<Float>> = null;
  /** Strength of the pull towards `posture`, per radian (or metre) away. */
  public var postureWeight:Float = 0.05;
  /**
   * Move the robot's base as well (`KinematicGroup.baseMotion`): the robot
   * root's current world pose. The target is then in the world frame, and the
   * result reports where the base moved (`IKResult.rootPose`).
   */
  public var rootPose:Null<Transform3> = null;
  /** With `rootPose`: how much the base's motion costs against the arm's (> 0 lets the arm go first). */
  public var baseCost:Float = 0.1;
  /** Group DOFs (`q` indices) held at `heldValues` while the rest solve, e.g. a positioner at a chosen angle. */
  public var held:Null<Array<Int>> = null;
  public var heldValues:Null<Array<Float>> = null;

  public function new(?positionTolerance:Float = 1e-4, ?orientationTolerance:Float = 1e-3, ?maxIterations:Int = 100,
      ?damping:Float = 0.02) {
    this.positionTolerance = positionTolerance;
    this.orientationTolerance = orientationTolerance;
    this.maxIterations = maxIterations;
    this.damping = damping;
  }

  /** Selects the hard tool task. Returns this. */
  public function freedom(orientation:FrameOrientation, ?positionAxes:Int = 7):IkOptions {
    if (orientation == null) throw "IK task requires an orientation policy";
    if (positionAxes < 0 || positionAxes > 7) throw "IK position axes must be a mask of X, Y and Z";
    this.positionAxes = positionAxes;
    this.orientation = orientation;
    return this;
  }

  /** Prefers an orientation without adding any hard orientation rows. */
  public function preferringOrientation(rotation:Quat, ?weight:Float = 0.05):IkOptions {
    if (rotation == null || !Math.isFinite(weight) || weight <= 0)
      throw "IK orientation preference needs a rotation and positive finite weight";
    orientationPreference = rotation;
    orientationPreferenceWeight = weight;
    return this;
  }

  /** Targets the flange instead of the tool centre point. Returns this. */
  public function flange():IkOptions {
    atFlange = true;
    return this;
  }

  /** Solves at a swivel angle, held exactly or as a preference. Returns this. */
  public function atSwivel(angle:Float, ?exact:Bool = true, ?tolerance:Float = 1e-4):IkOptions {
    swivel = angle;
    swivelExact = exact;
    swivelTolerance = tolerance;
    return this;
  }

  /** Moves the robot's base too, from its current world pose; the target is then in the world. Returns this. */
  public function movingBase(rootPose:Transform3, ?baseCost:Float = 0.1):IkOptions {
    this.rootPose = rootPose;
    this.baseCost = baseCost;
    return this;
  }

  /** Levenberg-Marquardt, for far targets from a distant seed. Returns this. */
  public function reaching():IkOptions {
    method = Reaching;
    return this;
  }

  /** Draws the arm towards `posture` while the external axes take up the rest. Returns this. */
  public function preferring(posture:Array<Float>, ?weight:Float = 0.05):IkOptions {
    this.posture = posture.copy();
    postureWeight = weight;
    return this;
  }

  /** Holds the group DOFs `indices` (in `q` order) at `values` while the rest solve. Returns this. */
  public function holding(indices:Array<Int>, values:Array<Float>):IkOptions {
    if (indices == null || values == null || indices.length != values.length)
      throw "Held DOFs need one value each";
    held = indices.copy();
    heldValues = values.copy();
    return this;
  }

  public function copy():IkOptions {
    var result = new IkOptions(positionTolerance, orientationTolerance, maxIterations, damping);
    result.positionAxes = positionAxes;
    result.orientation = orientation;
    result.orientationPreference = orientationPreference;
    result.orientationPreferenceWeight = orientationPreferenceWeight;
    result.method = method;
    result.atFlange = atFlange;
    result.swivel = swivel;
    result.swivelExact = swivelExact;
    result.swivelTolerance = swivelTolerance;
    result.posture = posture == null ? null : posture.copy();
    result.postureWeight = postureWeight;
    result.rootPose = rootPose;
    result.baseCost = baseCost;
    result.held = held == null ? null : held.copy();
    result.heldValues = heldValues == null ? null : heldValues.copy();
    return result;
  }
}
