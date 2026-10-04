package robotkit.runtime;

/** Reference and coordinate-zero state for the joints of one compiled machine.
 * Only physical home-switch latches establish a reference. Power-up invalidates it. */
class JointReferenceState {
  final names:Array<String> = [];
  final homes:Array<robotkit.model.JointSwitch>;
  final homeJoints:Array<Int> = [];
  final homeDrives:Array<SwitchDriveBinding> = [];
  final latched:Array<Bool> = [];
  final driveCalibrated:Array<Bool> = [];
  final latchOffsets:Array<Float> = [];
  final leaderLatchOffsets:Array<Float> = [];
  final referenced:Array<Bool> = [];
  final offsets:Array<Float> = [];
  final couplings:Array<RobotRuntimeJointCouplingBlueprint>;

  public function new(blueprint:RobotRuntimeBlueprint, ?template:JointReferenceState) {
    if (template != null) {
      homes = template.homes.copy(); couplings = template.couplings.copy();
      for (value in template.names) names.push(value);
      for (value in template.homeJoints) homeJoints.push(value);
      for (value in template.homeDrives) homeDrives.push(value);
      for (value in template.latched) latched.push(value);
      for (value in template.driveCalibrated) driveCalibrated.push(value);
      for (value in template.latchOffsets) latchOffsets.push(value);
      for (value in template.leaderLatchOffsets) leaderLatchOffsets.push(value);
      for (value in template.referenced) referenced.push(value);
      for (value in template.offsets) offsets.push(value);
      return;
    }
    if (blueprint == null) throw "Reference state requires a runtime blueprint";
    for (joint in 0...blueprint.jointCount) {
      var id = blueprint.identity == null ? null : blueprint.identity.jointId(joint);
      names.push(id == null ? 'joint[$joint]' : id);
      referenced.push(true); offsets.push(0.0);
    }
    homes = [for (contact in blueprint.switches) if (contact.role == "home") contact];
    couplings = blueprint.couplings.copy();
    for (term in couplings) {
      if (term == null) throw "Reference state has a null coupling";
      requireJoint(term.leader); requireJoint(term.follower);
      if (!Math.isFinite(term.ratio) || term.ratio == 0) throw "Reference coupling needs a finite nonzero ratio";
    }
    for (contact in homes) {
      var joint = names.indexOf(contact.joint);
      if (joint < 0) throw 'Home switch "${contact.id}" monitors an unknown joint';
      homeJoints.push(joint); latched.push(false); latchOffsets.push(0.0);
      leaderLatchOffsets.push(0.0);
      homeDrives.push(SwitchDriveBinding.resolve(blueprint, contact));
      driveCalibrated.push(false);
    }
    refresh();
  }

  /** Detached candidate for an atomic native calibration transaction. */
  public function copy():JointReferenceState return new JointReferenceState(null, this);

  public function coordinateOffsets():Array<Float> return offsets.copy();

  public function homeJoint(switchId:String):Int {
    for (i in 0...homes.length) if (homes[i].id == switchId) return homeJoints[i];
    throw 'Unknown home switch "$switchId"';
  }

  /** Only directly monitored joints need native requirements; coupling readiness propagates there. */
  public function requiresHome(joint:Int):Bool {
    requireJoint(joint);
    return homeJoints.indexOf(joint) >= 0;
  }

  /** A joint without a home switch retains its existing reference convention. */
  public function isReferenced(joint:Int):Bool {
    requireJoint(joint);
    return referenced[joint];
  }

  /** Refuse ordinary motion until every mapped coordinate is established. */
  public function requireReferenced(joints:Array<Int>):Void {
    if (joints == null) throw "Reference admission requires joint indices";
    for (joint in joints) {
      requireJoint(joint);
      if (!referenced[joint])
        throw 'RK_JOINT_UNREFERENCED: joint "${names[joint]}" requires homing';
    }
  }

  /** All required home signals, including both Y sides, must latch before admission. */
  public function latch(switchId:String, observedPosition:Float, leaderPosition:Null<Float> = null):Void {
    if (!Math.isFinite(observedPosition)) throw "Home latch position must be finite";
    var index = -1;
    for (i in 0...homes.length) if (homes[i].id == switchId) index = i;
    if (index < 0) throw 'Unknown home switch "$switchId"';
    var zero = homes[index].trip - observedPosition;
    if (!Math.isFinite(zero)) throw "Home coordinate zero must be finite";
    if (!Math.isFinite(homeDrives[index].ratio * zero)) throw "Home shaft coordinate zero must be finite";
    var leaderZero = homes[index].trip - (leaderPosition == null ? observedPosition : leaderPosition);
    if (!Math.isFinite(leaderZero)) throw "Home leader coordinate zero must be finite";
    latchOffsets[index] = zero;
    leaderLatchOffsets[index] = leaderZero;
    latched[index] = true;
    driveCalibrated[index] = false;
    refresh();
  }

  /** Individual side zeros are retained for dual-drive squaring. */
  public function homeOffset(switchId:String):Float {
    for (i in 0...homes.length) if (homes[i].id == switchId) {
      if (!latched[i]) throw 'Home switch "$switchId" has not latched';
      return latchOffsets[i];
    }
    throw 'Unknown home switch "$switchId"';
  }

  /** Individual shaft-coordinate translation from its own captured home edge.
   * This is retained side calibration, not the shared leader's propagated zero. */
  public function homeDriveOffset(switchId:String):Float {
    for (i in 0...homes.length) if (homes[i].id == switchId) {
      if (!latched[i]) throw 'Home switch "$switchId" has not latched';
      var zero = homeDrives[i].ratio * latchOffsets[i];
      if (!Math.isFinite(zero)) throw "Home shaft coordinate zero must be finite";
      return zero;
    }
    throw 'Unknown home switch "$switchId"';
  }

  public function homeDriveJoint(switchId:String):Int {
    for (i in 0...homes.length) if (homes[i].id == switchId) {
      if (!latched[i] || driveCalibrated[i]) throw "Motor calibration requires a new home latch";
      if (homes[i].driveJoint == null) throw "Motor calibration requires an explicit side drive";
      return homeDrives[i].joint;
    }
    throw 'Unknown home switch "$switchId"';
  }

  /** Mark a successfully committed batch; only a new captured latch permits rebasing again. */
  public function markHomeDrivesCalibrated(switchIds:Array<String>):Void {
    for (id in switchIds) homeDriveJoint(id);
    for (i in 0...homes.length) if (switchIds.indexOf(homes[i].id) >= 0) driveCalibrated[i] = true;
  }

  /** A power cycle loses every switch-derived reference and calibrated zero. */
  public function invalidate():Void {
    for (i in 0...latched.length) { latched[i] = false; latchOffsets[i] = 0.0; leaderLatchOffsets[i] = 0.0; driveCalibrated[i] = false; }
    refresh();
  }

  public function invalidateJoint(joint:Int):Void {
    requireJoint(joint);
    for (i in 0...homeJoints.length) if (homeJoints[i] == joint) {
      latched[i] = false; latchOffsets[i] = 0.0; leaderLatchOffsets[i] = 0.0; driveCalibrated[i] = false;
    }
    refresh();
  }

  /** Calibrated logical position = observed counter coordinate + established zero. */
  public function position(joint:Int, observed:Float):Float {
    requireJoint(joint);
    if (!Math.isFinite(observed)) throw "Observed position must be finite";
    return observed + offsets[joint];
  }

  /** Convert a logical target back to its counter coordinate. */
  public function target(joint:Int, logical:Float):Float {
    requireJoint(joint);
    if (!Math.isFinite(logical)) throw "Logical target must be finite";
    return logical - offsets[joint];
  }

  public function offset(joint:Int):Float { requireJoint(joint); return offsets[joint]; }

  function refresh():Void {
    for (joint in 0...names.length) {
      referenced[joint] = true; offsets[joint] = 0.0;
      var first = true;
      for (i in 0...homeJoints.length) if (homeJoints[i] == joint) {
        if (!latched[i]) referenced[joint] = false;
        // The first authored home is the leader's coordinate reference. Other
        // side latches are required independently; their zeros are not averaged.
        if (first) { offsets[joint] = latched[i] ? leaderLatchOffsets[i] : 0.0; first = false; }
      }
    }
    // Fixed-point propagation handles chained and multiple-input followers.
    for (_ in 0...names.length) {
      for (joint in 0...names.length) {
        var hasTerms = false, ready = true, sum = 0.0;
        for (term in couplings) if (term.follower == joint) {
          requireJoint(term.leader);
          hasTerms = true;
          ready = ready && referenced[term.leader];
          sum += term.ratio * offsets[term.leader];
        }
        if (hasTerms) {
          referenced[joint] = referenced[joint] && ready;
          if (!Math.isFinite(sum)) throw "Coupled reference zero must be finite";
          offsets[joint] = sum;
        }
      }
    }
  }

  function requireJoint(joint:Int):Void {
    if (joint < 0 || joint >= names.length) throw "Reference joint index is out of range";
  }
}
