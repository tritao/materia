package robotkit.policy;

import haxe.Int64;
import robotkit.model.Frame;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.Sensor;
import robotkit.policy.ObservationBuilder.Observed;
import robotkit.policy.VelocityReference.VelocityCommand;
import robotkit.world.JointTarget;
import robotkit.world.SensorFrame;

/** Privileged simulator state, for debugging only; see PolicyController.debugTruth. */
typedef DebugTruth = {down:Array<Float>, angularVelocity:Array<Float>};

/**
 * Runs a learned policy as a joint controller: each control period it reads
 * encoders and the base IMU, builds the policy's observation, evaluates the
 * network and returns joint servo targets (JointTarget.servo: the joint's
 * `kp (q_target - q) - kd qdot` evaluated every physics step, HU-D3).
 *
 * The controller sees what a real robot's controller would (HU-D4): encoder
 * positions and velocities, the IMU's gyroscope and accelerometer, its own last
 * action and the velocity command. The direction of gravity comes from a
 * complementary filter on the IMU, not from the simulator. Until the first IMU
 * sample arrives the controller holds the default pose.
 */
class PolicyController {
  public final spec:PolicySpec;
  final policy:OnnxPolicy;
  final observation:ObservationBuilder;
  final estimator:GravityEstimator;
  /** Model joint index of each policy joint, and of each joint that only holds a pose. */
  final policyJoints:Array<Int>;
  final holdJoints:Array<Int> = [];
  /** Travel limits of each policy joint; a target is clamped into them, as the runtime rejects one outside. */
  final lower:Array<Float> = [];
  final upper:Array<Float> = [];
  final holdPose:Array<Float> = [];
  final imuRotation:Array<Float>;
  final observationTensor:String;
  final recurrentState:Map<String, Array<Float>> = new Map();
  var lastAction:Array<Float>;
  var lastImuSequence:Int64 = Int64.ofInt(0);
  var lastImuTimeNs:Null<Int64> = null;
  var angularVelocity:Array<Float> = [0.0, 0.0, 0.0];
  var down:Array<Float> = [0.0, 0.0, -1.0];
  var imuReady:Bool = false;
  /** Policy evaluations so far; the gait clock is this times the control period. */
  public var evaluations(default, null):Int = 0;
  /** Policy targets that had to be clamped into a joint's travel so far. */
  public var clampedTargets(default, null):Int = 0;
  /** Enables the privileged state debugging tools compare the estimator to; never set it in a real controller. */
  public var debugTruth:Null<() -> DebugTruth> = null;

  /**
   * Adds the spec's IMU to `model` when it has no sensor of that ID, mounted
   * on the spec's link. Call before compiling the model.
   */
  public static function prepareModel(model:RobotModel, spec:PolicySpec):Void {
    for (sensor in model.sensors) if (sensor.id == spec.imu.id) return;
    var link:Null<Link> = null;
    for (candidate in model.links) if (candidate.id == spec.imu.link || candidate.name == spec.imu.link) link = candidate;
    if (link == null) throw 'policy spec "${spec.name}": the model has no link "${spec.imu.link}" for the IMU';
    var frame = new Frame(spec.imu.id + "_frame", link);
    frame.position = spec.imu.position.copy();
    frame.rotation = spec.imu.rotation.copy();
    model.addFrame(frame);
    var sensor = new Sensor(spec.imu.id, "imu", 0.0);
    sensor.frame = frame;
    model.addSensor(sensor);
  }

  public function new(spec:PolicySpec, policy:OnnxPolicy, model:RobotModel) {
    this.spec = spec;
    this.policy = policy;
    observation = new ObservationBuilder(spec);
    estimator = new GravityEstimator();
    observationTensor = spec.observationInput;
    lastAction = [for (_ in spec.joints) 0.0];

    var indexOf = function(id:String):Int {
      for (i in 0...model.joints.length) if (model.joints[i].id == id || model.joints[i].name == id) return i;
      return -1;
    };
    policyJoints = [];
    for (id in spec.joints) {
      var index = indexOf(id);
      if (index < 0) throw 'policy spec "${spec.name}": the model has no joint "$id"';
      if (policyJoints.indexOf(index) >= 0) throw 'policy spec "${spec.name}": joint "$id" is listed twice';
      policyJoints.push(index);
      var limits = model.joints[index].limits;
      var limited = limits.upper > limits.lower;
      lower.push(limited ? limits.lower : -Math.POSITIVE_INFINITY);
      upper.push(limited ? limits.upper : Math.POSITIVE_INFINITY);
    }
    var hold = spec.hold;
    if (hold != null)
      for (i in 0...model.joints.length)
        if (policyJoints.indexOf(i) < 0 && model.joints[i].type != robotkit.model.JointType.Fixed) {
          holdJoints.push(i);
          var pose = hold.pose.get(model.joints[i].id);
          if (pose == null) pose = hold.pose.get(model.joints[i].name);
          holdPose.push(pose == null ? 0.0 : pose);
        }
    if (policyJoints.length + holdJoints.length > JointTarget.MAX_BATCH_SIZE)
      throw 'policy spec "${spec.name}": more than ${JointTarget.MAX_BATCH_SIZE} joints to command';

    var imuSensor:Null<Sensor> = null;
    for (sensor in model.sensors) if (sensor.id == spec.imu.id) imuSensor = sensor;
    if (imuSensor == null || imuSensor.kind != "imu")
      throw 'policy spec "${spec.name}": the model has no IMU "${spec.imu.id}" (see PolicyController.prepareModel)';
    var frame = imuSensor.frame;
    imuRotation = frame == null ? [0.0, 0.0, 0.0, 1.0] : frame.rotation.copy();
    var root = rootLink(model);
    if (frame != null && frame.link.id != root)
      throw 'policy spec "${spec.name}": the IMU must be mounted on the base link ($root), not ${frame.link.id}';

    // The network's tensors must match the spec.
    var input = policy.input(spec.observationInput);
    if (input.elements != observation.inputSize())
      throw 'policy spec "${spec.name}": the network takes ${input.elements} observation values, the spec builds ${observation.inputSize()}';
    if (policy.output(spec.actionOutput).elements != spec.joints.length)
      throw 'policy spec "${spec.name}": the network outputs ${policy.output(spec.actionOutput).elements} actions for ${spec.joints.length} joints';
    for (state in spec.recurrent) {
      var stateIn = policy.input(state.input), stateOut = policy.output(state.output);
      if (stateIn.elements != stateOut.elements)
        throw 'policy spec "${spec.name}": recurrent ${state.input} and ${state.output} differ in size';
    }
    for (tensor in policy.inputs) {
      var carried = false;
      for (state in spec.recurrent) if (state.input == tensor.name) carried = true;
      if (tensor.name != spec.observationInput && !carried)
        throw 'policy spec "${spec.name}": the network input "${tensor.name}" is neither the observation nor recurrent state';
    }
    reset();
  }

  static function rootLink(model:RobotModel):String {
    var children = [for (joint in model.joints) joint.child];
    for (link in model.links) if (children.indexOf(link) < 0) return link.id;
    return model.links[0].id;
  }

  /** Forgets the network's state, the history, the IMU filter and the gait clock. */
  public function reset():Void {
    observation.reset();
    estimator.reset();
    for (i in 0...lastAction.length) lastAction[i] = 0.0;
    for (state in spec.recurrent) recurrentState.set(state.input, [for (_ in 0...policy.input(state.input).elements) 0.0]);
    evaluations = 0;
    imuReady = false;
    lastImuTimeNs = null;
    lastImuSequence = Int64.ofInt(0);
  }

  /** True once an IMU sample has been seen and the policy is driving the joints. */
  public function ready():Bool return imuReady;

  /** The estimator's current down vector in the base frame. */
  public function estimatedDown():Array<Float> return down.copy();

  /** Servo targets that hold every joint at the default (and hold) pose. */
  public function standTargets():Array<JointTarget> {
    var targets = [for (i in 0...policyJoints.length)
      JointTarget.servo(policyJoints[i], spec.defaultPose[i], 0.0, spec.kp[i], spec.kd[i], 0.0)];
    return targets.concat(holdTargets());
  }

  function holdTargets():Array<JointTarget> {
    var hold = spec.hold;
    return hold == null ? [] : [for (i in 0...holdJoints.length) JointTarget.servo(holdJoints[i], holdPose[i], 0.0, hold.kp, hold.kd, 0.0)];
  }

  /**
   * Evaluates the policy once. `q` and `dq` are the robot's joint positions and
   * velocities in model order and `sensors` its latest sensor frames.
   */
  public function update(q:Array<Float>, dq:Array<Float>, sensors:Array<SensorFrame>, command:VelocityCommand):Array<JointTarget> {
    readImu(sensors);
    if (!imuReady) return standTargets();
    evaluations++;
    var observed:Observed = {
      q: [for (i in policyJoints) q[i]],
      dq: [for (i in policyJoints) dq[i]],
      angularVelocity: angularVelocity,
      down: down,
      command: command,
      lastAction: lastAction,
      gaitTime: evaluations * spec.controlPeriod
    };
    var inputs = new Map<String, Array<Float>>();
    inputs.set(observationTensor, observation.build(observed));
    for (state in spec.recurrent) inputs.set(state.input, recurrentState.get(state.input));
    var outputs = policy.run(inputs);
    var action = outputs.get(spec.actionOutput);
    for (value in action) if (!Math.isFinite(value)) throw "the policy produced a non-finite action";
    for (state in spec.recurrent) recurrentState.set(state.input, outputs.get(state.output));
    lastAction = action.copy();
    var targets = [for (i in 0...policyJoints.length) {
      var wanted = spec.defaultPose[i] + spec.actionScale * action[i];
      var clamped = Math.max(lower[i], Math.min(upper[i], wanted));
      if (clamped != wanted) clampedTargets++;
      JointTarget.servo(policyJoints[i], clamped, 0.0, spec.kp[i], spec.kd[i], 0.0);
    }];
    return targets.concat(holdTargets());
  }

  /**
   * Feeds the IMU's latest sample to the gravity filter. Call it on every
   * sensor update, which may be much faster than the control period: the
   * filter integrates the gyroscope between policy evaluations, and sampling it
   * only once per evaluation aliases the gait's impacts into a tilt bias.
   */
  public function observeImu(sensors:Array<SensorFrame>):Void readImu(sensors);

  function readImu(sensors:Array<SensorFrame>):Void {
    var frame:Null<SensorFrame> = null;
    for (candidate in sensors) if (candidate.sensorId == spec.imu.id) frame = candidate;
    if (frame == null || frame.values.length < 6 || frame.sequence == Int64.ofInt(0)) return;
    var truth = debugTruth;
    if (frame.sequence != lastImuSequence) {
      var seconds = lastImuTimeNs == null ? 0.0 : Int64.toInt(frame.sourceTimestampNs - lastImuTimeNs) * 1e-9;
      lastImuSequence = frame.sequence;
      lastImuTimeNs = frame.sourceTimestampNs;
      var gyro = rotate(imuRotation, [frame.values.get(0), frame.values.get(1), frame.values.get(2)]);
      var force = rotate(imuRotation, [frame.values.get(3), frame.values.get(4), frame.values.get(5)]);
      angularVelocity = gyro;
      down = estimator.update(gyro, force, seconds);
    }
    if (truth != null) {
      var value = truth();
      angularVelocity = value.angularVelocity;
      down = value.down;
    }
    imuReady = true;
  }

  /** Rotates a sensor-frame vector into the base frame by the sensor's mount rotation (unit xyzw). */
  static function rotate(q:Array<Float>, v:Array<Float>):Array<Float> {
    var x = q[0], y = q[1], z = q[2], w = q[3];
    var tx = 2.0 * (y * v[2] - z * v[1]), ty = 2.0 * (z * v[0] - x * v[2]), tz = 2.0 * (x * v[1] - y * v[0]);
    return [v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx];
  }
}
