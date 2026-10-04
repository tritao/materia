package robotkit.policy;

import haxe.Json;

/** One term of the observation vector, in the order the policy was trained on. */
enum ObservationTerm {
  /** Base angular velocity from the gyroscope, rad/s in the base frame. 3 values. */
  AngularVelocity(scale:Array<Float>);
  /** Unit down vector in the base frame, estimated from the IMU. 3 values. */
  ProjectedGravity(scale:Array<Float>);
  /** The velocity reference: forward, lateral, yaw rate. 3 values. */
  VelocityCommand(scale:Array<Float>);
  /** Joint positions from the encoders, minus the default pose. One value per policy joint. */
  JointPosition(scale:Array<Float>);
  /** Joint velocities from the encoders. One value per policy joint. */
  JointVelocity(scale:Array<Float>);
  /** The previous action, as the policy output it. One value per policy joint. */
  LastAction(scale:Array<Float>);
  /** sin and cos of a gait clock with the given period in seconds. 2 values. */
  GaitPhase(period:Float);
}

/** A tensor the policy carries from one call to the next, e.g. an LSTM's h and c, zero at the start. */
typedef RecurrentState = {input:String, output:String};

/** Where the IMU the observation reads is mounted, when the model does not already have one. */
typedef ImuSpec = {
  id:String,
  link:String,
  position:Array<Float>,
  rotation:Array<Float>
};

/** Gains for the joints the policy does not drive: they hold a pose. */
typedef HoldSpec = {kp:Float, kd:Float, pose:Map<String, Float>};

/** Limits and deadline handling for the velocity command. */
typedef CommandSpec = {limit:Array<Float>, acceleration:Array<Float>};

/**
 * Everything a controller needs to run a learned policy on a robot, apart from
 * the network itself: what the network sees and how its output becomes joint
 * servo targets. It is authored beside the model as JSON (`policy.json`).
 *
 * Observation terms are scaled element-wise, as in training; a scale is one
 * number or one per element.
 */
class PolicySpec {
  public static inline final VERSION = 1;

  public final name:String;
  /** Free-form provenance: where the policy came from and under which licence. */
  public final source:Map<String, String>;
  /** ONNX file, relative to the spec file. */
  public final model:String;
  /** Seconds between policy evaluations. */
  public final controlPeriod:Float;
  /** Observations stacked into the input, newest last; 1 means no history. */
  public final historyLength:Int;
  /** RobotModel joint IDs the policy drives, in the order of its observation and action. */
  public final joints:Array<String>;
  public final defaultPose:Array<Float>;
  public final kp:Array<Float>;
  public final kd:Array<Float>;
  /** target = default + actionScale * action. */
  public final actionScale:Float;
  public final observation:Array<ObservationTerm>;
  public final observationInput:String;
  public final actionOutput:String;
  public final recurrent:Array<RecurrentState>;
  public final imu:ImuSpec;
  public final hold:Null<HoldSpec>;
  public final command:CommandSpec;

  public function new(fields:{
    name:String, source:Map<String, String>, model:String, controlPeriod:Float, historyLength:Int,
    joints:Array<String>, defaultPose:Array<Float>, kp:Array<Float>, kd:Array<Float>, actionScale:Float,
    observation:Array<ObservationTerm>, observationInput:String, actionOutput:String,
    recurrent:Array<RecurrentState>, imu:ImuSpec, hold:Null<HoldSpec>, command:CommandSpec
  }) {
    name = fields.name; source = fields.source; model = fields.model;
    controlPeriod = fields.controlPeriod; historyLength = fields.historyLength;
    joints = fields.joints; defaultPose = fields.defaultPose; kp = fields.kp; kd = fields.kd;
    actionScale = fields.actionScale; observation = fields.observation;
    observationInput = fields.observationInput; actionOutput = fields.actionOutput;
    recurrent = fields.recurrent; imu = fields.imu; hold = fields.hold; command = fields.command;
    validate();
  }

  /** Number of values in one (unstacked) observation. */
  public function observationSize():Int {
    var size = 0;
    for (term in observation) size += termSize(term);
    return size;
  }

  public function termSize(term:ObservationTerm):Int
    return switch term {
      case AngularVelocity(_) | ProjectedGravity(_) | VelocityCommand(_): 3;
      case JointPosition(_) | JointVelocity(_) | LastAction(_): joints.length;
      case GaitPhase(_): 2;
    };

  function validate():Void {
    var n = joints.length;
    if (n == 0) throw 'policy spec "$name": no joints';
    if (defaultPose.length != n || kp.length != n || kd.length != n)
      throw 'policy spec "$name": defaultPose, kp and kd need one value per joint ($n)';
    if (!(controlPeriod > 0.0) || !Math.isFinite(controlPeriod)) throw 'policy spec "$name": controlPeriod must be positive';
    if (historyLength < 1) throw 'policy spec "$name": historyLength must be at least 1';
    for (i in 0...n)
      if (!Math.isFinite(defaultPose[i]) || !(kp[i] >= 0.0) || !(kd[i] >= 0.0))
        throw 'policy spec "$name": joint ${joints[i]} has a non-finite pose or a negative gain';
    if (observation.length == 0) throw 'policy spec "$name": no observation terms';
    for (term in observation) {
      var scale = switch term {
        case AngularVelocity(s) | ProjectedGravity(s) | VelocityCommand(s) | JointPosition(s) | JointVelocity(s) | LastAction(s): s;
        case GaitPhase(period):
          if (!(period > 0.0)) throw 'policy spec "$name": gait period must be positive';
          [1.0];
      };
      var size = termSize(term);
      if (scale.length != 1 && scale.length != size)
        throw 'policy spec "$name": a scale needs 1 or $size values, got ${scale.length}';
    }
  }

  // ---- JSON -------------------------------------------------------------

  public static function load(path:String):PolicySpec return parse(sys.io.File.getContent(path));

  public static function parse(text:String):PolicySpec {
    var root:Dynamic = Json.parse(text);
    var version:Int = field(root, "version", -1);
    if (version != VERSION) throw 'policy spec version $version is not supported (expected $VERSION)';
    var source = new Map<String, String>();
    var sourceFields:Dynamic = Reflect.field(root, "source");
    if (sourceFields != null) for (key in Reflect.fields(sourceFields)) source.set(key, Std.string(Reflect.field(sourceFields, key)));
    var io:Dynamic = require(root, "io");
    var observation:Array<Dynamic> = require(root, "observation");
    var recurrent:Array<Dynamic> = Reflect.field(root, "recurrent");
    var imu:Dynamic = require(root, "imu");
    var hold:Dynamic = Reflect.field(root, "hold");
    var command:Dynamic = require(root, "command");
    return new PolicySpec({
      name: require(root, "name"),
      source: source,
      model: require(root, "model"),
      controlPeriod: require(root, "controlPeriod"),
      historyLength: field(root, "historyLength", 1),
      joints: strings(require(root, "joints")),
      defaultPose: floats(require(root, "defaultPose")),
      kp: floats(require(root, "kp")),
      kd: floats(require(root, "kd")),
      actionScale: require(root, "actionScale"),
      observation: [for (term in observation) parseTerm(term)],
      observationInput: require(io, "observation"),
      actionOutput: require(io, "action"),
      recurrent: recurrent == null ? [] : [for (item in recurrent) {input: require(item, "input"), output: require(item, "output")}],
      imu: {
        id: require(imu, "id"), link: require(imu, "link"),
        position: floats(field(imu, "position", [0.0, 0.0, 0.0])),
        rotation: floats(field(imu, "rotation", [0.0, 0.0, 0.0, 1.0]))
      },
      hold: hold == null ? null : {
        kp: require(hold, "kp"), kd: require(hold, "kd"),
        pose: {
          var pose = new Map<String, Float>();
          var poseFields:Dynamic = Reflect.field(hold, "pose");
          if (poseFields != null) for (key in Reflect.fields(poseFields)) pose.set(key, Reflect.field(poseFields, key));
          pose;
        }
      },
      command: {limit: floats(require(command, "limit")), acceleration: floats(require(command, "acceleration"))}
    });
  }

  static function parseTerm(term:Dynamic):ObservationTerm {
    var kind:String = require(term, "term");
    var scale = Reflect.hasField(term, "scale") ? floats(Reflect.field(term, "scale")) : [1.0];
    return switch kind {
      case "angularVelocity": AngularVelocity(scale);
      case "projectedGravity": ProjectedGravity(scale);
      case "velocityCommand": VelocityCommand(scale);
      case "jointPosition": JointPosition(scale);
      case "jointVelocity": JointVelocity(scale);
      case "lastAction": LastAction(scale);
      case "gaitPhase": GaitPhase(require(term, "period"));
      case _: throw 'unknown observation term "$kind"';
    };
  }

  static function require(object:Dynamic, name:String):Dynamic {
    if (object == null || !Reflect.hasField(object, name)) throw 'policy spec: missing "$name"';
    return Reflect.field(object, name);
  }

  static function field(object:Dynamic, name:String, fallback:Dynamic):Dynamic
    return object != null && Reflect.hasField(object, name) ? Reflect.field(object, name) : fallback;

  /** A JSON number or array of numbers. */
  static function floats(value:Dynamic):Array<Float> {
    if (Std.isOfType(value, Array)) return [for (item in (value : Array<Dynamic>)) (item : Float)];
    return [(value : Float)];
  }

  static function strings(value:Dynamic):Array<String>
    return [for (item in (value : Array<Dynamic>)) (item : String)];
}
