import haxe.io.Bytes;
import robotkit.model.Actuator;
import robotkit.model.ActuatorDrive.HydraulicDrive;
import robotkit.model.ActuatorDrive.CylinderDrive;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.RobotModelCodec;
import robotkit.model.Transmission;
import robotkit.profile.RobotProfile;
import robotkit.runtime.CylinderControl;
import robotkit.runtime.RobotRuntimeCompiler;

/** Hydraulic process data stays typed from SI force geometry to its execution blueprint. */
class HydraulicModelTests {
  static var assertions:Int = 0;
  public static function main():Void run();
  static function drive(home:Float = 0, sign:Float = 1):HydraulicDrive
    return new HydraulicDrive(0.160, 0.100, 0.100, 10e6, 0.080, 0.008, 0.070, 0.0685,
      "valve/coilA", "valve/coilB", sign, home);
  static function model(home:Float = 0, type:JointType = JointType.Prismatic):RobotModel {
    var result = new RobotModel("hydraulic-reference");
    var base = result.addLink(new Link("base")), ram = result.addLink(new Link("ram"));
    var joint = result.addJoint(new Joint("ram", type, base, ram));
    joint.limits.lower = home; joint.limits.upper = home + 0.1;
    var hydraulic = drive(home);
    var actuator = new Actuator("cylinder", hydraulic.extensionForce(), hydraulic.ratedSpeed,
      Transmission.SimpleTransmission(joint.id, 1, 0));
    actuator.drive = hydraulic; result.addActuator(actuator);
    return result;
  }
  public static function run():Void {
    assertions = 0;
    forceAndBlueprint();
    processPlanning();
    savedDrive();
    invalidBindings();
    passiveOwnership();
    Sys.println('Hydraulic model tests passed ($assertions assertions)');
  }
  static function forceAndBlueprint():Void {
    var hydraulic = drive();
    near(hydraulic.extensionForce(), 201061.92982974676, "full-bore force in N");
    near(hydraulic.retractionForce(), 122522.11349000193, "annular force in N");
    near(hydraulic.extensionSpeed(0.068499), 0.08, "approach before change point");
    near(hydraulic.extensionSpeed(0.0685), 0.008, "working at change point");
    near(hydraulic.returningSpeed(), 0.07, "return speed in m/s");
    cylinderCapability(hydraulic);
    var source = model();
    var blueprint = RobotRuntimeCompiler.compile(source, new RobotProfile());
    check(blueprint.cylinderDrives.length == 1 && blueprint.channels.length == 2, "one shared process binding and two coils");
    var binding = blueprint.cylinderDrives[0];
    check(binding.control == CylinderControl.FlowLimited, "hydraulic binding selects flow control");
    near(binding.extensionForce, hydraulic.extensionForce(), "compiled extension force");
    near(binding.retractionForce, hydraulic.retractionForce(), "compiled return force");
    near(binding.approachSpeed, 0.08, "compiled approach speed");
    near(binding.workingSpeed, 0.008, "compiled working speed");
    near(binding.returnSpeed, 0.07, "compiled return speed");
    near(binding.changePoint, 0.0685, "compiled change travel");
  }
  static function processPlanning():Void {
    var source = model();
    var effort = source.actuators[0].planningEffort();
    var rate = source.actuators[0].planningRate();
    check(effort == null && rate == null, "hydraulic process joints do not enter motion planning");
  }
  static function cylinderCapability(hydraulic:HydraulicDrive):Void {
    cylinderForce(hydraulic.cylinder(), hydraulic.extensionForce());
  }
  static function cylinderForce(cylinder:Null<CylinderDrive>, expected:Float):Void {
    if (cylinder == null) throw "Hydraulic drive lost its cylinder capability";
    near(cylinder.extensionForce(), expected, "shared cylinder capability");
  }
  static function savedDrive():Void {
    var hydraulic = drive();
    var source = model();
    var encoded = RobotModelCodec.encode(source);
    var restored = RobotModelCodec.decode(encoded);
    var restoredDrive = restored.actuators[0].drive;
    if (restoredDrive == null) throw "Saved hydraulic actuator lost its drive";
    var restoredHydraulic = restoredDrive.hydraulic();
    if (restoredHydraulic == null) throw "Saved hydraulic actuator lost its typed hydraulic data";
    check(restored.schemaVersion == robotkit.model.RobotModel.CURRENT_VERSION && restoredDrive.kind() == "hydraulic", "one current RobotModel format");
    near(restoredHydraulic.changePoint, 0.0685, "saved phase boundary");
    near(restoredHydraulic.extensionForce(), hydraulic.extensionForce(), "saved pressure-area force");
    check(restoredHydraulic.channelA == "valve/coilA" && restoredHydraulic.channelB == "valve/coilB",
      "saved valve channels");
    var text = encoded.toString();
    check(text.indexOf('"schemaVersion":' + robotkit.model.RobotModel.CURRENT_VERSION) >= 0, "serialized current version");
    for (version in [9, 10, 11, 12]) {
      var old = Bytes.ofString(StringTools.replace(text, '"schemaVersion":' + robotkit.model.RobotModel.CURRENT_VERSION, '"schemaVersion":' + version));
      reject(function() { var invalid = RobotModelCodec.decode(old); }, "older RobotModel rejected");
    }
  }
  static function invalidBindings():Void {
    var posed = RobotRuntimeCompiler.compile(model(-0.025), new RobotProfile());
    near(posed.cylinderDrives[0].retractedPosition, -0.025, "edited placement preserves the retracted coordinate");
    var shortGuide = model(); shortGuide.joints[0].limits.upper = 0.099;
    diagnosed(shortGuide, "RK_HYDRAULIC_TRAVEL");
    var wrongHome = model(); wrongHome.joints[0].limits.lower = 0.01; wrongHome.joints[0].limits.upper = 0.11;
    diagnosed(wrongHome, "RK_HYDRAULIC_HOME");
    var rotating = model(0, JointType.Revolute);
    diagnosed(rotating, "RK_CYLINDER_JOINT");
    reject(function() {
      var invalid = new HydraulicDrive(0.16, 0.1, 0.1, 10e6, 0.08, 0, 0.07, 0.0685, "A", "B", 1);
    }, "zero working speed");
    reject(function() {
      var invalid = new HydraulicDrive(0.16, 0.1, 0.1, 10e6, 0.08, 0.008, 0.07, 0.101, "A", "B", 1);
    }, "change point outside stroke");
    reject(function() {
      var invalid = new HydraulicDrive(0.16, 0.1, 0.1, 10e6, 0.08, 0.008, 0.07, 0.0685, "A", "A", 1);
    }, "coils need distinct channels");
  }
  static function passiveOwnership():Void {
    var source = model();
    source.joints[0].passive = true;
    diagnosed(source, "RK_PASSIVE_ACTUATOR");
    reject(function() { var encoded = RobotModelCodec.encode(source); },
      "passive joint cannot retain a cylinder actuator");
    source.actuators.resize(0);
    var restored = RobotModelCodec.decode(RobotModelCodec.encode(source));
    check(restored.joints[0].passive, "saved passive ownership");
    var blueprint = RobotRuntimeCompiler.compile(restored, new RobotProfile());
    check(blueprint.joints[0].passive && blueprint.cylinderDrives.length == 0,
      "passive ownership reaches runtime without a drive");
    source.joints[0].passive = false;
    var controlled = RobotModelCodec.decode(RobotModelCodec.encode(source));
    check(!controlled.joints[0].passive, "ordinary joints retain host ownership");
  }
  static function diagnosed(model:RobotModel, code:String):Void {
    var found = false;
    for (diagnostic in RobotRuntimeCompiler.validate(model, new RobotProfile())) if (diagnostic.code == code) found = true;
    check(found, 'missing $code diagnosis');
  }
  static function near(actual:Float, expected:Float, message:String):Void
    check(Math.isFinite(actual) && Math.abs(actual - expected) <= 1e-9 * Math.max(1, Math.abs(expected)), message);
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
  static function reject(action:Void->Void, message:String):Void {
    var failed = false;
    try action() catch (_:Dynamic) failed = true;
    check(failed, message);
  }
}
