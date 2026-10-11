package tests;

import RobotKitRuntime;
import haxe.Int64;
import robotkit.runtime.RobotSnapshot;
import robotkit.model.CollisionShape;
import robotkit.model.CollisionShape.CollisionPrimitive;
import robotkit.model.CollisionShape.ContactSurface;
import robotkit.model.CollisionShape.ShapeContact;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.Sensor;
import robotkit.profile.RobotProfile;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.SimulationHarness;

/** Sheet pressure closes a passive finger switch; gravity resets it after stock removal. */
class PressFingerContactTests {
  public static function main():Void {
    var assertions = run();
    Sys.println('Press finger contact tests passed ($assertions assertions)');
  }

  /** MuJoCo resolves contact for both authored primitives and CAD-style convex hulls. */
  public static function run():Int {
    for (timestep in [0.001, 0.01]) {
      exercise(false, timestep);
      exercise(true, timestep);
    }
    return 12;
  }

  static function exercise(useHull:Bool, timestep:Float):Void {
    var model = new RobotModel("press-finger-contact");
    model.collisionApproximation = robotkit.model.CollisionApproximation.None;
    var base = model.addLink(new Link("base"));
    var finger = model.addLink(new Link("finger"));
    finger.mass = 0.020 * 0.060 * 0.021 * 7850;
    finger.centerOfMass = [0, -0.020, 0.0105];
    var ix = finger.mass * (0.060 * 0.060 + 0.021 * 0.021) / 12;
    var iy = finger.mass * (0.020 * 0.020 + 0.021 * 0.021) / 12;
    var iz = finger.mass * (0.020 * 0.020 + 0.060 * 0.060) / 12;
    finger.inertiaTensor = [ix, 0, 0, 0, iy, 0, 0, 0, iz];
    var shape = new CollisionShape(Box(0.010, 0.030, 0.0105), finger.centerOfMass.copy());
    var surface = new ContactSurface(null, 0, 0, 1);
    shape.surface = surface;
    shape.contact = ShapeContact.PairsAndEnvironment;
    var vertices:Array<Float> = [];
    for (x in [-0.010, 0.010]) for (y in [-0.050, 0.010]) for (z in [0.0, 0.021]) {
      vertices.push(x); vertices.push(y); vertices.push(z);
    }
    if (useHull) {
      shape = new CollisionShape(collisionkit.CollisionGeometry.Convex(vertices));
      shape.surface = surface;
      shape.contact = ShapeContact.PairsAndEnvironment;
    }
    finger.collisionShapes.push(shape);
    var hinge = model.addJoint(new Joint("finger-hinge", JointType.Revolute, base, finger));
    hinge.axis = [-1, 0, 0]; hinge.parentFramePosition = [0, 0.050, 1];
    hinge.limits.lower = -Math.PI / 360; hinge.limits.upper = 0;
    hinge.limits.overtravel = 0.001;
    // The stop stores the entire gravity-drop energy within its declared compression.
    var drop = -hinge.limits.lower;
    var energy = finger.mass * 9.80665 * (0.0105 - (-0.020 * Math.sin(drop) + 0.0105 * Math.cos(drop)));
    var stiffness = 2 * energy / (hinge.limits.overtravel * hinge.limits.overtravel);
    var hingeInertia = ix + finger.mass * (0.020 * 0.020 + 0.0105 * 0.0105);
    hinge.limitTimeConstant = Math.sqrt(hingeInertia / stiffness);
    hinge.limitDampingRatio = 1;
    surface.contactTimeConstant = hinge.limitTimeConstant;
    hinge.passive = true;
    var contact = model.addSensor(new Sensor("finger-contact", "joint_switch"));
    contact.joint = hinge.id; contact.windowLower = -0.001; contact.windowUpper = 0.001;
    contact.hysteresis = 0.0001;
    var blueprint = RobotRuntimeCompiler.compile(model, new RobotProfile());
    var harness = new SimulationHarness(timestep, 1, 1);
    var periodNs = Std.int(timestep * 1e9);
    var restTicks = Std.int(1.0 / timestep);
    var advanceTicks = Std.int(0.02 / timestep);
    var tick = 0;
    try {
      var runtime = harness.simulation.addRobotAtPose(blueprint, [0, 0, 0], [0, 0, 0, 1],
        null, null, null, null, null, null, null, true);
      var substeps:Int = @:privateAccess harness.space.requiredSubsteps;
      Sys.println('Finger integration hull=$useHull tick=$timestep substeps=$substeps');
      for (_ in 0...restTicks) {
        harness.step(Int64.ofInt(++tick * periodNs));
        requireHealthy(runtime.snapshot(), 'gravity, tick=$timestep step=$tick');
      }
      var resting = runtime.snapshot();
      check(resting.q.get(0) < -0.006 && signal(resting) == 0,
        'gravity opens the contact switch: tick=$timestep q=${resting.q.get(0)} fault=${resting.faultCode} safety=${resting.safety}');
      var sheet = harness.spawnBox([0, -0.052, 1.02075], [0.020, 0.050, 0.00075]);
      for (advance in 0...20) {
        var tip = -0.002 + 0.002 * (advance + 1) / 20;
        harness.teleportObject(sheet, [0, tip - 0.050, 1.02075]);
        for (_ in 0...advanceTicks) {
          harness.step(Int64.ofInt(++tick * periodNs));
          requireHealthy(runtime.snapshot(), 'stock contact, tick=$timestep step=$tick');
        }
      }
      var loaded = runtime.snapshot();
      Sys.println('Finger contact hull=$useHull tick=$timestep: resting=${resting.q.get(0)} loaded=${loaded.q.get(0)} switch=${signal(loaded)}');
      check(loaded.q.get(0) > -0.001 && signal(loaded) == 1, "sheet contact raises the finger and closes its switch");
      harness.removeObject(sheet);
      for (_ in 0...restTicks) {
        harness.step(Int64.fromFloat(++tick * (periodNs * 1.0)));
        requireHealthy(runtime.snapshot(), 'gravity return, tick=$timestep step=$tick');
      }
      var released = runtime.snapshot();
      check(released.q.get(0) < -0.006 && signal(released) == 0, "removing stock lets gravity reset the switch");
      harness.dispose();
    } catch (error:Dynamic) { harness.dispose(); throw error; }
  }
  static function requireHealthy(snapshot:RobotSnapshot, phase:String):Void {
    check(snapshot.faultCode == 0 && snapshot.safety == RobotKitRuntimeConstants.RK_SAFETY_READY,
      'Finger runtime failed during $phase: fault=${snapshot.faultCode}, safety=${snapshot.safety}');
  }
  static function signal(snapshot:RobotSnapshot):Float {
    for (i in 0...snapshot.sensors.length) {
      var sample = snapshot.sensors.get(i);
      if (sample.sensorId == "finger-contact") return sample.values.get(0);
    }
    throw "Missing measured finger switch";
  }
  static function check(condition:Bool, message:String):Void {
    if (!condition) throw message;
  }
}
