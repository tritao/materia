import kinematicskit.JacobianLayout;
import kinematicskit.JointKind;
import kinematicskit.KinematicModel;
import kinematicskit.KinematicModelBuilder;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import kinematicskit.Transform;
import kinematicskit.Vector3;
import kinematicskit.native.NativeKinematics;

class NativeKinematicsTests {
  static var assertions = 0;

  public static function main():Void {
    testForwardAndJacobianParity();
    testRejectsBadInput();
    Sys.println('KinematicsKit native tests passed ($assertions assertions)');
  }

  static function testForwardAndJacobianParity():Void {
    var worstPose = 0.0, worstJacobian = 0.0;
    for (seed in 1...6) {
      var rng = new Rng(seed);
      var model = randomForest(rng);
      var native = new NativeKinematics(model);
      var snapshot = new KinematicSnapshot(model);
      var n = model.dofCount();
      for (_ in 0...20) {
        var state = new KinematicState(model, [for (_ in 0...n) rng.signed() * 2.0]);
        snapshot.evaluate(state);
        var poses = native.forward(state.q);
        for (body in 0...model.bodyCount()) {
          var expected = snapshot.bodyPose(body);
          var values = [expected.x, expected.y, expected.z, expected.qx, expected.qy, expected.qz, expected.qw];
          for (k in 0...7) worstPose = Math.max(worstPose, Math.abs(poses[body * 7 + k] - values[k]));
        }
        // A subset of columns, in a shuffled order, at a point off each body.
        var columns = [for (dof in 0...n) if (rng.signed() > -0.4) dof];
        columns.reverse();
        if (columns.length == 0) columns = [0];
        var layout = new JacobianLayout(model, columns);
        var expectedJacobian = [for (_ in 0...6 * columns.length) 0.0];
        for (body in 0...model.bodyCount()) {
          var point = [rng.signed(), rng.signed(), rng.signed()];
          snapshot.pointJacobianColumns(body, point[0], point[1], point[2], layout, expectedJacobian);
          var got = native.pointJacobian(state.q, body, point, columns);
          for (i in 0...expectedJacobian.length) worstJacobian = Math.max(worstJacobian, Math.abs(got[i] - expectedJacobian[i]));
        }
      }
      native.dispose();
    }
    check(worstPose == 0.0, 'native forward kinematics is bit-identical to the Haxe snapshot (worst $worstPose)');
    check(worstJacobian == 0.0, 'native Jacobians are bit-identical to the Haxe snapshot (worst $worstJacobian)');
  }

  static function testRejectsBadInput():Void {
    var model = randomForest(new Rng(9));
    var native = new NativeKinematics(model);
    rejects(() -> native.forward([0.0]), "a wrong DOF count is rejected");
    rejects(() -> native.forward([for (_ in 0...model.dofCount()) Math.NaN]), "non-finite joint values are rejected");
    rejects(() -> native.pointJacobian([for (_ in 0...model.dofCount()) 0.0], model.bodyCount(), [0.0, 0.0, 0.0], [0]),
      "an out-of-range body is rejected");
    rejects(() -> native.pointJacobian([for (_ in 0...model.dofCount()) 0.0], 0, [0.0, 0.0, 0.0], [0, 0]),
      "a repeated column is rejected");
    native.dispose();
    rejects(() -> native.forward([for (_ in 0...model.dofCount()) 0.0]), "a disposed model is rejected");
    var ints = NativeKinematics.packInts(model), reals = NativeKinematics.packReals(model);
    ints[0] = 2;
    check(KinematicsKitNative.kk_model_create(ints, reals).status == KinematicsKitNativeConstants.KK_ERROR_INVALID_ARGUMENT,
      "an unknown packed format is rejected");
  }

  /** Two roots; a chain with revolute, prismatic, fixed and coupled joints; a side branch. */
  static function randomForest(rng:Rng):KinematicModel {
    var builder = new KinematicModelBuilder();
    var bodies = [builder.addBody("root")];
    var kinds = [JointKind.Revolute, JointKind.Prismatic, JointKind.Fixed, JointKind.Revolute, JointKind.Revolute,
      JointKind.Prismatic, JointKind.Revolute];
    for (i in 0...kinds.length) {
      var body = builder.addBody('b$i');
      var parent = i == 5 ? bodies[2] : bodies[bodies.length - 1];
      builder.addJoint('j$i', kinds[i], parent, body, randomTransform(rng), randomTransform(rng),
        new Vector3(rng.signed(), rng.signed(), rng.signed() + 0.1));
      bodies.push(body);
    }
    builder.couple("j4", "j3", 0.75, 0.1);
    var other = builder.addBody("other_root");
    builder.setRootPose(other, randomTransform(rng));
    var arm = builder.addBody("other_arm");
    builder.addJoint("k0", JointKind.Revolute, other, arm, randomTransform(rng), randomTransform(rng), new Vector3(0, 0, 1));
    builder.addFrame("tip", bodies[bodies.length - 1], randomTransform(rng));
    return builder.build();
  }

  static function randomTransform(rng:Rng):Transform {
    var ax = rng.signed(), ay = rng.signed(), az = rng.signed() + 0.1;
    var norm = Math.sqrt(ax * ax + ay * ay + az * az);
    return new Transform(rng.signed(), rng.signed(), rng.signed(), 0.0, 0.0, 0.0, 1.0)
      .compose(Transform.axisAngle(ax / norm, ay / norm, az / norm, rng.signed() * 2.0));
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw 'assertion failed: $message';
  }

  static function rejects(action:Void->Void, message:String):Void {
    var threw = false;
    try action() catch (_:Dynamic) threw = true;
    check(threw, message);
  }
}

private class Rng {
  var state:Int;
  public function new(seed:Int) state = seed;
  public function signed():Float {
    state = (state * 1103515245 + 12345) & 0x7fffffff;
    return state / 2147483647.0 * 2.0 - 1.0;
  }
}
