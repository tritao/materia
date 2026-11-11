package robotkit.kinematics;

import kinematicskit.KinematicModel;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import kinematicskit.Transform;
import robotkit.model.RobotModel;
import robotkit.runtime.RobotRuntimeIdentity;

/**
 * The effort each joint must exert to hold a robot still against gravity, from its own links' masses and centres of mass:
 * the derivative of potential energy, per runtime joint, with gravity along -Z of the robot's root frame. `actuated`
 * carries coupled joints' loads to the drives that move them by virtual work (a lead screw's carriage weight becomes
 * torque on its motor). This is a drive's model-based torque feedforward
 * (gravity compensation); it has no gains or tolerances of its own.
 */
class GravityTorques {
  public static inline final G = 9.81;

  final robot:RobotModel;
  final model:KinematicModel;
  final state:KinematicState;
  final snapshot:KinematicSnapshot;
  final runtimeDofs:Array<Int>;
  /** Each joint as a combination of root (uncoupled) joints: `coefficients[j]` maps root -> d q_j / d q_root. */
  final coefficients:Array<Map<Int, Float>>;

  public function new(robot:RobotModel, identity:RobotRuntimeIdentity) {
    this.robot = robot;
    model = RobotKinematics.compile(robot);
    state = new KinematicState(model);
    snapshot = new KinematicSnapshot(model);
    runtimeDofs = [for (i in 0...model.dofCount()) identity.jointIndex(robot.joints[model.dofJoint[i]].id)];
    var terms = [for (c in robot.couplings) {leader: identity.jointIndex(c.leader), follower: identity.jointIndex(c.follower), ratio: c.ratio}];
    var count = robot.joints.length;
    var coupled = [for (_ in 0...count) false];
    for (t in terms) coupled[t.follower] = true;
    var maps:Array<Null<Map<Int, Float>>> = [for (j in 0...count) coupled[j] ? null : [j => 1.0]];
    // A follower is the sum of its terms: compose once every one of its leaders is known.
    var pending = [for (j in 0...count) if (coupled[j]) j];
    while (pending.length > 0) {
      var progressed = false;
      for (f in pending.copy()) {
        var own = terms.filter(t -> t.follower == f);
        if (own.filter(t -> maps[t.leader] == null).length > 0) continue;
        var map = new Map<Int, Float>();
        for (t in own) {
          var leader = maps[t.leader];
          if (leader != null) for (root in leader.keys()) {
            var before:Null<Float> = map.get(root), c:Null<Float> = leader.get(root);
            map.set(root, (before == null ? 0.0 : before) + t.ratio * (c == null ? 0.0 : c));
          }
        }
        maps[f] = map;
        pending.remove(f);
        progressed = true;
      }
      if (!progressed) throw "Joint couplings form a cycle";
    }
    coefficients = [for (m in maps) m == null ? new Map<Int, Float>() : m];
  }

  /** Places the robot's root (metres) when it is not at the world origin, so gravity is measured in the right frame. */
  public function mount(root:Transform):Void state.setRootPose(model.bodyIndex(robot.links[0].id), root);

  /**
   * The holding effort of every joint on its own (runtime joint order; N m or N), for joint positions `q` in runtime order.
   * `payloads` add point masses carried by links (runtime link index, world point in metres, kg).
   */
  public function torques(q:Array<Float>, ?payloads:Array<{link:Int, point:Array<Float>, mass:Float}>):Array<Float> {
    for (i in 0...runtimeDofs.length) state.q[i] = q[runtimeDofs[i]];
    snapshot.evaluate(state);
    var result = [for (_ in 0...q.length) 0.0], n = model.dofCount();
    function add(body:Int, x:Float, y:Float, z:Float, mass:Float):Void {
      var jac = snapshot.pointJacobian(body, x, y, z);
      for (i in 0...n) result[runtimeDofs[i]] += mass * G * jac[2 * n + i];
    }
    for (link in robot.links) if (link.mass > 0) {
      var body = model.bodyIndex(link.id), c = link.centerOfMass, point = snapshot.bodyPose(body).transformPoint(c[0], c[1], c[2]);
      add(body, point.x, point.y, point.z, link.mass);
    }
    if (payloads != null) for (payload in payloads)
      add(model.bodyIndex(robot.links[payload.link].id), payload.point[0], payload.point[1], payload.point[2], payload.mass);
    return result;
  }

  /**
   * The efforts the drives on `driven` joints must supply to hold the robot at `q` (runtime order; zero elsewhere). Each
   * joint's load is summed onto the root joints it moves with (by virtual work through the couplings); the drives' efforts
   * are the least-norm efforts that produce those root loads, so a lead screw's motor carries its carriage's weight and two
   * motors on one axis share it. Root joints no drive moves are left to whatever holds them.
   */
  public function actuated(q:Array<Float>, driven:Array<Int>, ?payloads:Array<{link:Int, point:Array<Float>, mass:Float}>):Array<Float> {
    var tau = torques(q, payloads);
    var roots:Array<Int> = [];
    for (d in driven) for (root in coefficients[d].keys()) if (roots.indexOf(root) < 0) roots.push(root);
    var result = [for (_ in 0...tau.length) 0.0];
    if (roots.length == 0) return result;
    var load = [for (r in roots) {
      var sum = 0.0;
      for (j in 0...tau.length) if (tau[j] != 0) sum += coefficient(j, r) * tau[j];
      sum;
    }];
    // A (roots x drives): a drive's effort per unit of each root it moves. Solve (A A^T) y = load, efforts = A^T y.
    var a = [for (r in roots) [for (d in driven) coefficient(d, r)]];
    var n = roots.length, m = [for (i in 0...n) [for (k in 0...n) {
      var sum = 0.0;
      for (c in 0...driven.length) sum += a[i][c] * a[k][c];
      sum;
    }]];
    var y = solve(m, load);
    for (c in 0...driven.length) {
      var effort = 0.0;
      for (i in 0...n) effort += a[i][c] * y[i];
      result[driven[c]] = effort;
    }
    return result;
  }

  /** d q_joint / d q_root: how far `joint` moves per unit of `root` (zero when they are not coupled). */
  function coefficient(joint:Int, root:Int):Float {
    var c:Null<Float> = coefficients[joint].get(root);
    return c == null ? 0.0 : c;
  }

  /** Gaussian elimination with partial pivoting; a root no drive can move gets no effort. */
  static function solve(matrix:Array<Array<Float>>, rhs:Array<Float>):Array<Float> {
    var n = rhs.length, m = [for (row in matrix) row.copy()], b = rhs.copy();
    for (col in 0...n) {
      var best = col;
      for (row in col + 1...n) if (Math.abs(m[row][col]) > Math.abs(m[best][col])) best = row;
      var swap = m[col];
      m[col] = m[best];
      m[best] = swap;
      var t = b[col];
      b[col] = b[best];
      b[best] = t;
      if (Math.abs(m[col][col]) < 1e-15) continue;
      for (row in 0...n) if (row != col) {
        var f = m[row][col] / m[col][col];
        if (f == 0) continue;
        for (k in col...n) m[row][k] -= f * m[col][k];
        b[row] -= f * b[col];
      }
    }
    return [for (i in 0...n) Math.abs(m[i][i]) < 1e-15 ? 0.0 : b[i] / m[i][i]];
  }

  /** Potential energy (J) at `q`: its derivative is `torques`, which `check` compares. */
  function potential(q:Array<Float>):Float {
    for (i in 0...runtimeDofs.length) state.q[i] = q[runtimeDofs[i]];
    snapshot.evaluate(state);
    var result = 0.0;
    for (link in robot.links) {
      var c = link.centerOfMass, point = snapshot.bodyPose(model.bodyIndex(link.id)).transformPoint(c[0], c[1], c[2]);
      result += link.mass * G * point.z;
    }
    return result;
  }

  /** Compares `torques` at `q` with a central difference of the potential for `joints`; the count checked. */
  public function check(q:Array<Float>, joints:Array<Int>):Int {
    var analytic = torques(q), epsilon = 0.00001;
    for (j in joints) {
      var plus = q.copy(), minus = q.copy();
      plus[j] += epsilon;
      minus[j] -= epsilon;
      var derivative = (potential(plus) - potential(minus)) / (2 * epsilon);
      if (!Math.isFinite(derivative) || Math.abs(derivative - analytic[j]) > Math.max(0.00001, Math.abs(derivative) * 0.000001))
        throw "Gravity torque differs from the potential-energy derivative";
    }
    return joints.length;
  }
}
