import collisionkit.CollisionDistance;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionMargins;
import collisionkit.CollisionPair;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import collisionkit.CollisionPose;
import collisionkit.CollisionWorld;
import collisionkit.native.NativeCollisionWorld;
import collisionkit.CollisionBuild;
import collisionkit.kinematics.ModelBodies;
import kinematicskit.AvoidancePair;
import kinematicskit.BodyPairRelation;
import kinematicskit.FrameOrientation;
import kinematicskit.FrameTask;
import kinematicskit.KinematicProblem;
import kinematicskit.StepLimits;
import kinematicskit.native.DifferentialIk;
import kinematicskit.native.NativeQpStep;
import kinematicskit.ClosureKind;
import kinematicskit.JointKind;
import kinematicskit.KinematicBodyPairs;
import kinematicskit.KinematicModel;
import kinematicskit.KinematicModelBuilder;
import kinematicskit.KinematicSnapshot;
import kinematicskit.KinematicState;
import kinematicskit.Transform;
import kinematicskit.Vector3;

/**
 * collisionkit's native world from Haxe (COLLISION.md CL2): bodies posed
 * from kinematics snapshots, the kit's rigid, adjacent and closure pairs as
 * rules, several models in one world, held parts, overlap at reference
 * within one articulation, inflation, violations and batched checks.
 */
class NativeCollisionTests {
  static var assertions = 0;

  public static function main():Void {
    testBodyPairsForCollision();
    testWorldFollowsTheSnapshot();
    testStatusesAndChanges();
    testTwoModelsInOneWorld();
    testInflationAgainstExactDistances();
    testViolationsAndBatches();
    testRefusedHeights();
    testServoStopsShortAndSlides();
    testAvoidsAnotherRobot();
    testRecoversFromInsideTheMargin();
    testRelaxesRowsThatCannotHold();
    Sys.println('CollisionKit native tests passed ($assertions assertions)');
  }

  /** Rigid, adjacent and closure pairs, with their reasons, from the structure alone (COLLISION.md CL-D3). */
  static function testBodyPairsForCollision():Void {
    var builder = new KinematicModelBuilder();
    var base = builder.addBody("base");
    var upper = builder.addBody("upper");
    var fore = builder.addBody("fore");
    var tool = builder.addBody("tool");
    var rod = builder.addBody("rod");
    var z = new Vector3(0.0, 0.0, 1.0);
    builder.addJoint("shoulder", JointKind.Revolute, base, upper, Transform.identity(), Transform.identity(), z);
    builder.addJoint("elbow", JointKind.Revolute, upper, fore, Transform.translation(1, 0, 0), Transform.identity(), z);
    builder.addJoint("flange", JointKind.Fixed, fore, tool, Transform.translation(1, 0, 0), Transform.identity());
    builder.addJoint("rod", JointKind.Revolute, base, rod, Transform.translation(0, 1, 0), Transform.identity(), z);
    var tip = builder.addFrame("tip", tool, Transform.identity());
    var end = builder.addFrame("end", rod, Transform.translation(2, -1, 0));
    builder.addClosure("loop", ClosureKind.Revolute, tip, end, z, 1e-3);
    var model = builder.build();
    var found = [for (pair in KinematicBodyPairs.of(model)) '${pair.a}-${pair.b}:${(pair.relation : Int)}'];
    var expected = ['$base-$upper:1', '$base-$rod:1', '$upper-$fore:1', '$upper-$tool:1', '$fore-$tool:0',
      '$fore-$rod:2', '$tool-$rod:2'];
    check(found.join(",") == expected.join(","), 'body pairs for collision (${found.join(",")})');
    var groups = KinematicBodyPairs.rigidGroups(model);
    check(groups[tool] == fore && groups[fore] == fore && groups[rod] == rod, "rigid groups follow fixed joints");

    // As world rules, each with its reason.
    var world = new NativeCollisionWorld();
    var first = register(world, model, 0);
    var spheres = [for (body in 0...model.bodyCount()) world.add(first + body, CollisionPose.identity(),
      CollisionGeometry.Sphere(0.01))];
    check(world.pairStatus(spheres[fore], spheres[tool]) == CollisionPairStatus.Rigid, "rigid by the kit");
    check(world.pairStatus(spheres[upper], spheres[tool]) == CollisionPairStatus.Adjacent, "adjacent by the kit");
    check(world.pairStatus(spheres[tool], spheres[rod]) == CollisionPairStatus.Closure, "closure by the kit");
    check(world.pairStatus(spheres[base], spheres[fore]) == CollisionPairStatus.Checked, "unrelated bodies checked");
    world.dispose();
  }

  /**
   * A sphere on every body of random trees, against a sphere fixed in the
   * world, posed from the Haxe snapshot: the native distance is the distance
   * between the sphere centres minus the radii, and the closest points lie
   * on that line.
   */
  static function testWorldFollowsTheSnapshot():Void {
    var worst = 0.0, worstPoint = 0.0;
    for (seed in 11...14) {
      var rng = new Rng(seed);
      var model = randomForest(rng);
      var world = new NativeCollisionWorld();
      var first = register(world, model, 0);
      var snapshot = new KinematicSnapshot(model);
      var offsets = [for (_ in 0...model.bodyCount()) Transform.translation(rng.signed(), rng.signed(), rng.signed())];
      var spheres = [for (body in 0...model.bodyCount()) world.add(first + body, pose(offsets[body]),
        CollisionGeometry.Sphere(0.05))];
      var target = world.add(-1, CollisionPose.translation(0.3, -0.2, 0.4), CollisionGeometry.Sphere(0.1));
      for (_ in 0...10) {
        var state = new KinematicState(model, [for (_ in 0...model.dofCount()) rng.signed() * 2.0]);
        snapshot.evaluate(state);
        place(world, first, model, snapshot);
        var found = world.distances(100.0);
        for (body in 0...model.bodyCount()) {
          var centre = snapshot.bodyPose(body).compose(offsets[body]);
          var expected = Math.sqrt(Math.pow(centre.x - 0.3, 2) + Math.pow(centre.y + 0.2, 2) + Math.pow(centre.z - 0.4, 2))
            - 0.15;
          var row:Null<CollisionDistance> = null;
          for (candidate in found) if (candidate.a == spheres[body] && candidate.b == target) row = candidate;
          if (row == null) throw 'body $body sphere is not within the query';
          check(row.bodyA == first + body && row.bodyB == -1, "results name the bodies");
          worst = Math.max(worst, Math.abs(row.distance - expected));
          // The closest point on the body's sphere is 0.05 from its centre, toward the target.
          var gap = Math.sqrt(Math.pow(row.pointA[0] - centre.x, 2) + Math.pow(row.pointA[1] - centre.y, 2)
            + Math.pow(row.pointA[2] - centre.z, 2));
          worstPoint = Math.max(worstPoint, Math.abs(gap - 0.05));
        }
      }
      world.dispose();
    }
    check(worst < 1e-9, 'collision distances follow the snapshot (worst $worst)');
    check(worstPoint < 1e-9, 'closest points lie on the spheres (worst $worstPoint)');
  }

  /** Pair statuses from the kit's pairs, declared rules, reference overlap, re-attaching and terrain. */
  static function testStatusesAndChanges():Void {
    var model = twoLinkArm();
    var base = 0, upper = 1, fore = 2, tool = 3;
    var world = new NativeCollisionWorld();
    var first = register(world, model, 0);
    var arm = [for (body in 0...model.bodyCount()) first + body];
    var baseBox = world.add(base, CollisionPose.identity(), CollisionGeometry.Box(0.1, 0.1, 0.1));
    var upperSphere = world.add(upper, CollisionPose.translation(0.5, 0, 0), CollisionGeometry.Sphere(0.1));
    var foreCapsule = world.add(fore, CollisionPose.translation(0.5, 0, 0), CollisionGeometry.Capsule(0.05, 0.2));
    var toolBox = world.add(tool, CollisionPose.identity(), CollisionGeometry.Box(0.1, 0.1, 0.1));
    var obstacle = world.add(-1, CollisionPose.translation(2.5, 0, 0), CollisionGeometry.Box(0.2, 0.2, 0.2));
    var floor = world.add(-1, CollisionPose.translation(0, 0, -1),
      CollisionGeometry.HeightField(4, 4, [for (_ in 0...9) 0.0], 3, -1));

    check(world.pairStatus(foreCapsule, toolBox) == CollisionPairStatus.Rigid, "forearm and tool are rigid");
    check(world.pairStatus(upperSphere, toolBox) == CollisionPairStatus.Adjacent, "upper arm and tool are adjacent");
    check(world.pairStatus(obstacle, floor) == CollisionPairStatus.Static, "obstacle and floor are static");
    check(world.pairStatus(baseBox, foreCapsule) == CollisionPairStatus.Checked, "base and forearm are checked");

    var snapshot = new KinematicSnapshot(model);
    var state = new KinematicState(model, [0.0, 0.0]);
    snapshot.evaluate(state);
    place(world, first, model, snapshot);
    check(world.colliding(0.0).length == 0, "nothing collides stretched out");
    var near = world.distances(0.5);
    check(near.length == 1 && near[0].a == toolBox && near[0].b == obstacle, "only the tool is near the obstacle");
    check(Math.abs(near[0].distance - 0.2) < 1e-9, "tool-obstacle clearance");
    check(Math.abs(near[0].normal[0] - 1.0) < 1e-9, "normal from tool to obstacle");

    // Grasp the obstacle: it rides on the tool and follows the tool's rules.
    world.attach(obstacle, tool, CollisionPose.translation(0.3, 0, 0));
    check(world.pairStatus(foreCapsule, obstacle) == CollisionPairStatus.Rigid, "a grasped part is rigid with the tool");
    check(world.pairStatus(upperSphere, obstacle) == CollisionPairStatus.Adjacent, "and adjacent to the upper arm");
    snapshot.evaluate(new KinematicState(model, [0.0, Math.PI / 2]));
    place(world, first, model, snapshot);
    var held = world.distances(10.0).filter(d -> d.a == baseBox && d.b == obstacle);
    // With the elbow bent the held box is centred at (1, 1.3, 0): 0.7 along x and 1.0 along y from the base box.
    check(held.length == 1 && Math.abs(held[0].distance - Math.sqrt(0.7 * 0.7 + 1.0 * 1.0)) < 1e-9,
      'the held part follows the tool (${held.length == 1 ? held[0].distance : -1})');

    // Raise the terrain to z = 0, into every link but the held part.
    world.setHeights(floor, [for (_ in 0...9) 1.0]);
    snapshot.evaluate(state);
    place(world, first, model, snapshot);
    var hits = world.colliding(0.0);
    check(hits.length > 0 && hits.filter(p -> p.b == floor).length == hits.length, "the raised terrain collides");
    world.setObjectRule(baseBox, floor, CollisionPairRule.Allow, CollisionPairStatus.Allowed);
    check(world.pairStatus(baseBox, floor) == CollisionPairStatus.Allowed, "allowed by rule");
    // Overlap at reference covers the arm's own pairs only: terrain through the arm is a layout error.
    var marked = world.allowOverlapping(arm);
    check(marked == 0, 'no arm pair overlaps at reference ($marked)');
    check(world.colliding(0.0).length == hits.length - 1, "the environment overlaps stay checked");
    check(world.pairStatus(upperSphere, floor) == CollisionPairStatus.Checked, "terrain through the arm is checked");
    // Within the arm, an overlap at reference is marked.
    var sleeve = world.add(fore, CollisionPose.translation(-0.5, 0, 0), CollisionGeometry.Sphere(0.05));
    world.setBodyRule(upper, fore, CollisionPairRule.Default, CollisionPairStatus.Checked);
    check(world.allowOverlapping(arm) == 1, "the sleeve overlaps the upper arm at reference");
    check(world.pairStatus(upperSphere, sleeve) == CollisionPairStatus.OverlapsAtReference, "marked");
    world.remove(obstacle);
    rejects(() -> world.pairStatus(obstacle, floor), "a removed object has no status");
    world.dispose();
    rejects(() -> world.setBodyPose(0, CollisionPose.identity()), "a disposed world is refused");
  }

  /**
   * Two arms, each its own model, in one world: each keeps its own rules,
   * the pairs between them are checked, and each is posed from its own
   * snapshot.
   */
  static function testTwoModelsInOneWorld():Void {
    var left = twoLinkArm(), right = twoLinkArm(Transform.translation(4.2, 0, 0).compose(Transform.axisAngle(0, 0, 1, Math.PI)));
    var world = new NativeCollisionWorld();
    var a = register(world, left, 0), b = register(world, right, 0);
    check(a == 0 && b == 4 && world.bodyCount() == 8, "bodies of both models");
    world.setBodyStatic(a, true);
    world.setBodyStatic(b, true);
    var tools = [world.add(a + 3, CollisionPose.identity(), CollisionGeometry.Sphere(0.11)),
      world.add(b + 3, CollisionPose.identity(), CollisionGeometry.Sphere(0.11))];
    var fores = [world.add(a + 2, CollisionPose.translation(0.5, 0, 0), CollisionGeometry.Sphere(0.1)),
      world.add(b + 2, CollisionPose.translation(0.5, 0, 0), CollisionGeometry.Sphere(0.1))];
    var bases = [world.add(a, CollisionPose.identity(), CollisionGeometry.Box(0.1, 0.1, 0.1)),
      world.add(b, CollisionPose.identity(), CollisionGeometry.Box(0.1, 0.1, 0.1))];
    check(world.pairStatus(tools[0], fores[0]) == CollisionPairStatus.Rigid, "left keeps its rules");
    check(world.pairStatus(tools[1], fores[1]) == CollisionPairStatus.Rigid, "right keeps its rules");
    check(world.pairStatus(tools[0], fores[1]) == CollisionPairStatus.Checked, "left tool against right forearm");
    check(world.pairStatus(bases[0], bases[1]) == CollisionPairStatus.Static, "two fixed bases are static");
    var snapshotA = new KinematicSnapshot(left), snapshotB = new KinematicSnapshot(right);
    snapshotA.evaluate(new KinematicState(left, [0.0, 0.0]));
    snapshotB.evaluate(new KinematicState(right, [0.0, 0.0]));
    place(world, a, left, snapshotA);
    place(world, b, right, snapshotB);
    // Tools at x = 2 and x = 2.2: spheres of 0.11 overlap.
    var hits = world.colliding(0.0);
    check(hits.length == 1 && hits[0].a == tools[0] && hits[0].b == tools[1] && hits[0].bodyA == a + 3
      && hits[0].bodyB == b + 3, "the two tools touch");
    // Only the right arm moves: its elbow folds by 90 degrees, its tool to (3.2, -1, 0).
    snapshotB.evaluate(new KinematicState(right, [0.0, Math.PI / 2]));
    place(world, b, right, snapshotB);
    var gap = world.pairDistances([new CollisionPair(tools[0], tools[1], a + 3, b + 3)])[0].distance;
    check(Math.abs(gap - (Math.sqrt(1.2 * 1.2 + 1.0) - 0.22)) < 1e-9, 'moving one model moves only its bodies ($gap)');
    world.dispose();
  }

  /** Inflation carries padding exactly: distances to inflated primitives and convex sets shrink by the radius. */
  static function testInflationAgainstExactDistances():Void {
    var world = new NativeCollisionWorld();
    var body = world.addBody(0);
    var cube = [for (i in 0...8) for (axis in 0...3) ((i >> axis) & 1) == 1 ? 0.1 : -0.1];
    var sphere = world.add(body, CollisionPose.identity(), CollisionGeometry.Sphere(0.2));
    var box = world.add(-1, CollisionPose.translation(1, 0, 0), CollisionGeometry.Box(0.1, 0.3, 0.3));
    var hull = world.add(-1, CollisionPose.translation(0, 1, 0), CollisionGeometry.Convex(cube));
    var capsule = world.add(-1, CollisionPose.translation(0, 0, 1), CollisionGeometry.Capsule(0.05, 0.2));
    var worst = 0.0;
    for (radius in [0.0, 0.01, 0.05, 0.2]) {
      for (object in [sphere, box, hull, capsule]) world.setInflation(object, radius);
      var found = world.distances(10.0);
      function of(object:Int):Float {
        for (d in found) if (d.a == Std.int(Math.min(sphere, object)) && d.b == Std.int(Math.max(sphere, object)))
          return d.distance;
        throw 'pair with $object not found';
      }
      // Exact distances: centre gaps minus the radii and inflations of both.
      worst = Math.max(worst, Math.abs(of(box) - (1.0 - 0.1 - 0.2 - 2 * radius)));
      worst = Math.max(worst, Math.abs(of(hull) - (1.0 - 0.1 - 0.2 - 2 * radius)));
      worst = Math.max(worst, Math.abs(of(capsule) - (1.0 - 0.25 - 0.2 - 2 * radius)));
    }
    check(worst < 1e-9, 'inflated distances are exact (worst $worst)');
    var mesh = world.add(-1, CollisionPose.identity(), CollisionGeometry.Mesh([0, 0, -1, 1, 0, -1, 0, 1, -1], [0, 1, 2]));
    rejects(() -> world.setInflation(mesh, 0.1), "a mesh cannot be inflated");
    rejects(() -> world.setInflation(box, -0.1), "inflation is not negative");
    world.dispose();
  }

  /** Violations with group margins and per-body inflation, the closest pair, and batched pose sets from snapshots. */
  static function testViolationsAndBatches():Void {
    var model = twoLinkArm();
    var world = new NativeCollisionWorld();
    var first = register(world, model, 0);
    world.setBodyStatic(first, true);
    world.setBodyGroup(first + 3, 1);
    var toolBox = world.add(first + 3, CollisionPose.identity(), CollisionGeometry.Box(0.1, 0.1, 0.1));
    world.add(first + 2, CollisionPose.translation(0.5, 0, 0), CollisionGeometry.Sphere(0.05));
    var obstacle = world.add(-1, CollisionPose.translation(2.5, 0, 0), CollisionGeometry.Box(0.2, 0.2, 0.2));
    var snapshot = new KinematicSnapshot(model);
    snapshot.evaluate(new KinematicState(model, [0.0, 0.0]));
    place(world, first, model, snapshot);
    var margins = new CollisionMargins(2, 0.1);
    check(world.violation(margins) == null, "the tool clears 0.1");
    var inflation = [0.0, 0.0, 0.0, 0.11];
    var found = world.violation(margins, inflation);
    if (found == null) throw "assertion failed: the tool's inflation brings it within the margin";
    check(found.a == toolBox && found.b == obstacle && found.bodyA == first + 3 && found.bodyB == -1,
      "the tool's inflation brings it within the margin");
    check(Math.abs(found.distance - 0.2) < 1e-9 && Math.abs(found.required - 0.21) < 1e-12,
      'distance and required clearance (${found.distance}, ${found.required})');
    margins.set(1, 0, 0.3);
    var wider = world.violation(margins);
    check(wider != null && wider.required == 0.3, "the tool group's margin to the world group");
    rejects(() -> world.closest(CollisionMargins.uniform(0.0)), "one group's table does not cover the tool's group");
    margins.set(1, 0, 0.1);
    var closest = world.closest(margins);
    if (closest == null) throw "assertion failed: a closest pair";
    check(closest.a == toolBox && Math.abs(closest.distance - 0.2) < 1e-9 && closest.required == 0.1, "the closest pair");

    // A sweep of the elbow, as pose sets; the tool reaches the obstacle once inflated at the last set.
    var poses:Array<Float> = [];
    var sets = 5;
    for (i in 0...sets) {
      snapshot.evaluate(new KinematicState(model, [0.0, Math.PI / 2 * (1 - i / (sets - 1))]));
      for (body in 0...model.bodyCount()) pose(snapshot.bodyPose(body)).writeTo(poses);
    }
    check(world.firstViolation(poses, margins) == null, "the sweep is clear");
    var inflations = [for (i in 0...sets) for (body in 0...4) i == sets - 1 && body == 3 ? 0.11 : 0.0];
    var batch = world.firstViolation(poses, margins, inflations);
    if (batch == null) throw "assertion failed: the batch fails";
    check(batch.set == sets - 1 && batch.a == toolBox, "the batch names the failing set");
    rejects(() -> world.firstViolation(poses.slice(0, 20), margins), "a pose set covers every body");
    rejects(() -> world.violation(CollisionMargins.uniform(0.1)), "the margins cover every group");
    world.dispose();
  }

  static function testRefusedHeights():Void {
    var world = new NativeCollisionWorld();
    rejects(() -> world.add(-1, CollisionPose.identity(), CollisionGeometry.HeightField(1, 1, [0, 0, 0, -2], 2, -1)),
      "heights below the minimum are refused");
    var field = world.add(-1, CollisionPose.identity(), CollisionGeometry.HeightField(1, 1, [0, 0, 0, 0], 2, -1));
    world.setHeights(field, [0, -1, 0, 0]);
    rejects(() -> world.setHeights(field, [0, -1.5, 0, 0]), "a dig below the minimum is refused, not clamped");
    world.dispose();
  }


  /** A two-link arm with a flange frame at the forearm's tip. */
  static function flangeArm(?root:Transform):KinematicModel {
    var builder = new KinematicModelBuilder();
    var base = builder.addBody("base"), upper = builder.addBody("upper"), fore = builder.addBody("fore");
    if (root != null) builder.setRootPose(base, root);
    var z = new Vector3(0, 0, 1);
    builder.addJoint("shoulder", JointKind.Revolute, base, upper, Transform.identity(), Transform.identity(), z, -3, 3);
    builder.addJoint("elbow", JointKind.Revolute, upper, fore, Transform.translation(1, 0, 0), Transform.identity(), z, -3, 3);
    builder.addFrame("flange", fore, Transform.translation(1, 0, 0));
    return builder.build();
  }

  /**
   * Servoes `model` toward `target` (flange position) for `steps` ticks of
   * 10 ms with avoidance rows from `world` (ds 10 mm, di 100 mm, xi 0.5 m/s):
   * returns the smallest distance seen on the moving arm's pairs, and leaves
   * `state` where it ends.
   */
  static function servo(model:KinematicModel, bodies:ModelBodies, build:CollisionBuild, state:KinematicState,
      target:Vector3, steps:Int, avoid:Bool):Float {
    var task = FrameTask.atFrame(model, model.frameIndex("flange"), Transform.translation(target.x, target.y, target.z),
      1e-6, 1e-6, null, 7, FrameOrientation.Free);
    var problem = new KinematicProblem(model).add(task);
    var qp = new NativeQpStep(problem.layout().width);
    var limits = StepLimits.ofVelocity([1.0, 1.0]);
    var closest = Math.POSITIVE_INFINITY;
    for (_ in 0...steps) {
      bodies.place(build, state);
      var near = build.world.distances(0.1);
      for (d in near) closest = Math.min(closest, d.distance);
      var pairs = avoid ? bodies.avoidancePairs(build, near, 0.01, 0.1) : null;
      var step = DifferentialIk.step(problem, state, 0.01, qp, limits, 0.2, 1e-3, null, 1000, pairs, 0.5);
      for (column in 0...problem.layout().width) state.q[problem.layout().dofs[column]] += step.velocity[column] * 0.01;
    }
    bodies.place(build, state);
    for (d in build.world.distances(0.1)) closest = Math.min(closest, d.distance);
    qp.dispose();
    return closest;
  }

  static function flangePoint(model:KinematicModel, state:KinematicState):Transform {
    var snapshot = new KinematicSnapshot(model);
    snapshot.evaluate(state);
    return snapshot.framePose(model.frameIndex("flange"));
  }

  /** CL5: reaching for a point behind a wall, the arm stops short of it at the margin and slides along it. */
  static function testServoStopsShortAndSlides():Void {
    var model = flangeArm();
    for (avoid in [false, true]) {
      var description = new collisionkit.CollisionDescription();
      var reference = new KinematicState(model, [1.2, -1.6]);
      var bodies = ModelBodies.describe(description, model, "arm", 0, reference, "start", true);
      description.addObject("tip", bodies.bodies[2], CollisionPose.translation(1, 0, 0), CollisionGeometry.Sphere(0.05));
      bodies.declare();
      // A wall whose face is the plane x = 1.5.
      description.addFixed("wall", CollisionPose.translation(1.6, 0, 0), CollisionGeometry.Box(0.1, 2, 2), 1);
      var world = new NativeCollisionWorld();
      var build = description.build(world);
      check(build.layoutErrors.length == 0, "the arm starts clear of the wall");
      var state = reference.copy();
      var closest = servo(model, bodies, build, state, new Vector3(1.7, 0.5, 0), 400, avoid);
      var tip = flangePoint(model, state);
      if (!avoid) check(closest < 0, 'without avoidance the tip goes into the wall ($closest)');
      else {
        check(closest >= 0.01 - 1e-3, 'with avoidance it stops short at the 10 mm margin ($closest)');
        check(Math.abs(tip.x - (1.5 - 0.05 - 0.01)) < 5e-3, 'at the wall (${tip.x})');
        check(Math.abs(tip.y - 0.5) < 0.02, 'having slid along it to the target height (${tip.y})');
      }
      world.dispose();
    }
  }

  /** CL5: the other side of a pair is another robot (no Jacobian in this solve); the arm still stops short of it. */
  static function testAvoidsAnotherRobot():Void {
    var model = flangeArm(), other = flangeArm(Transform.translation(2.0, 1.2, 0).compose(Transform.axisAngle(0, 0, 1, Math.PI)));
    var description = new collisionkit.CollisionDescription();
    var bodies = ModelBodies.describe(description, model, "arm", 0, new KinematicState(model, [0.0, 0.0]), "start", true);
    description.addObject("tip", bodies.bodies[2], CollisionPose.translation(1, 0, 0), CollisionGeometry.Sphere(0.05));
    bodies.declare();
    var otherBodies = ModelBodies.describe(description, other, "other", 2, new KinematicState(other, [0.0, 0.0]), "start", true);
    for (body in 1...3) description.addObject('other$body', otherBodies.bodies[body], CollisionPose.translation(0.5, 0, 0),
      CollisionGeometry.Capsule(0.05, 0.5));
    otherBodies.declare();
    var world = new NativeCollisionWorld();
    var build = description.build(world);
    check(build.layoutErrors.length == 0, "the two arms start clear");
    var state = new KinematicState(model, [0.0, 0.0]);
    // Reach for a point on the other robot's upper arm.
    var closest = servo(model, bodies, build, state, new Vector3(1.5, 1.2, 0), 400, true);
    check(closest >= 0.01 - 1e-3, 'the arm stops short of the other robot ($closest)');
    var pairs = bodies.avoidancePairs(build, world.distances(0.1), 0.01, 0.1);
    check(pairs.length > 0 && pairs.filter(p -> p.bodyB == -1 || p.bodyA == -1).length == pairs.length,
      "the other robot's side has no Jacobian in this solve");
    world.dispose();
  }

  /** CL5: starting inside the margin, the rows demand separation and the arm backs out. */
  static function testRecoversFromInsideTheMargin():Void {
    var model = flangeArm();
    var description = new collisionkit.CollisionDescription();
    // Bent, so the tip can move along the wall's normal (stretched out it could not).
    var reference = new KinematicState(model, [0.5, -1.0]);
    var bodies = ModelBodies.describe(description, model, "arm", 0, reference, "start", true);
    description.addObject("tip", bodies.bodies[2], CollisionPose.translation(1, 0, 0), CollisionGeometry.Sphere(0.05));
    bodies.declare();
    // The wall's face is 4 mm beyond the tip sphere.
    var tipX = flangePoint(model, reference).x;
    description.addFixed("wall", CollisionPose.translation(tipX + 0.05 + 0.004 + 0.1, 0, 0), CollisionGeometry.Box(0.1, 2, 2), 1);
    var world = new NativeCollisionWorld();
    var build = description.build(world);
    var state = reference.copy();
    bodies.place(build, state);
    var start = world.distances(0.1)[0].distance;
    check(Math.abs(start - 0.004) < 1e-9, 'it starts 4 mm from the wall ($start)');
    // Holding its place (the target is where it is) it still backs out to the margin.
    var task = FrameTask.atFrame(model, model.frameIndex("flange"), flangePoint(model, state), 1e-6, 1e-6, null, 7, FrameOrientation.Free);
    var problem = new KinematicProblem(model).add(task);
    var qp = new NativeQpStep(2);
    var distances = [start];
    for (_ in 0...300) {
      bodies.place(build, state);
      var pairs = bodies.avoidancePairs(build, world.distances(0.1), 0.01, 0.1);
      var step = DifferentialIk.step(problem, state, 0.01, qp, StepLimits.ofVelocity([1.0, 1.0]), 0.2, 1e-3, null, 1000, pairs, 0.5);
      for (column in 0...2) state.q[problem.layout().dofs[column]] += step.velocity[column] * 0.01;
      bodies.place(build, state);
      distances.push(world.distances(0.1)[0].distance);
    }
    qp.dispose();
    var rising = true;
    for (i in 1...distances.length) if (distances[i] < distances[i - 1] - 1e-9 && distances[i - 1] < 0.01 - 1e-4) rising = false;
    var last = distances[distances.length - 1];
    check(rising && last >= 0.01 - 1e-3, 'the distance rises back to the margin (${distances[1]}, ..., $last)');
    world.dispose();
  }

  /** CL5: rows the step limits cannot honour are relaxed, and the step says which. */
  static function testRelaxesRowsThatCannotHold():Void {
    var model = flangeArm();
    var state = new KinematicState(model, [0.5, -1.0]);
    var task = FrameTask.atFrame(model, model.frameIndex("flange"), flangePoint(model, state), 1e-6, 1e-6, null, 7, FrameOrientation.Free);
    var problem = new KinematicProblem(model).add(task);
    var qp = new NativeQpStep(2);
    // Inside the margin, but no joint may move at all.
    var pair = new AvoidancePair(2, -1, new Vector3(2.05, 0, 0), new Vector3(2.054, 0, 0), new Vector3(1, 0, 0), 0.004, 0.01, 0.1);
    var step = DifferentialIk.step(problem, state, 0.01, qp, StepLimits.ofVelocity([0.0, 0.0]), 0.2, 1e-3, null, 1000, [pair], 0.5);
    check(step.avoided.length == 1 && step.relaxed.length == 1 && step.relaxed[0] == pair, "the row that cannot hold is relaxed and named");
    check(Math.abs(step.velocity[0]) < 1e-9 && Math.abs(step.velocity[1]) < 1e-9, "and the limits still hold exactly");
    // With room to move, it holds as given.
    var free = DifferentialIk.step(problem, state, 0.01, qp, StepLimits.ofVelocity([1.0, 1.0]), 0.2, 1e-3, null, 1000, [pair], 0.5);
    check(free.relaxed.length == 0 && !free.fallback, "a row that can hold is not relaxed");
    qp.dispose();
  }

  /** Registers a model's bodies (in its order) in `group`, with the kit's pairs as rules; returns the first body. */
  static function register(world:CollisionWorld, model:KinematicModel, group:Int):Int {
    var first = world.bodyCount();
    for (_ in 0...model.bodyCount()) world.addBody(group);
    for (pair in KinematicBodyPairs.of(model)) {
      var reason = switch pair.relation {
        case BodyPairRelation.Rigid: CollisionPairStatus.Rigid;
        case BodyPairRelation.Adjacent: CollisionPairStatus.Adjacent;
        case _: CollisionPairStatus.Closure;
      }
      world.setBodyRule(first + pair.a, first + pair.b, CollisionPairRule.Allow, reason);
    }
    return first;
  }

  /** Poses a model's bodies from its snapshot. */
  static function place(world:CollisionWorld, first:Int, model:KinematicModel, snapshot:KinematicSnapshot):Void {
    var poses:Array<Float> = [];
    for (body in 0...model.bodyCount()) pose(snapshot.bodyPose(body)).writeTo(poses);
    world.setBodyPoses(first, poses);
  }

  static function pose(t:Transform):CollisionPose return new CollisionPose(t.x, t.y, t.z, t.qx, t.qy, t.qz, t.qw);

  /** Base -> revolute z -> upper -> revolute z at x = 1 -> fore -> fixed at x = 1 -> tool. */
  static function twoLinkArm(?root:Transform):KinematicModel {
    var builder = new KinematicModelBuilder();
    var base = builder.addBody("base");
    var upper = builder.addBody("upper");
    var fore = builder.addBody("fore");
    var tool = builder.addBody("tool");
    if (root != null) builder.setRootPose(base, root);
    var z = new Vector3(0, 0, 1);
    builder.addJoint("shoulder", JointKind.Revolute, base, upper, Transform.identity(), Transform.identity(), z);
    builder.addJoint("elbow", JointKind.Revolute, upper, fore, Transform.translation(1, 0, 0), Transform.identity(), z);
    builder.addJoint("flange", JointKind.Fixed, fore, tool, Transform.translation(1, 0, 0), Transform.identity());
    return builder.build();
  }

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
