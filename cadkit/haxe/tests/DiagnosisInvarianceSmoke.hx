import cadkit.modeling.AssemblyState;
import cadkit.sketch.ConstrainedSketch;
import cadkit.sketch.SketchConstraint;
import cadkit.sketch.SketchEntity;
import cadkit.sketch.SketchPoint;
import cadkit.sketch.SketchSolveError;
import cadkit.sketch.SolveDiagnostic;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

private typedef SketchFixture = {
	var name:String;
	/** The correct classification, e.g. "fully-constrained dof=0"; with " ids=…" the constraints it names too. */
	var expected:String;
	var build:Void->ConstrainedSketch;
}

private typedef AssemblyFixture = {
	var name:String;
	var expected:String;
	var build:Void->AssemblyDefinition;
	var dependent:Array<String>;
}

/**
	C0 of `cadkit/plans/CONSTRAINT_SOLVING.md`. Each fixture's diagnosis must be
	its correct answer, and must not change under transformations that leave
	the mathematics alone. `KNOWN` lists where today's diagnosis falls short;
	the smoke fails when that list is wrong in either direction, so a fix must
	remove its entry and a regression cannot hide.
*/
class DiagnosisInvarianceSmoke {
	static final KNOWN:Array<String> = [
		// An unclosable loop ends at a stationary residual but is reported as out of iterations (C3).
		"four-bar-impossible/expected",
	];

	public static function run():Void {
		var failures:Array<String> = [];
		for (fixture in sketchFixtures())
			checkSketch(fixture, failures);
		for (fixture in assemblyFixtures())
			checkAssembly(fixture, failures);
		checkDegenerateFlag(failures);

		var unexpected = [for (failure in failures) if (KNOWN.indexOf(key(failure)) < 0) failure];
		var failedKeys = [for (failure in failures) key(failure)];
		var fixed = [for (known in KNOWN) if (failedKeys.indexOf(known) < 0) known];
		if (unexpected.length > 0 || fixed.length > 0)
			throw "Diagnosis invariance changed.\nNew failures:\n  " + unexpected.join("\n  ") + "\nNow passing (remove from KNOWN):\n  " + fixed.join("\n  ");
	}

	/** A failure reads "fixture/check: detail"; `KNOWN` holds the part before the colon. */
	static function key(failure:String):String
		return failure.substr(0, failure.indexOf(":"));

	// ---- Sketches -------------------------------------------------------------------------------------------

	static function sketchFixtures():Array<SketchFixture> {
		return [
			{name: "rectangle", expected: "fully-constrained dof=0", build: () -> rectangle(true, null)},
			{name: "rectangle-open", expected: "under-constrained dof=1", build: () -> rectangle(false, null)},
			{name: "rectangle-redundant", expected: "redundant dof=0 ids=top-length,v0,v1,width", build: () -> rectangle(true, 10)},
			{name: "rectangle-conflict", expected: "conflicting ids=top-length,v0,v1,width", build: () -> rectangle(true, 12)},
			{name: "two-rectangles-redundant", expected: "redundant dof=0 ids=a.top-length,a.v0,a.v1,a.width,b.top-length,b.v0,b.v1,b.width",
				build: twoRedundantRectangles},
			// Generically fully constrained; at this solution the two circles C lies on touch, so the local
			// Jacobian loses a rank the design does not.
			{name: "touching-circles", expected: "fully-constrained dof=0", build: () -> touchingCircles(false)},
			// The same, authored exactly on the singular pose: the solve starts there and never leaves.
			{name: "touching-circles-exact", expected: "fully-constrained dof=0", build: () -> touchingCircles(true)},
			// Parallelism is transitive only on the shape itself: a witness pose must keep this dependency.
			{name: "parallel-lines", expected: "redundant dof=10 ids=p12,p13,p23", build: parallelLines},
		];
	}

	static function checkSketch(fixture:SketchFixture, failures:Array<String>):Void {
		var base = sketchSummary(fixture.build());
		if ((fixture.expected.indexOf(" ids=") >= 0 ? base : classification(base)) != fixture.expected)
			failures.push('${fixture.name}/expected: want ${fixture.expected}, got $base');
		var transforms:Array<{name:String, apply:ConstrainedSketch->ConstrainedSketch}> = [
			{name: "scale-1e-6", apply: s -> transformSketch(s, 1e-6, 0, 0, 0, false, 0)},
			{name: "scale-1e6", apply: s -> transformSketch(s, 1e6, 0, 0, 0, false, 0)},
			{name: "translate", apply: s -> transformSketch(s, 1, 1234.5, -678.9, 0, false, 0)},
			{name: "rotate", apply: s -> transformSketch(s, 1, 0, 0, 0.7, false, 0)},
			{name: "reorder", apply: s -> transformSketch(s, 1, 0, 0, 0, true, 0)},
			{name: "perturb", apply: s -> transformSketch(s, 1, 0, 0, 0, false, 0.02)},
		];
		for (transform in transforms) {
			var summary = sketchSummary(transform.apply(fixture.build()));
			if (summary != base)
				failures.push('${fixture.name}/${transform.name}: $summary, untransformed $base');
		}
		var sketch = fixture.build();
		sketchSummary(sketch);
		var again = sketchSummary(sketch);
		if (again != base)
			failures.push('${fixture.name}/repeat: $again, first $base');
	}

	/** Degeneracy belongs to a pose, so it is checked directly rather than through the invariant summary. */
	static function checkDegenerateFlag(failures:Array<String>):Void {
		if (!touchingCircles(true).solve().diagnostic.degenerate)
			failures.push("touching-circles-exact/degenerate-flag: the singular pose is not reported degenerate");
		if (touchingCircles(false).solve().diagnostic.degenerate)
			failures.push("touching-circles/degenerate-flag: a regular pose is reported degenerate");
		if (rectangle(true, 10).solve().diagnostic.degenerate)
			failures.push("rectangle-redundant/degenerate-flag: a real redundancy is reported as degeneracy");
	}

	/** "status dof=N ids=a,b" with ids sorted; a failed solve reports its diagnostic too. */
	static function sketchSummary(sketch:ConstrainedSketch):String {
		var diagnostic:SolveDiagnostic;
		try {
			diagnostic = sketch.solve().diagnostic;
		} catch (error:SketchSolveError) {
			diagnostic = error.diagnostic;
		}
		var ids = diagnostic.constraintIds.copy();
		ids.sort(Reflect.compare);
		if (!diagnostic.converged)
			return diagnostic.status + " ids=" + ids.join(",");
		return diagnostic.status + " dof=" + diagnostic.degreesOfFreedom + " ids=" + ids.join(",");
	}

	/** Status and degrees of freedom, without the constraint IDs. */
	static function classification(summary:String):String {
		var ids = summary.indexOf(" ids=");
		return ids < 0 ? summary : summary.substr(0, ids);
	}

	/** Four points, horizontal/vertical sides, origin fixed, width 10; height 5 when `closed`; optionally a top length. */
	static function rectangle(closed:Bool, topLength:Null<Float>, prefix:String = "", dx:Float = 0):ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		addRectangle(sketch, closed, topLength, prefix, dx);
		return sketch;
	}

	static function addRectangle(sketch:ConstrainedSketch, closed:Bool, topLength:Null<Float>, prefix:String, dx:Float):Void {
		var p = [for (i in 0...4) prefix + "p" + i];
		sketch.addPoint(new SketchPoint(p[0], dx + 0.2, -0.1)).addPoint(new SketchPoint(p[1], dx + 9.7, 0.3))
			.addPoint(new SketchPoint(p[2], dx + 10.4, 5.1)).addPoint(new SketchPoint(p[3], dx - 0.3, 4.8));
		sketch.addEntity(SketchEntity.line(prefix + "bottom", p[0], p[1])).addEntity(SketchEntity.line(prefix + "right", p[1], p[2]))
			.addEntity(SketchEntity.line(prefix + "top", p[2], p[3])).addEntity(SketchEntity.line(prefix + "left", p[3], p[0]));
		sketch.addConstraint(SketchConstraint.fixed(prefix + "origin", p[0]))
			.addConstraint(SketchConstraint.horizontal(prefix + "h0", prefix + "bottom"))
			.addConstraint(SketchConstraint.horizontal(prefix + "h1", prefix + "top"))
			.addConstraint(SketchConstraint.vertical(prefix + "v0", prefix + "right"))
			.addConstraint(SketchConstraint.vertical(prefix + "v1", prefix + "left"))
			.addConstraint(SketchConstraint.distance(prefix + "width", p[0], p[1], 10));
		if (closed)
			sketch.addConstraint(SketchConstraint.distance(prefix + "height", p[1], p[2], 5));
		if (topLength != null)
			sketch.addConstraint(SketchConstraint.distance(prefix + "top-length", p[2], p[3], topLength));
	}

	static function twoRedundantRectangles():ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		addRectangle(sketch, true, 10, "a.", 0);
		addRectangle(sketch, true, 10, "b.", 30);
		return sketch;
	}

	/** A fixed, B on a horizontal line 10 away, C 5 from both: C must be the midpoint. */
	static function touchingCircles(exact:Bool):ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		sketch.addPoint(new SketchPoint("a", 0, 0));
		if (exact)
			sketch.addPoint(new SketchPoint("b", 10, 0)).addPoint(new SketchPoint("c", 5, 0));
		else
			sketch.addPoint(new SketchPoint("b", 9.8, 0.2)).addPoint(new SketchPoint("c", 5.1, 0.6));
		sketch.addEntity(SketchEntity.line("ab", "a", "b"));
		sketch.addConstraint(SketchConstraint.fixed("origin", "a"))
			.addConstraint(SketchConstraint.horizontal("level", "ab"))
			.addConstraint(SketchConstraint.distance("ab-length", "a", "b", 10))
			.addConstraint(SketchConstraint.distance("ac-length", "a", "c", 5))
			.addConstraint(SketchConstraint.distance("bc-length", "b", "c", 5));
		return sketch;
	}

	/** Three free lines, each pair constrained parallel: one of the three constraints is implied by the others. */
	static function parallelLines():ConstrainedSketch {
		var sketch = new ConstrainedSketch();
		var ends = [[0.0, 0.0, 10.0, 1.0], [0.0, 3.0, 9.0, 4.2], [1.0, 7.0, 11.0, 7.8]];
		for (i in 0...3) {
			sketch.addPoint(new SketchPoint('s$i', ends[i][0], ends[i][1])).addPoint(new SketchPoint('e$i', ends[i][2], ends[i][3]));
			sketch.addEntity(SketchEntity.line('l$i', 's$i', 'e$i'));
		}
		sketch.addConstraint(SketchConstraint.parallel("p12", "l0", "l1"))
			.addConstraint(SketchConstraint.parallel("p23", "l1", "l2"))
			.addConstraint(SketchConstraint.parallel("p13", "l0", "l2"));
		return sketch;
	}

	/**
		Scales, rotates (about the origin) and translates the authored points,
		scales lengths with them, optionally reverses every list, and moves
		each point by a small deterministic offset of `perturbation` (in the
		sketch's own units, before scaling).
	*/
	static function transformSketch(source:ConstrainedSketch, scale:Float, dx:Float, dy:Float, angle:Float, reverse:Bool,
			perturbation:Float):ConstrainedSketch {
		var result = new ConstrainedSketch(source.plane, source.units, source.settings);
		var cos = Math.cos(angle), sin = Math.sin(angle);
		var points = source.points(), entities = source.entities(), constraints = source.constraints();
		if (reverse) {
			points.reverse();
			entities.reverse();
			constraints.reverse();
		}
		var index = 0;
		for (point in points) {
			index++;
			var x = point.x + perturbation * Math.sin(index * 1.7), y = point.y + perturbation * Math.cos(index * 2.3);
			x *= scale;
			y *= scale;
			result.addPoint(new SketchPoint(point.id, x * cos - y * sin + dx, x * sin + y * cos + dy));
		}
		for (entity in entities)
			result.addEntity(switch entity.kind {
				case "circle": SketchEntity.circle(entity.id, entity.first, entity.radius * scale, entity.construction);
				case "arc": SketchEntity.arc(entity.id, entity.first, entity.radius * scale, entity.startAngle + angle,
						entity.endAngle + angle, entity.clockwise, entity.construction);
				default: entity;
			});
		for (constraint in constraints)
			result.addConstraint(constraint.kind == "distance" || constraint.kind == "radius"
				? SketchConstraint.raw(constraint.id, constraint.kind, constraint.first, constraint.second, constraint.third,
					constraint.value * scale)
				: constraint);
		return result;
	}

	// ---- Assemblies -----------------------------------------------------------------------------------------

	static function assemblyFixtures():Array<AssemblyFixture> {
		return [
			{name: "four-bar", expected: "converged dof=0", dependent: ["coupler", "rocker"],
				build: () -> AssemblyLoopSmoke.fourBarDefinition(2200, null, null, 1.4)},
			// Coupler and rocker collinear at the only closed pose: generically rigid, locally singular.
			{name: "four-bar-toggle", expected: "converged dof=0", dependent: ["coupler", "rocker"], build: toggleFourBar},
			{name: "slider-crank", expected: "converged dof=0", dependent: ["rod", "slide"],
				build: AssemblyLoopSmoke.sliderCrankDefinition},
			{name: "four-bar-impossible", expected: "conflicting", dependent: ["coupler", "rocker"],
				build: () -> AssemblyLoopSmoke.fourBarDefinition(5000, null, null, 1.4)},
			// Planar linkages built from 3D revolute closures: their out-of-plane rows are redundant by design.
			{name: "excavator", expected: "converged dof=0", dependent: excavatorDependent(),
				build: ProceduralExcavatorAssembly.buildDefinition},
		];
	}

	static function checkAssembly(fixture:AssemblyFixture, failures:Array<String>):Void {
		var base = assemblySummary(fixture.build(), fixture.dependent, 0);
		if (classification(base) != fixture.expected)
			failures.push('${fixture.name}/expected: want ${fixture.expected}, got $base');
		var transforms:Array<{name:String, apply:AssemblyDefinition->AssemblyDefinition, seed:Float}> = [
			{name: "scale-1e-6", apply: d -> transformAssembly(d, 1e-6, AssemblyFrames.identity(), false), seed: 0},
			// Assembly positions are capped at 1e9 units by the schema, so the upward scale stops at 1e5.
			{name: "scale-1e5", apply: d -> transformAssembly(d, 1e5, AssemblyFrames.identity(), false), seed: 0},
			{name: "translate", apply: d -> transformAssembly(d, 1, AssemblyFrames.translation(1234.5, -678.9, 42), false), seed: 0},
			{name: "rotate", apply: d -> transformAssembly(d, 1, rotation(0.3, 0.8, 0.52, 0.7), false), seed: 0},
			{name: "reorder", apply: d -> transformAssembly(d, 1, AssemblyFrames.identity(), true), seed: 0},
			{name: "codec", apply: d -> AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(d)), seed: 0},
			{name: "perturb", apply: d -> d, seed: 0.01},
		];
		for (transform in transforms) {
			var summary = assemblySummary(transform.apply(fixture.build()), fixture.dependent, transform.seed);
			if (summary != base)
				failures.push('${fixture.name}/${transform.name}: $summary, untransformed $base');
		}
		var state = new AssemblyState(fixture.build());
		assemblyStateSummary(state, fixture.dependent, 0);
		var again = assemblyStateSummary(state, fixture.dependent, 0);
		if (again != base)
			failures.push('${fixture.name}/repeat: $again, first $base');
	}

	static function assemblySummary(definition:AssemblyDefinition, dependent:Array<String>, seed:Float):String
		return assemblyStateSummary(new AssemblyState(definition), dependent, seed);

	/** "status dof=N ids=a,b"; `seed` nudges every dependent coordinate before solving. */
	static function assemblyStateSummary(state:AssemblyState, dependent:Array<String>, seed:Float):String {
		if (seed != 0)
			for (joint in dependent)
				state.setJoint(joint, state.joint(joint) + seed);
		var result = state.solveClosures(dependent);
		var ids = result.closureIds.copy();
		ids.sort(Reflect.compare);
		if (!result.converged)
			return result.status + " ids=" + ids.join(",");
		return result.status + " dof=" + result.degreesOfFreedom + " ids=" + ids.join(",");
	}

	/** Ground pivots 2000 apart, crank 1000 at 0.6 rad; the coupler is exactly long enough to fold over the rocker. */
	static function toggleFourBar():AssemblyDefinition {
		var tipX = 1000 * Math.cos(0.6), tipY = 1000 * Math.sin(0.6);
		var reach = Math.sqrt((2000 - tipX) * (2000 - tipX) + tipY * tipY);
		var heading = Math.atan2(-tipY, 2000 - tipX);
		var definition = AssemblyLoopSmoke.fourBarDefinition(reach + 1500, null, null, heading + 0.05);
		for (joint in definition.joints)
			if (joint.id == "coupler")
				joint.defaultValue = heading - 0.6 - 0.05;
		return definition;
	}

	static function excavatorDependent():Array<String> {
		var candidates = ["link-one-hinge", "link-two-hinge", "boom-cylinder-hinge", "boom-cylinder-slide", "stick-cylinder-hinge",
			"stick-cylinder-slide", "bucket-cylinder-hinge", "bucket-cylinder-slide"];
		var tree = [for (joint in ProceduralExcavatorAssembly.buildDefinition().joints) if (joint.role == AssemblyJointRole.Tree) joint.id];
		return [for (id in candidates) if (tree.indexOf(id) >= 0) id];
	}

	/**
		Scales every length (connector and root positions, prismatic
		coordinates and limits, closure tolerances), places each root at
		`placement` composed with its own pose, and optionally reverses every
		list.
	*/
	static function transformAssembly(source:AssemblyDefinition, scale:Float, placement:AssemblyFrame, reverse:Bool):AssemblyDefinition {
		var copy = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(source));
		for (component in copy.definitions)
			for (connector in component.connectors)
				connector.frame = scaled(connector.frame, scale);
		var children = [for (joint in copy.joints) if (joint.role == AssemblyJointRole.Tree) joint.child];
		for (occurrence in copy.occurrences) {
			occurrence.initialPose = scaled(occurrence.initialPose, scale);
			if (children.indexOf(occurrence.id) < 0)
				occurrence.initialPose = AssemblyFrames.compose(placement, occurrence.initialPose);
		}
		for (joint in copy.joints) {
			if (joint.type == AssemblyJointType.Prismatic) {
				joint.defaultValue *= scale;
				if (joint.limits.lower != null) joint.limits.lower = joint.limits.lower * scale;
				if (joint.limits.upper != null) joint.limits.upper = joint.limits.upper * scale;
			}
			if (joint.closureTolerance != null) joint.closureTolerance = joint.closureTolerance * scale;
		}
		if (reverse) {
			copy.definitions.reverse();
			copy.occurrences.reverse();
			copy.joints.reverse();
		}
		return copy;
	}

	static function scaled(frame:AssemblyFrame, scale:Float):AssemblyFrame
		return {x: frame.x * scale, y: frame.y * scale, z: frame.z * scale, qx: frame.qx, qy: frame.qy, qz: frame.qz, qw: frame.qw};

	/** A rotation of `angle` about the axis (ax, ay, az), which is normalised here. */
	static function rotation(ax:Float, ay:Float, az:Float, angle:Float):AssemblyFrame {
		var length = Math.sqrt(ax * ax + ay * ay + az * az), s = Math.sin(angle / 2) / length;
		return {x: 0, y: 0, z: 0, qx: ax * s, qy: ay * s, qz: az * s, qw: Math.cos(angle / 2)};
	}
}
