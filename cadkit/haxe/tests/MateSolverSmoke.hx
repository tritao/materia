import cadkit.modeling.AssemblyMateDrag;
import cadkit.modeling.AssemblyMateJoints;
import cadkit.modeling.AssemblyMateJoints.AssemblyMateJointConversion;
import cadkit.modeling.AssemblyMateSolver;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyMate;
import materia.assembly.AssemblyDefinition.AssemblyMateKind;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Parts placed by mates: free parts move as rigid bodies, the diagnosis says what stays free or conflicts. */
class MateSolverSmoke {
	public static function run():Void {
		// A motor's flange on a bracket face with its shaft in the bore: one turn about the shaft stays free.
		var seated = motorOnBracket([mate("seat", AssemblyMateKind.Planar), mate("shaft", AssemblyMateKind.Coaxial, "bore", "axis")]);
		var result = AssemblyMateSolver.solve(seated);
		check(result.converged && result.freeRoots.join(",") == "motor", 'the motor is placed (${result.status}: ${result.message})');
		check(result.report.degreesOfFreedom == 1, 'planar + coaxial leave one turn free: ${result.report.degreesOfFreedom}');
		check(result.implied.length == 0, 'planar + coaxial overlap on the axis but neither is implied: ${result.implied}');
		check(result.movable.join(",") == "motor", 'the motor can still turn, the grounded bracket cannot: ${result.movable}');
		var placed = new AssemblyState(AssemblyMateSolver.place(seated, result));
		var flange = placed.worldConnector("motor", "flange"), axis = placed.worldConnector("motor", "axis");
		near(flange.z, 100, "the flange sits on the face");
		near(axis.x, 40, "the shaft is on the bore's axis (x)");
		near(axis.y, -20, "the shaft is on the bore's axis (y)");

		checkDocuments(seated);
		checkDrag(AssemblyMateSolver.place(seated, result));
		checkJoints(AssemblyMateSolver.place(seated, result));

		// Locked: fully placed, exactly on the target frame.
		var locked = motorOnBracket([mate("weld", AssemblyMateKind.Lock)]);
		var lockResult = AssemblyMateSolver.solve(locked);
		check(lockResult.converged && lockResult.report.degreesOfFreedom == 0, 'a lock places the motor fully (${lockResult.status})');
		check(lockResult.movable.length == 0, "a locked motor cannot move");
		var lockedState = new AssemblyState(locked, lockResult.state(locked));
		var face = lockedState.worldConnector("bracket", "face"), welded = lockedState.worldConnector("motor", "flange");
		near(welded.x, face.x, "the locked flange is on the face (x)");
		near(Math.abs(welded.qw * face.qw + welded.qx * face.qx + welded.qy * face.qy + welded.qz * face.qz), 1, "and turned like it");

		// Two planar mates with different offsets cannot both hold.
		var conflicting = motorOnBracket([mate("seat", AssemblyMateKind.Planar), mate("lift", AssemblyMateKind.Planar, null, null, 5)]);
		var conflict = AssemblyMateSolver.solve(conflicting);
		check(!conflict.converged && conflict.status == "conflicting" && conflict.report.conflictingOwners().join(",") == "lift,seat",
			'contradicting mates conflict: ${conflict.status} ${conflict.report.conflictingOwners()}');

		// The same mate twice is redundant, and still places the part.
		var twice = motorOnBracket([mate("seat", AssemblyMateKind.Planar), mate("again", AssemblyMateKind.Planar)]);
		var redundant = AssemblyMateSolver.solve(twice);
		check(redundant.converged && redundant.report.redundantOwners().join(",") == "again,seat",
			'a repeated mate is redundant: ${redundant.report.redundantOwners()}');
		check(redundant.implied.join(",") == "seat,again" || redundant.implied.join(",") == "again,seat",
			'and each copy is implied by the other: ${redundant.implied}');

		// A mate between a jointed arm's tip and a fixture turns the arm's joint, not the grounded base.
		var arm = armToFixture();
		var reach = AssemblyMateSolver.solve(arm);
		check(reach.converged && reach.freeRoots.length == 0 && reach.jointCoordinates.length == 1,
			'the arm reaches the fixture by its joint (${reach.status}, roots ${reach.freeRoots})');
		var armState = new AssemblyState(arm, reach.state(arm));
		var tip = armState.worldConnector("link", "tip"), target = armState.worldConnector("base", "fixture");
		near(Math.sqrt(Math.pow(tip.x - target.x, 2) + Math.pow(tip.y - target.y, 2)), 0, "the tip is on the fixture point");
		check(!reach.degenerate, "an ordinary placement is not degenerate");

		// A folded arm whose tip is already at the mated distance from a point on its line: there, both joints move the
		// tip across the distance, so its row vanishes; anywhere else on the solutions it does not. The witness tells.
		var folded = foldedArm();
		var singular = AssemblyMateSolver.solve(folded);
		check(singular.converged && singular.degenerate && singular.report.degreesOfFreedom == 1,
			'a folded arm at its mated distance is a degenerate placement with one freedom (${singular.degenerate}, ${singular.report.degreesOfFreedom})');
	}

	/** Two links (100 and 40 mm) folded back along x, so the tip is at x = 60, 140 mm from a point at x = 200. */
	static function foldedArm():AssemblyDefinition {
		var free:AssemblyJointLimits = {lower: null, upper: null, velocity: null, effort: null};
		return {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "folded-arm", lengthUnit: "mm",
			definitions: [{id: "base", connectors: [{name: "pivot", frame: frame(0, 0, 0)}, {name: "point", frame: frame(200, 0, 0)}]},
				{id: "upper", connectors: [{name: "root", frame: frame(0, 0, 0)}, {name: "elbow", frame: frame(100, 0, 0)}]},
				{id: "fore", connectors: [{name: "root", frame: frame(0, 0, 0)}, {name: "tip", frame: frame(40, 0, 0)}]}],
			occurrences: [{id: "base", definition: "base", initialPose: frame(0, 0, 0), grounded: true},
				{id: "upper", definition: "upper", initialPose: frame(0, 0, 0)}, {id: "fore", definition: "fore", initialPose: frame(100, 0, 0)}],
			joints: [{id: "shoulder", type: AssemblyJointType.Revolute, role: AssemblyJointRole.Tree, parent: "base", parentConnector: "pivot",
				child: "upper", childConnector: "root", axis: {x: 0, y: 0, z: 1}, limits: free, defaultValue: 0},
				{id: "elbow", type: AssemblyJointType.Revolute, role: AssemblyJointRole.Tree, parent: "upper", parentConnector: "elbow",
					child: "fore", childConnector: "root", axis: {x: 0, y: 0, z: 1}, limits: free, defaultValue: Math.PI}],
			mates: [{id: "reach", kind: AssemblyMateKind.Distance, first: "base", firstConnector: "point", second: "fore",
				secondConnector: "tip", axis: {x: 0, y: 0, z: 1}, value: 140}]};
	}

	/**
		Dragging a mated part (plan C4.5d): a point on the seated motor's flange pulled sideways turns the motor about
		its shaft and it stays seated; pulled straight up, the mates hold it; a grounded part is not dragged.
	*/
	static function checkDrag(placed:AssemblyDefinition):Void {
		var start = new AssemblyState(placed).record();
		var drag = new AssemblyMateDrag(placed, start, "motor", new kinematicskit.Vector3(50, 0, 0));
		var grabbed = drag.grabbedPoint();
		var dx = grabbed.x - 40, dy = grabbed.y + 20;
		near(Math.sqrt(dx * dx + dy * dy), 50, "the grabbed point is 50 mm from the shaft");
		var turn = 0.5, cosine = Math.cos(turn), sine = Math.sin(turn);
		var target = new kinematicskit.Vector3(40 + dx * cosine - dy * sine, -20 + dx * sine + dy * cosine, grabbed.z);
		var result = drag.drag(target);
		check(result.following, 'pulled around the shaft, the motor follows: ${result.message}');
		var flange = drag.previewPose("motor");
		near(flange.x, 40, "it stays on the shaft (x)");
		near(flange.y, -20, "it stays on the shaft (y)");
		near(flange.z, 100, "it stays on the face");
		var up = drag.drag(new kinematicskit.Vector3(target.x, target.y, target.z + 50));
		check(!up.following && StringTools.startsWith(up.message, "Held by its mates"), 'pulled off the face, it is held: ${up.message}');
		near(drag.previewPose("motor").z, 100, "and stays on the face");
		var committed = new AssemblyState(placed, drag.commit()).worldPose("motor");
		near(committed.x, flange.x, "the committed state is the preview (x)");
		near(Math.abs(committed.qz * flange.qz + committed.qw * flange.qw + committed.qx * flange.qx + committed.qy * flange.qy), 1,
			"the committed state is the preview (turn)");
		var refused = false;
		try new AssemblyMateDrag(placed, start, "bracket", new kinematicskit.Vector3(0, 0, 0)) catch (_:Dynamic) refused = true;
		check(refused, "the grounded bracket is not dragged");
	}

	/**
		Mates to joints (plan C4.5e): planar + coaxial leave the motor a turn about its shaft, which becomes a
		revolute joint that keeps the placement at 0 and turns the motor about the shaft; coaxial + parallel x axes
		leave a slide (prismatic); coaxial alone leaves two motions, which make no joint.
	*/
	static function checkJoints(placed:AssemblyDefinition):Void {
		var start = new AssemblyState(placed).record();
		var revolute = AssemblyMateJoints.infer(placed, start, "motor");
		check(revolute.type == AssemblyJointType.Revolute && revolute.parent == "bracket", 'planar + coaxial is a revolute joint: ${revolute.reason}');
		near(revolute.point[0], 40, "on the shaft (x)");
		near(revolute.point[1], -20, "on the shaft (y)");
		near(Math.abs(revolute.axis[2]), 1, "about z");
		var conversion = AssemblyMateJoints.convert(placed, start, revolute, "hinge");
		var jointed = withJoint(placed, conversion);
		var state = new AssemblyState(jointed);
		var before = new AssemblyState(placed).worldPose("motor"), at = state.worldPose("motor");
		near(at.x, before.x, "the joint keeps the placement (x)");
		near(at.y, before.y, "the joint keeps the placement (y)");
		near(at.z, before.z, "the joint keeps the placement (z)");
		near(Math.abs(at.qx * before.qx + at.qy * before.qy + at.qz * before.qz + at.qw * before.qw), 1, "and the turn");
		state.setJoint("hinge", 0.7);
		var turned = state.worldConnector("motor", "flange");
		near(turned.x, 40, "turning the joint keeps the flange on the shaft (x)");
		near(turned.z, 100, "and on the face");

		var sliding = motorOnBracket([mate("shaft", AssemblyMateKind.Coaxial, "bore", "axis"),
			{id: "square", kind: AssemblyMateKind.Parallel, first: "bracket", firstConnector: "face", second: "motor", secondConnector: "flange",
				axis: {x: 1, y: 0, z: 0}}]);
		var slid = AssemblyMateSolver.place(sliding, AssemblyMateSolver.solve(sliding));
		var prismatic = AssemblyMateJoints.infer(slid, new AssemblyState(slid).record(), "motor");
		check(prismatic.type == AssemblyJointType.Prismatic, 'coaxial + parallel x axes is a prismatic joint: ${prismatic.reason}');
		near(Math.abs(prismatic.axis[2]), 1, "along z");

		var loose = motorOnBracket([mate("shaft", AssemblyMateKind.Coaxial, "bore", "axis")]);
		var looseState = AssemblyMateSolver.place(loose, AssemblyMateSolver.solve(loose));
		var none = AssemblyMateJoints.infer(looseState, new AssemblyState(looseState).record(), "motor");
		check(none.type == null && none.reason.indexOf("2 motions") >= 0, 'coaxial alone makes no joint: ${none.reason}');
	}

	/** `definition` with `conversion`'s joint and connectors, without the mates it replaces. */
	static function withJoint(definition:AssemblyDefinition, conversion:AssemblyMateJointConversion):AssemblyDefinition {
		var copy = AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(definition));
		for (component in copy.definitions) {
			if (component.id == conversion.parentComponent) component.connectors.push(conversion.parentConnector);
			if (component.id == conversion.childComponent) component.connectors.push(conversion.childConnector);
		}
		copy.joints.push(conversion.joint);
		var mates = copy.mates == null ? [] : copy.mates;
		copy.mates = [for (mate in mates) if (conversion.mates.indexOf(mate.id) < 0) mate];
		AssemblyDefinitionCodec.validate(copy);
		return copy;
	}

	/** Mates and grounded occurrences survive assembly documents and a document save and reload. */
	static function checkDocuments(definition:AssemblyDefinition):Void {
		var document = new cadkit.parametric.Document();
		var root = cadkit.parametric.AssemblyDocuments.fromDefinition(document, definition);
		var reloaded = cadkit.parametric.DocumentCodec.decode(cadkit.parametric.DocumentCodec.encode(document), false, false);
		var back = cadkit.parametric.AssemblyDocuments.toDefinition(reloaded.element(root.id));
		var mates = back.mates == null ? [] : back.mates;
		var ids = [for (m in mates) m.id + ":" + m.kind + ":" + m.firstConnector + ">" + m.secondConnector];
		check(ids.join(",") == "seat:planar:face>flange,shaft:coaxial:bore>axis", 'mates survive documents: $ids');
		var bracket = [for (o in back.occurrences) if (o.id == "bracket") o][0];
		check(bracket.grounded == true, "grounded survives documents");
		check(AssemblyMateSolver.solve(back).report.degreesOfFreedom == 1, "the reloaded assembly solves the same");
		reloaded.close();
		document.close();
	}

	static function mate(id:String, kind:AssemblyMateKind, ?first:String, ?second:String, ?value:Float):AssemblyMate {
		var result:AssemblyMate = {id: id, kind: kind, first: "bracket", firstConnector: first == null ? "face" : first,
			second: "motor", secondConnector: second == null ? "flange" : second, axis: {x: 0, y: 0, z: 1}};
		if (value != null) result.value = value;
		return result;
	}

	/** A grounded bracket with a face at z = 100 and a bore along z through (40, -20); a free motor placed far off. */
	static function motorOnBracket(mates:Array<AssemblyMate>):AssemblyDefinition {
		return {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "motor-on-bracket", lengthUnit: "mm",
			definitions: [{id: "bracket", connectors: [{name: "face", frame: frame(40, -20, 100)}, {name: "bore", frame: frame(40, -20, 0)}]},
				{id: "motor", connectors: [{name: "flange", frame: frame(0, 0, 0)}, {name: "axis", frame: frame(0, 0, -30)}]}],
			occurrences: [{id: "bracket", definition: "bracket", initialPose: frame(0, 0, 0), grounded: true},
				{id: "motor", definition: "motor", initialPose: rotated(300, 150, -80, 0.4)}],
			joints: [], mates: mates};
	}

	/** A grounded base with a fixture point, and a 1000 mm link on a revolute joint at the origin. */
	static function armToFixture():AssemblyDefinition {
		var fixtureAngle = 0.9;
		return {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "arm-to-fixture", lengthUnit: "mm",
			definitions: [{id: "base", connectors: [{name: "pivot", frame: frame(0, 0, 0)},
				{name: "fixture", frame: frame(1000 * Math.cos(fixtureAngle), 1000 * Math.sin(fixtureAngle), 0)}]},
				{id: "link", connectors: [{name: "root", frame: frame(0, 0, 0)}, {name: "tip", frame: frame(1000, 0, 0)}]}],
			occurrences: [{id: "base", definition: "base", initialPose: frame(0, 0, 0), grounded: true},
				{id: "link", definition: "link", initialPose: frame(0, 0, 0)}],
			joints: [{id: "shoulder", type: AssemblyJointType.Revolute, role: AssemblyJointRole.Tree, parent: "base", parentConnector: "pivot",
				child: "link", childConnector: "root", axis: {x: 0, y: 0, z: 1}, limits: {lower: null, upper: null, velocity: null, effort: null},
				defaultValue: 0.2}],
			mates: [{id: "touch", kind: AssemblyMateKind.Coincident, first: "base", firstConnector: "fixture", second: "link",
				secondConnector: "tip", axis: {x: 0, y: 0, z: 1}}]};
	}

	static function frame(x:Float, y:Float, z:Float):AssemblyFrame
		return {x: x, y: y, z: z, qx: 0, qy: 0, qz: 0, qw: 1};

	static function rotated(x:Float, y:Float, z:Float, angle:Float):AssemblyFrame
		return {x: x, y: y, z: z, qx: Math.sin(angle / 2) * 0.6, qy: 0, qz: Math.sin(angle / 2) * 0.8, qw: Math.cos(angle / 2)};

	static function near(value:Float, expected:Float, label:String):Void
		check(Math.abs(value - expected) < 1e-3, '$label: $value, expected $expected');

	static function check(value:Bool, label:String):Void {
		if (!value) throw label;
	}
}
