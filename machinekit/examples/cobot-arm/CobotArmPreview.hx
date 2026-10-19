import machinekit.robotics.CobotArm;
import machinekit.robotics.CobotClass;
import machinekit.robotics.CobotWristLink;
import machinekit.robotics.CobotToolLink;
import machinekit.robotics.CobotJoint;
import machinekit.robotics.CobotArmLink;
import machinekit.robotics.CobotLinkCover;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.PosedParts;
import machinekit.assembly.PosedParts.PartBox;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;
import haxe.io.Bytes;

/** Each class runs a planned free-motion loop, exercising all six joint drives. */
class CobotArmPreview {
	public static function scene(cls:CobotClass):materia.project.SceneArtifact.SceneArtifactData {
		var arm = new CobotArm(cls);
		var ready = arm.ready();
		var scene = AssemblyPreview.scene(arm, "cobot-" + arm.reference.designation.toLowerCase());
		var placed = new AssemblyState(scene.assemblyDefinition);
		for (i in 0...6) placed.setJoint('j${i + 1}', ready[i]);
		scene.assemblyState = placed.record();
		var excursions = [[0.12, 0.08, -0.1, 0.08, 0.1, -0.12],
			[-0.12, -0.08, 0.1, -0.08, -0.1, 0.12], [0.0, 0, 0, 0, 0, 0]];
		scene.missions = [{loop: true, steps: [for (offset in excursions)
			{kind: "moveJoints", at: {occurrence: "toolFlange", connector: "face"},
				joints: [for (i in 0...6) {joint: 'j${i + 1}', position: ready[i] + offset[i]}]}]}];
		return scene;
	}
	public static function arm(cls:CobotClass):Bytes return SceneArtifact.encode(scene(cls));
	public static function reach500():Bytes return arm(Reach500);
	public static function reach850():Bytes return arm(Reach850);
	public static function reach900():Bytes return arm(Reach900);
	public static function reach1300():Bytes return arm(Reach1300);

	/** Reviewable scene plus posed mesh data, generated directly from the model. */
	public static function export():Void {
		var directory = "machinekit/examples/cobot-arm/build";
		for (cls in [Reach500, Reach850, Reach900, Reach1300]) {
			var selected = [for (arg in Sys.args()) if (StringTools.startsWith(arg, "--class=")) arg];
			if (selected.length > 0 && selected.indexOf("--class=" + Std.string(cls)) < 0) continue;
			var data:materia.project.SceneArtifact.SceneArtifactData = CobotArmPreview.scene(cls);
			var definition:materia.assembly.AssemblyDefinition = data.assemblyDefinition;
			var state = new AssemblyState(definition, data.assemblyState);
			var stem = directory + "/" + Std.string(cls).toLowerCase();
			sys.io.File.saveBytes(stem + ".scene", SceneArtifact.encode(data));
			var meshes:Array<{id:String, material:Null<String>, vertices:Array<Float>, indices:Array<Int>}> = [];
			for (occurrence in definition.occurrences) {
				var id = occurrence.id;
				if (id == "powerSupply" || StringTools.startsWith(id, "driver") || StringTools.startsWith(id, "gearbox")) continue;
				var pose = state.worldPose(id);
				for (part in data.parts) if (part.id == occurrence.definition) {
					var vertices:Array<Float> = [];
					for (i in 0...part.vertexCount) {
						var p = AssemblyFrames.transformPoint(pose, part.vertices.getDouble(i * 24),
							part.vertices.getDouble(i * 24 + 8), part.vertices.getDouble(i * 24 + 16));
						vertices.push(p.x); vertices.push(p.y); vertices.push(p.z);
					}
					meshes.push({id:id, material:part.materialId, vertices:vertices,
						indices:[for (i in 0...part.indexCount) part.indices.getInt32(i * 4)]});
				}
			}
			sys.io.File.saveContent(stem + ".mesh.json", haxe.Json.stringify(meshes));
			Sys.println('Exported $stem.scene and $stem.mesh.json'); Sys.stdout().flush();
		}
	}
}

/** Independent DH arithmetic checks the mechanically assembled flange. */
class CobotArmChecks {
	static function near(a:Float, b:Float, label:String, tolerance:Float = 0.01):Void
		if (!(Math.abs(a - b) <= tolerance)) throw '$label: expected $b, got $a';

	public static function run():Void {
		var selected = [for (arg in Sys.args()) if (StringTools.startsWith(arg, "--class=")) arg];
		var matched = false;
		for (cls in [Reach500, Reach850, Reach900, Reach1300])
			if (selected.length == 0 || selected.indexOf("--class=" + Std.string(cls)) >= 0) {
				matched = true;
				PosedParts.scope(parts -> checkClass(cls, parts));
			}
		if (!matched) throw "Unknown cobot class selection";
	}

	/** One class's DH, overlap, limit, torque and mass checks; its link geometry is built once in `parts` for all poses. */
	static function checkClass(cls:CobotClass, parts:PosedParts):Void {
		var arm = new CobotArm(cls);
		Sys.println('${arm.reference.designation}: checking housing construction'); Sys.stdout().flush();
		var model = new AssemblyModel("mm");
		arm.addTo(model, "");
		var definition = model.definition("cobot-arm");
		var state = new AssemblyState(definition);
		checkWristConstruction(arm, state, parts);
		checkArmConstruction(arm, state, parts);
		var rigidChecks:Map<String, Bool> = [];
		var poses = [[0.0, 0, 0, 0, 0, 0], arm.ready(), [0.2, -1, 0.8, -0.5, -0.7, 0.3],
			[-0.4, -0.8, 0.5, 0.6, -1.2, -0.2], [0.6, -1.4, 1.2, -0.4, -0.9, 0.1]];
		var oppositeWrist = arm.ready();
		oppositeWrist[4] = Math.PI / 2;
		poses.push(oppositeWrist);
		var turnedWrist = oppositeWrist.copy();
		turnedWrist[3] = Math.PI / 2;
		poses.push(turnedWrist);
		var readyPose = arm.ready();
		var excursion = [0.12, 0.08, -0.1, 0.08, 0.1, -0.12];
		for (fraction in [-1.0, -0.5, 0.5, 1.0])
			poses.push([for (i in 0...6) readyPose[i] + fraction * excursion[i]]);
		for (q in poses) {
			Sys.println('${arm.reference.designation}: checking pose $q'); Sys.stdout().flush();
			for (i in 0...6) state.setJoint('j${i + 1}', q[i]);
			var expected = dh(arm, q), actual = state.worldPose("toolFlange");
			near(actual.x, expected.x, "DH flange X"); near(actual.y, expected.y, "DH flange Y");
			near(actual.z, expected.z, "DH flange Z");
			var a = AssemblyFrames.toRotationMatrix(actual), b = AssemblyFrames.toRotationMatrix(expected);
			for (i in 0...9) near(a[i], b[i], "DH flange orientation", 1e-8);
			checkOverlap(arm, state, parts, rigidChecks);
		}
		for (i in 0...6) {
			var exported = [for (joint in definition.joints) if (joint.id == 'j${i + 1}') joint];
			if (exported.length != 1) throw "Cobot must export each joint exactly once";
			var lower = exported[0].limits.lower, upper = exported[0].limits.upper;
			if (lower == null || upper == null) throw "Cobot must export finite mechanical joint limits";
			near(lower, -2 * Math.PI, "Exported lower joint limit", 1e-10);
			near(upper, 2 * Math.PI, "Exported upper joint limit", 1e-10);
		}
		var ready = arm.ready();
		for (i in 0...6) state.setJoint('j${i + 1}', ready[i]);
		checkOverlap(arm, state, parts, rigidChecks);
		for (i in 0...6) state.setJoint('j${i + 1}', 0);
		var shoulder = state.worldPose("joint2"), flange = state.worldPose("toolFlange");
		var dx = flange.x - shoulder.x, dy = flange.y - shoulder.y, dz = flange.z - shoulder.z;
		var reach = Math.sqrt(dx * dx + dy * dy + dz * dz);
		var length = Math.abs(arm.reference.a[1]) + Math.abs(arm.reference.a[2]);
		var offset = arm.reference.d[3] + arm.reference.d[5];
		near(reach, Math.sqrt(length * length + offset * offset + arm.reference.d[4] * arm.reference.d[4]),
			"DH-zero shoulder reference distance");
		var torques = arm.holdingTorques(state, arm.reference.payload);
		for (i in 0...6) if (Math.abs(torques[i]) > arm.modules[i].outputRatedTorque)
			throw '${arm.reference.designation} joint ${i + 1} needs ${Math.abs(torques[i])} N m, exceeds ${arm.modules[i].outputRatedTorque} N m rated torque';
		var mass = arm.armMass();
		var checkedMass = switch cls {
			case Reach500: 14.471688223078141;
			case Reach850: 22.130439422601416;
			case Reach900: 36.343211560031065;
			case Reach1300: 36.79045128655561;
		};
		if (!Math.isFinite(mass) || Math.abs(mass - checkedMass) > 0.02)
			throw '${arm.reference.designation} assembled mass $mass kg differs from its checked assumed assembly mass $checkedMass kg';
		if (definition.encoders == null || definition.encoders.length != 6) throw "Cobot needs six output encoders";
		Sys.println('${arm.reference.designation}: $mass kg assembled (published reference ${arm.reference.mass} kg), DH-zero shoulder distance $reach mm, holding N m $torques, DH FK and CAD overlap at ${poses.length} poses, exported limits passed');
	}

	/** Contacts and actual CAD section properties complement the overlap test:
	 * empty space passes a collision check just as readily as a connected elbow.
	 * Section stiffness is compared with the old neck; this is a geometry
	 * regression, not a substitute for whole-elbow FEA or bearing selection.
	 */
	static function checkWristConstruction(arm:CobotArm, state:AssemblyState, parts:PosedParts):Void {
		for (i in [0, 3, 4]) {
			if (!Std.isOfType(arm.links[i], CobotWristLink)) throw "Cobot wrist needs structural elbows";
			var elbow:CobotWristLink = cast arm.links[i];
			var local = parts.local(elbow), minimum = Math.POSITIVE_INFINITY;
			var gap = elbow.seat.z - elbow.output.diameter / 2;
			var oldRadius = Math.min(Math.max(12, elbow.input.diameter * 0.23), (gap - 4) * 0.8);
			var oldWall = Math.max(1, Math.min(2, oldRadius * 0.12));
			var oldInertia = Math.PI / 4 * (Math.pow(oldRadius, 4) - Math.pow(oldRadius - oldWall, 4));
			for (z in [CobotWristLink.HUB_DEPTH + 0.5, elbow.seat.z / 2, elbow.seat.z - CobotWristLink.HUB_DEPTH - 0.5]) {
				var thickness = 0.2;
				var blank = Part.box(4 * elbow.output.diameter, 4 * elbow.output.diameter, thickness);
				var cutter = blank.translated(new Vector(0, 0, z)); blank.close();
				var section = local.intersect(cutter); cutter.close();
				var mass = section.massProperties(); section.close();
				if (!(mass.volume > 0)) throw "Wrist elbow has an empty load section";
				var tensor = mass.inertia;
				var lower = (tensor.xx + tensor.yy - Math.sqrt(Math.pow(tensor.xx - tensor.yy, 2) + 4 * tensor.xy * tensor.xy)) / 2;
				var inertia = lower / thickness - mass.volume * thickness / 12;
				minimum = Math.min(minimum, inertia);
			}
			if (!(minimum > 2 * oldInertia))
				throw '${arm.reference.designation} wrist elbow ${i + 1} section stiffness $minimum mm^4 must exceed twice the old neck $oldInertia mm^4';
			for (pair in [[i == 0 ? "shoulderRotor" : "wristRotor" + (i + 1), "link" + (i + 1)], ["link" + (i + 1), "joint" + (i + 2)]]) {
				checkContact(arm, state, parts, pair[0], pair[1]);
			}
			var rebuilt:CobotJoint = cast elbow.input.componentType().create(elbow.input.values());
			if (rebuilt.wristRotor != elbow.input.wristRotor || rebuilt.wristStator != elbow.input.wristStator ||
				rebuilt.geometryKey() != elbow.input.geometryKey()) throw "Wrist interface must survive a saved module recipe";
			Sys.println('${arm.reference.designation} elbow ${i + 1}: minimum sampled section $minimum mm^4 (old neck $oldInertia mm^4)');
		}
		if (!Std.isOfType(arm.links[5], CobotToolLink)) throw "Cobot tool flange needs an enclosed output housing";
		var terminal:CobotToolLink = cast arm.links[5];
		checkContact(arm, state, parts, "wristRotor6", "link6");
		checkContact(arm, state, parts, "link6", "toolFlange");
		var local = parts.local(terminal), envelope = terminal.geometry(Envelope);
		var outside = local.subtract(envelope); envelope.close();
		var escapedVolume = outside.volume(); outside.close();
		if (escapedVolume > 1e-3) throw "Tool housing preview must fit inside its collision envelope";
		var toolRejected = false;
		try new CobotToolLink(arm.modules[5].length + arm.toolFlange.thickness,
			arm.modules[5], arm.toolFlange) catch (_:Dynamic) toolRejected = true;
		if (!toolRejected) throw "An undersized tool housing must fail before constructing geometry";
		var rejected = false;
		try new CobotWristLink(arm.modules[4].length + arm.modules[5].diameter / 2,
			-Math.PI / 2, arm.modules[4], arm.modules[5]) catch (_:Dynamic) rejected = true;
		if (!rejected) throw "An undersized wrist layout must fail instead of shrinking its structural section";
	}

	static function checkArmConstruction(arm:CobotArm, state:AssemblyState, parts:PosedParts):Void {
		for (i in [1, 2]) {
			if (!Std.isOfType(arm.links[i], CobotArmLink)) throw "Long links need serviceable hollow housings";
			var link:CobotArmLink = cast arm.links[i];
			var shell = parts.local(link), envelope = link.geometry(Envelope);
			if (!shell.valid() || shell.solidCount() != 1) { envelope.close(); throw "Arm housing must be one valid connected solid"; }
			var escaped = shell.subtract(envelope); envelope.close();
			var escapedVolume = escaped.volume(); escaped.close();
			if (escapedVolume > 1e-3) throw "Arm shell escapes its collision envelope";
			// A connected solid can still have open outer quadrants where two
			// hollow cylinders meet. Probe the expected wall at both hub bends.
			for (side in 0...2) {
				var radius = side == 0 ? link.inputRadius : link.outputRadius;
				var centre = side == 0 ? new Vector(0, 0, CobotArmLink.CAP + radius) :
					new Vector(link.seat.x, 0, link.seat.z - CobotArmLink.CAP - radius);
				var sign = side == 0 ? 1.0 : -1.0;
				for (angle in [Math.PI / 6, Math.PI / 4, Math.PI / 3]) {
					var r = radius - CobotArmLink.WALL / 2;
					var point = centre.add(new Vector(sign * r * Math.sin(angle), 0, sign * r * Math.cos(angle)));
					var ball = Part.sphere(0.35), gauge = ball.translated(point); ball.close();
					var missing = gauge.subtract(shell); gauge.close();
					var missingVolume = missing.volume(); missing.close();
					if (missingVolume > 1e-3) throw 'Arm link ${i + 1} hub bend $side has a missing outer wall';
				}
			}
			var duct = Part.cylinderAlong(CobotArmLink.PORT_RADIUS - 0.3, link.bridgeStart,
				link.bridgeAxis, link.bridgeLength);
			var blocked = shell.intersect(duct); duct.close();
			var blockedVolume = blocked.volume(); blocked.close();
			if (blockedVolume > 1e-3) throw "Arm cable duct must be continuous through the bridge";
			for (t in [0.15, 0.85]) {
				var p = link.bridgeStart.add(link.bridgeEnd.subtract(link.bridgeStart).scale(t));
				var gauge = Part.cylinderAlong(CobotArmLink.PORT_RADIUS - 0.3,
					p.add(new Vector(0, -link.inputRadius - 1, 0)), Vector.Y(), link.inputRadius + 1);
				blocked = shell.intersect(gauge); gauge.close(); blockedVolume = blocked.volume(); blocked.close();
				if (blockedVolume > 1e-3) throw "Cable entry port must open into the internal duct";
			}
			var access = link.atService(CobotArmLink.roundedPlate(link.serviceLength - 20,
				link.serviceWidth - 20, CobotArmLink.WALL + 1, 1, -CobotArmLink.WALL - 0.5));
			// The opening's centre can be reached from outside with the cover removed.
			blocked = shell.intersect(access); access.close(); blockedVolume = blocked.volume(); blocked.close();
			if (blockedVolume > 1e-3) throw "Service opening must remain clear";
			checkContact(arm, state, parts, 'link${i + 1}', 'linkGasket${i + 1}');
			checkContact(arm, state, parts, 'linkGasket${i + 1}', 'linkCover${i + 1}');
			checkContact(arm, state, parts, 'armRotor${i + 1}', 'link${i + 1}');
			checkContact(arm, state, parts, 'link${i + 1}', 'joint${i + 2}');
			Sys.println('${arm.reference.designation} link ${i + 1}: continuous 8.4 mm duct gauge, two cable ports and removable service cover passed');
			Sys.stdout().flush();
		}
	}

	static function checkContact(arm:CobotArm, state:AssemblyState, parts:PosedParts, first:String, second:String):Void {
		var a = parts.member(arm, state, "wrist", first), b = parts.member(arm, state, "wrist", second);
		var joined = a.combine(b); a.close(); b.close();
		var connected = joined.solidCount() == 1; joined.close();
		if (!connected) throw 'Wrist interface $first / $second must share a mounting face';
	}

	static function dh(arm:CobotArm, q:Array<Float>):AssemblyFrame {
		var result = AssemblyFrames.translation(0, 0, arm.baseFlange.thickness);
		for (i in 0...6) {
			var c = Math.cos(q[i]), s = Math.sin(q[i]);
			var ca = Math.cos(arm.reference.alpha[i]), sa = Math.sin(arm.reference.alpha[i]);
			var a = arm.reference.a[i], d = arm.reference.d[i];
			result = AssemblyFrames.compose(result, AssemblyFrames.fromRotationMatrix(a * c, a * s, d,
				[c, -s * ca, s * sa, s, c * ca, -c * sa, 0, sa, ca]));
		}
		return result;
	}

	/**
	 * No two links share volume. Pairs are tested with a boolean only when their boxes (each link's own box carried to its
	 * pose, see `PosedParts.box`) are not `PosedParts.apart`, and each link is moved to its pose only for such a pair.
	 */
	static function checkOverlap(arm:CobotArm, state:AssemblyState, parts:PosedParts, rigidChecks:Map<String, Bool>):Void {
		var entries = arm.components(), boxes:Array<PartBox> = [], placed:Array<Null<Part>> = [];
		// Fixed-connected members have identical relative geometry at every pose.
		// Check each such pair once; pairs across moving joints are checked at every pose.
		var fixedParents:Map<String, String> = [], roots:Array<String> = [];
		for (joint in state.definition.joints) if (joint.type == materia.assembly.AssemblyDefinition.AssemblyJointType.Fixed &&
			joint.role == materia.assembly.AssemblyDefinition.AssemblyJointRole.Tree) fixedParents.set(joint.child, joint.parent);
		for (entry in entries) {
			var root = entry.id;
			while (fixedParents.exists(root)) root = fixedParents.get(root);
			roots.push(root);
			var local = parts.local(entry.component);
			if (!local.valid() || local.solidCount() != 1)
				throw '${arm.reference.designation}: ${entry.id} must be one valid solid';
			boxes.push(parts.box(entry.component, state.worldPose(entry.id)));
			placed.push(null);
		}
		function at(index:Int):Part {
			var part = placed[index];
			if (part == null) {
				part = parts.posed(entries[index].component, state.worldPose(entries[index].id));
				placed[index] = part;
			}
			return part;
		}
		try {
			for (i in 0...entries.length) for (j in i + 1...entries.length) {
				var rigid = roots[i] == roots[j], key = i + ":" + j;
				if (rigid && rigidChecks.exists(key)) continue;
				if (PosedParts.apart(boxes[i], boxes[j])) { if (rigid) rigidChecks.set(key, true); continue; }
				var common = at(i).intersect(at(j)), volume = common.volume();
				common.close();
				if (volume > 1e-3) throw '${arm.reference.designation}: ${entries[i].id} overlaps ${entries[j].id} by $volume mm³';
				if (rigid) rigidChecks.set(key, true);
			}
		} catch (error:Dynamic) {
			PosedParts.closeAll([for (part in placed) if (part != null) part]);
			throw error;
		}
		PosedParts.closeAll([for (part in placed) if (part != null) part]);
	}
}

function main():Void {
	if (Sys.args().indexOf("--export") >= 0) CobotArmPreview.export(); else CobotArmChecks.run();
}
