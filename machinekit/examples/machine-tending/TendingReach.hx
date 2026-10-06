import machinekit.robotics.CobotArm;
import machinekit.robotics.CobotClass;
import machinekit.robotics.ArmGripperTool;
import machinekit.assembly.AssemblyPreview;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import machinekit.assembly.MachineAssembly;
import machinekit.pneumatic.AirSupply;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import cadkit.modeling.Part;
import cadkit.parametric.GeometricConnectors;
import CncRouter.RouterPlate;
import robotkit.model.Frame;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.IkOptions;
import robotkit.manipulation.ArmClearance;
import robotkit.manipulation.Grasp;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

typedef TendingReachResult = {
	var cls:CobotClass;
	var riser:Float;
	var armBase:AssemblyFrame;
	var jointMargin:Float;
	var wristSine:Float;
	var positions:Array<Array<Float>>;
	var names:Array<String>;
	/** Required world TCP frames in millimetres, for the later cell generator. */
	var targets:Array<AssemblyFrame>;
}

/** Static reach, with the physical pneumatic gripper TCP at the vise, tray slots and both sides
 * of the doorway. Mill and blank geometry decide the poses; the gripper supplies its own envelope.
 */
class TendingReach {
	public static function run():Void {
		var result = study(new EnclosedBenchMill());
		if (result.cls != Reach850 || result.riser != 550 || result.names.length != 15 ||
			result.jointMargin < 0.1 || result.wristSine < 0.2)
			throw 'Tending reach changed class, riser, pose coverage or margins: ' +
				'${result.cls}/${result.riser}/${result.names.length}, joint=${result.jointMargin}, wrist=${result.wristSine}';
		trace('Tending reach: ${result.cls}, ${result.riser} mm riser, ${result.names.length} poses, ' +
			'${result.jointMargin * 100}% joint margin, wrist |sin(q5)| ${result.wristSine}');
	}

	public static function study(cell:EnclosedBenchMill):TendingReachResult {
		var failures:Array<String> = [];
		var armTool = new ArmGripperTool(32, BenchMill.STOCK_WIDTH, 60, 16);
		var gripper = armTool.spec;
		var millScene = SceneArtifact.decode(SceneArtifact.encode(AssemblyPreview.scene(cell, "reach-mill")));
		var millPhysical = AssemblyPhysicalPartView.fromSceneArtifact(millScene);
		var flat = AssemblyDefinitionFlattener.flatten(millScene.assemblyDefinition);
		var toolWidth = gripper.overallHalfWidth() * 2;
		var toolDepth = gripper.bodyDepth;
		var toolLength = gripper.bodyLength + gripper.fingerLength;
		var state = cell.state();
		for (i in 0...3) state.setJoint("mill/" + cell.mill.specs[i].id, cell.loadPosition[i]);
		state.setJoint("door", cell.doorStroke);
		var blank = state.worldPose("mill/stock");
		// Select the real width-compatible pinch from the stock's planar faces and collision hull.
		// Approach from above so the fingers pass through the blank while the jaws close on X,
		// clear of the vise's Y-axis clamping faces. The target TCP is the underside fingertip plane.
		var stock = new RouterPlate(BenchMill.STOCK_WIDTH, BenchMill.STOCK_DEPTH,
			BenchMill.STOCK_HEIGHT, "aluminium 6061", "Bearing block blank");
		var stockShape:Part = stock.geometry();
		var stockFaces = GeometricConnectors.planarFaces(GeometricConnectors.describeFaces(stockShape.shape));
		stockShape.close();
		var stockHull:Null<Array<Float>> = null;
		for (part in millPhysical.parts) if (part.id == "mill/stock") stockHull = part.collisionHull;
		if (stockHull == null) throw "Reach study needs the stock's physical collision hull";
		var grasp = Grasp.find([for (face in stockFaces)
			{origin: new robotkit.spatial.Vec3(face.origin.x, face.origin.y, face.origin.z),
				normal: new robotkit.spatial.Vec3(face.normal.x, face.normal.y, face.normal.z)}],
			[for (i in 0...Std.int(stockHull.length / 3)) new robotkit.spatial.Vec3(
				stockHull[3 * i], stockHull[3 * i + 1], stockHull[3 * i + 2])],
			new robotkit.spatial.Vec3(0, 0, 1), gripper.minimumPartWidth(), gripper.expectedPartWidth,
			gripper.maximumPartWidth(), gripper.fingerLength, new robotkit.spatial.Vec3(1, 0, 0));
		var graspLocal:AssemblyFrame = {x: grasp.contact.x, y: grasp.contact.y, z: grasp.contact.z,
			qx: grasp.toolRotation.x, qy: grasp.toolRotation.y, qz: grasp.toolRotation.z, qw: grasp.toolRotation.w};
		var graspWorld = AssemblyFrames.compose(blank, graspLocal);
		// Raise the fingertips and wrist clear of the blank before crossing the doorway.
		var approach = gripper.fingerLength + BenchMill.STOCK_HEIGHT;
		var names = ["vise", "outside door", "inside door"];
		var points = [[graspWorld.x, graspWorld.y, graspWorld.z]];

		var environment = [for (occurrence in flat.occurrences) if (occurrence.id != "mill/stock") {
			var physical = [for (part in millPhysical.parts) if (part.id == occurrence.definition) part][0];
			var pose = state.worldPose(occurrence.id), vertices:Array<Float> = [];
			if (physical.collisionHull == null) throw "Reach environment has no collision hull";
			var hull:Array<Float> = cast physical.collisionHull;
			for (i in 0...Std.int(hull.length / 3)) {
				var p = AssemblyFrames.transformPoint(pose, hull[3 * i], hull[3 * i + 1], hull[3 * i + 2]);
				vertices.push(p.x / 1000); vertices.push(p.y / 1000); vertices.push(p.z / 1000);
			}
			{name: occurrence.id, vertices: vertices};
		}];
		// The two tray banks are fixed to the tending cell, independent of tool or
		// arm selection. Slot pitch follows the stock blank and clearance; the
		// banks sit outside each side of the door opening, in front of the machine.
		var pitchX = BenchMill.STOCK_WIDTH + 20;
		var pitchY = BenchMill.STOCK_DEPTH + 20;
		var trayWidth = 3 * pitchX + 20, trayDepth = 2 * pitchY + 20;
		var trayOffset = cell.openingWidth / 2 + trayWidth / 2 + 20;
		var trayX = cell.opening.x;
		var trayY = cell.opening.y - (BenchMill.STOCK_DEPTH + cell.mill.column.width / 2 + 20);
		// Reserve 10 mm below the fingertip plane for the future pocket rails;
		// tray poses are the 10 mm approach above that support surface.
		var trayTop = graspWorld.z - 10;
		for (side in [-1, 1]) {
			var centre = trayX + side * trayOffset;
			environment.push({name: side < 0 ? "tray:infeed" : "tray:outfeed",
				vertices: boxVertices(centre - trayWidth / 2, centre + trayWidth / 2,
					trayY - trayDepth / 2, trayY + trayDepth / 2, trayTop - 20, trayTop)});
		}
		for (cls in [Reach850, Reach1300]) {
			var firstClassFailure:Null<String> = null, lastClassFailure:Null<String> = null;
			var riserFailure:Null<String> = null;
			var source = new CobotArm(cls, armTool);
			var reachAssembly = new MachineAssembly();
			reachAssembly.include("arm", source);
			reachAssembly.addComponent("airSupply", new AirSupply(), AssemblyFrames.translation(1200, -500, 0));
			reachAssembly.connectPorts("arm-air", "airSupply", "air", "arm", "compressedAir");
			if (reachAssembly.validate().length != 0) throw "Reach arm pneumatic service does not validate";
			if (graspWorld.x - toolWidth / 2 < cell.opening.x - cell.openingWidth / 2 ||
				graspWorld.x + toolWidth / 2 > cell.opening.x + cell.openingWidth / 2)
				throw "Reference gripper does not fit the actual doorway";
			var scene = SceneArtifact.decode(SceneArtifact.encode(AssemblyPreview.scene(reachAssembly, "reach-arm")));
			var converted = AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition,
				AssemblyPhysicalPartView.fromSceneArtifact(scene));
			var toolOccurrence = "arm/" + ArmGripperTool.TCP_PART;
			var toolPrefix = "arm/tool/";
			var baseCarrier = converted.partLinks.get("arm/baseFlange");
			var carrier = converted.partLinks.get(toolOccurrence);
			if (baseCarrier == null || carrier == null) throw "Reach arm has no base or tool link";
			var toolDefinition:Null<String> = null, tcpConnector:Null<AssemblyFrame> = null;
			var toolAssembly = AssemblyDefinitionFlattener.flatten(scene.assemblyDefinition);
			for (occurrence in toolAssembly.occurrences) if (occurrence.id == toolOccurrence)
				toolDefinition = occurrence.definition;
			for (component in toolAssembly.definitions) if (component.id == toolDefinition)
				for (connector in component.connectors) if (connector.name == "tcp") tcpConnector = connector.frame;
			if (tcpConnector == null) throw "Physical reach tool has no TCP connector";
			var frame = converted.model.addFrame(new Frame("reach-tcp", converted.model.links[carrier.link]));
			var tcpLocal:AssemblyFrame = {x: tcpConnector.x * scene.metresPerUnit,
				y: tcpConnector.y * scene.metresPerUnit, z: tcpConnector.z * scene.metresPerUnit,
				qx: tcpConnector.qx, qy: tcpConnector.qy, qz: tcpConnector.qz, qw: tcpConnector.qw};
			var tcp = AssemblyFrames.compose(carrier.offset, tcpLocal);
			frame.position = [tcp.x, tcp.y, tcp.z]; frame.rotation = [tcp.qx, tcp.qy, tcp.qz, tcp.qw];
			var rootLink = converted.model.links[baseCarrier.link].id;
			var arm = new Manipulator(converted.model, rootLink, frame.id);
			var interfaces:Array<robotkit.manipulation.ArmClearance.ClearanceInterfaceData> = [
				for (pair in converted.jointInterfaces)
					{first: pair.first, second: pair.second, margin: ArmClearance.INTERFACE_MARGIN}
			];
			var clearance = new ArmClearance(arm, [for (hull in converted.linkHulls)
				{name: hull.part, link: converted.model.links[hull.link].id, vertices: hull.vertices,
					tool: StringTools.startsWith(hull.part, toolPrefix)}], source.ready(),
				ArmClearance.MARGIN, ArmClearance.CONTACT_MARGIN, null, interfaces);
			var standOff = toolLength + source.baseFlange.flangeDiameter + toolDepth + 20;
			var baseY = cell.opening.y - standOff;
			// The doorway must pass the wrist as well as the tool. Keep its roll module
			// below the header; the vise approach can rise after it is inside.
			var wristPackage = toolLength + source.toolFlange.thickness + source.reference.d[5] + source.modules[5].length;
			var viaZ = Math.min(graspWorld.z + approach, cell.opening.z + cell.openingHeight - wristPackage - ArmClearance.MARGIN * 1000);
			if (viaZ <= cell.opening.z + ArmClearance.MARGIN * 1000) throw "Wrist and tool do not fit through the doorway";
			var allNames = names.copy(), allPoints = [points[0],
				[graspWorld.x, cell.opening.y - toolDepth, viaZ], [graspWorld.x, cell.opening.y + toolDepth, viaZ]];
			for (side in [-1, 1]) for (row in 0...2) for (column in 0...3) {
				allNames.push((side < 0 ? "infeed" : "outfeed") + '-$row-$column');
				allPoints.push([trayX + side * trayOffset + (column - 1) * pitchX,
					trayY + (row - 0.5) * pitchY, graspWorld.z]);
			}
			var options = new IkOptions(0.00001, 0.0001, 250, 0.001).reaching();
			for (level in 0...25) {
				var height = level * 50.0;
				var base = AssemblyFrames.compose(AssemblyFrames.translation(source.reference.d[3], baseY, height),
					AssemblyFrames.fromRotationMatrix(0, 0, 0, [0.0, 1, 0, -1, 0, 0, 0, 0, 1]));
				var inverse = transform(base).inverse();
				var solved:Array<Array<Float>> = [], targets:Array<AssemblyFrame> = [], seed = source.ready();
				var margin = 1.0, wrist = 1.0, valid = true;
				var failedAt:Null<String> = null, failedReason:Null<String> = null;
				for (pointIndex in 0...allPoints.length) {
					var point = allPoints[pointIndex];
					var selected:Null<Array<Float>> = null, selectedTarget:Null<AssemblyFrame> = null;
					var selectionFailure = "no tool roll candidate";
					var yaws = pointIndex == 0 ? [0.0, Math.PI] : [0.0, Math.PI / 2, Math.PI, 3 * Math.PI / 2];
					for (yaw in yaws) {
						var pose = graspFrame(point, graspWorld, yaw);
						var target = inverse.compose(new Transform3(new Vec3(point[0] / 1000, point[1] / 1000, point[2] / 1000),
							new Quat(pose.qx, pose.qy, pose.qz, pose.qw)));
						var solution = arm.solve(target, seed, options);
						if (!solution.converged) { selectionFailure = 'IK residual ${solution.positionError}/${solution.orientationError}'; continue; }
						if (!withinMargins(arm, solution.q)) {
							selectionFailure = 'joint margin ${jointMargin(arm, solution.q)}, wrist ${Math.abs(Math.sin(solution.q[4]))}';
							continue;
						}
						var selfHit = clearance.violation(solution.q);
						if (selfHit != null) { selectionFailure = 'self ${selfHit.a}/${selfHit.b}'; continue; }
						selected = solution.q; selectedTarget = pose; break;
					}
					// Numeric IK can converge on a self-colliding branch even when another
					// shoulder/elbow/wrist branch reaches the same frame. Search those branches
					// before rejecting the machine layout.
					if (selected == null || selectedTarget == null) for (yaw in yaws) {
						var pose = graspFrame(point, graspWorld, yaw);
						var target = inverse.compose(new Transform3(new Vec3(point[0] / 1000, point[1] / 1000, point[2] / 1000),
							new Quat(pose.qx, pose.qy, pose.qz, pose.qw)));
						for (pan in [-Math.PI / 2, 0.0, Math.PI / 2, Math.PI]) {
							if (selected != null) break;
							for (elbow in [-Math.PI / 2, Math.PI / 2]) {
								if (selected != null) break;
								for (wristSeed in [-Math.PI / 2, Math.PI / 2]) {
									var attempt = arm.solve(target,
										[pan, -Math.PI / 2, elbow, -elbow, wristSeed, 0.0], options);
									if (attempt.converged && withinMargins(arm, attempt.q) && clearance.violation(attempt.q) == null) {
										selected = attempt.q; selectedTarget = pose; break;
									}
								}
							}
						}
					}
					if (selected == null || selectedTarget == null) {
						valid = false; failedAt = allNames[pointIndex]; failedReason = selectionFailure; break;
					}
					solved.push(selected); targets.push(selectedTarget); seed = selected.copy();
				}
				if (valid) {
					var bodies:Array<robotkit.manipulation.ArmClearance.ClearanceBodyData> = [for (hull in converted.linkHulls)
						{name: hull.part, link: converted.model.links[hull.link].id, vertices: hull.vertices,
							tool: StringTools.startsWith(hull.part, toolPrefix)}];
					for (body in environment) {
						var vertices:Array<Float> = [];
						for (i in 0...Std.int(body.vertices.length / 3)) {
							var p = inverse.transformPoint(new Vec3(body.vertices[3 * i], body.vertices[3 * i + 1], body.vertices[3 * i + 2]));
							vertices.push(p.x); vertices.push(p.y); vertices.push(p.z);
						}
					bodies.push({name: "machine:" + body.name, link: rootLink, vertices: vertices, tool: false});
					}
					// Only tool-to-vise pairs get the reduced work-contact margin, and
					// only at the vise grasp pose. Door, mill, floor and trays keep 5 mm.
					var viseContactBodies = [for (body in environment)
						if (StringTools.startsWith(body.name, "mill/vise/")) "machine:" + body.name];
					var full = new ArmClearance(arm, bodies, source.ready(),
						ArmClearance.MARGIN, ArmClearance.CONTACT_MARGIN, viseContactBodies, interfaces);
					for (i in 0...solved.length) {
						var hit = full.violation(solved[i], i == 0);
						if (hit == null) continue;
						var point = allPoints[i];
						var alternative:Null<Array<Float>> = null;
						var alternativeTarget:Null<AssemblyFrame> = null;
						// Numeric IK has several shoulder, elbow and wrist branches. A colliding
						// first solution does not mean the pose is unreachable. Roll about the
						// approach axis to keep the physical valve and hose clear as well.
						var yaws = i == 0 ? [0.0, Math.PI] : [0.0, Math.PI / 2, Math.PI, 3 * Math.PI / 2];
						for (yaw in yaws) {
							if (alternative != null) break;
							var pose = graspFrame(point, graspWorld, yaw);
							var target = inverse.compose(new Transform3(new Vec3(point[0] / 1000, point[1] / 1000, point[2] / 1000),
								new Quat(pose.qx, pose.qy, pose.qz, pose.qw)));
							for (pan in [-Math.PI / 2, 0.0, Math.PI / 2, Math.PI]) {
							if (alternative != null) break;
							for (elbow in [-Math.PI / 2, Math.PI / 2]) {
								if (alternative != null) break;
								for (wristSeed in [-Math.PI / 2, Math.PI / 2]) {
									var attempt = arm.solve(target, [pan, -Math.PI / 2, elbow, -elbow, wristSeed, 0.0], options);
									if (attempt.converged && withinMargins(arm, attempt.q) && clearance.violation(attempt.q) == null &&
										full.violation(attempt.q, i == 0) == null) {
										alternative = attempt.q; alternativeTarget = pose; break;
										}
								}
							}
						}
						}
						if (alternative != null && alternativeTarget != null) {
							solved[i] = alternative; targets[i] = alternativeTarget;
						} else {
							failedAt = allNames[i];
							failedReason = 'clearance ${hit.a}/${hit.b} (${hit.distance} m)';
							valid = false; break;
						}
					}
					if (valid) {
						margin = 1; wrist = 1;
						for (q in solved) {
							margin = Math.min(margin, jointMargin(arm, q));
							wrist = Math.min(wrist, Math.abs(Math.sin(q[4])));
						}
						return {cls: cls, riser: height, armBase: base, jointMargin: margin, wristSine: wrist,
							positions: solved, names: allNames,
							targets: targets};
					}
				}
				if (!valid && failedAt != null) {
					var failure = '$height mm at $failedAt (${allPoints[allNames.indexOf(failedAt)]}): $failedReason';
					if (firstClassFailure == null) firstClassFailure = failure;
					lastClassFailure = failure;
					if (height == 550) riserFailure = failure;
				}
			}
			if (firstClassFailure != null && lastClassFailure != null)
				failures.push('$cls first ${firstClassFailure}, 550 mm ${riserFailure}, last ${lastClassFailure}');
		}
		throw 'Neither tending arm class reaches every required pose with the joint and wrist margins' +
			(failures.length == 0 ? "" : '; candidates: ${failures.join("; ")}');
	}
	static function jointMargin(arm:Manipulator, q:Array<Float>):Float {
		var margin = 1.0;
		for (i in 0...6) {
			var limit = arm.group.limitsOf(i), span = limit.upper - limit.lower;
			margin = Math.min(margin, Math.min(q[i] - limit.lower, limit.upper - q[i]) / span);
		}
		return margin;
	}
	static function withinMargins(arm:Manipulator, q:Array<Float>):Bool
		return jointMargin(arm, q) >= 0.1 && Math.abs(Math.sin(q[4])) >= 0.2;
	static function boxVertices(x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float):Array<Float> {
		var vertices:Array<Float> = [];
		for (x in [x0, x1]) for (y in [y0, y1]) for (z in [z0, z1]) {
			vertices.push(x / 1000); vertices.push(y / 1000); vertices.push(z / 1000);
		}
		return vertices;
	}
	static function graspFrame(point:Array<Float>, grasp:AssemblyFrame, yaw:Float):AssemblyFrame {
		var roll = Quat.fromAxisAngle(new Vec3(0, 0, 1), yaw);
		var rotation = roll.multiply(new Quat(grasp.qx, grasp.qy, grasp.qz, grasp.qw));
		return {x: point[0], y: point[1], z: point[2], qx: rotation.x, qy: rotation.y,
			qz: rotation.z, qw: rotation.w};
	}
	static function transform(frame:AssemblyFrame):Transform3 return new Transform3(
		new Vec3(frame.x / 1000, frame.y / 1000, frame.z / 1000), new Quat(frame.qx, frame.qy, frame.qz, frame.qw));
}

function main():Void TendingReach.run();
