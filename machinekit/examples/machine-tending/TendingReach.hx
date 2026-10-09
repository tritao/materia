import machinekit.robotics.CobotArm;
import machinekit.robotics.CobotClass;
import machinekit.assembly.AssemblyPreview;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import robotkit.model.Frame;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.IkOptions;
import robotkit.collision.CollisionClearance;
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

/** Static reach, with a tool-down TCP at the vise, tray slots and both sides of the doorway.
 * Mill and blank geometry decide the poses. The only tool used is the assumed future envelope.
 */
class TendingReach {
	public static function run():Void {
		var result = study(new EnclosedBenchMill());
		trace('Tending reach: ${result.cls}, ${result.riser} mm riser, ${result.names.length} poses, ' +
			'${result.jointMargin * 100}% joint margin, wrist |sin(q5)| ${result.wristSine}');
	}

	public static function study(cell:EnclosedBenchMill):TendingReachResult {
		var tool = new TendingToolEnvelope();
		var state = cell.state();
		for (i in 0...3) state.setJoint("mill/" + cell.mill.specs[i].id, cell.loadPosition[i]);
		state.setJoint("door", cell.doorStroke);
		var blank = state.worldPose("mill/stock");
		var grasp = {x: blank.x, y: blank.y, z: blank.z + BenchMill.STOCK_HEIGHT / 2};
		// Lift the fingers clear of the blank without raising the wrist into the header.
		var approach = tool.fingerDepth + BenchMill.STOCK_HEIGHT;
		var names = ["vise", "outside door", "inside door"];
		var points = [[grasp.x, grasp.y, grasp.z]];

		var millScene = SceneArtifact.decode(SceneArtifact.encode(AssemblyPreview.scene(cell, "reach-mill")));
		var millPhysical = AssemblyPhysicalPartView.fromSceneArtifact(millScene);
		var flat = AssemblyDefinitionFlattener.flatten(millScene.assemblyDefinition);
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
		for (cls in [Reach850, Reach1300]) {
			var source = new CobotArm(cls, tool);
			if (grasp.x - (tool.width + tool.openingTravel) / 2 < cell.opening.x - cell.openingWidth / 2 ||
				grasp.x + (tool.width + tool.openingTravel) / 2 > cell.opening.x + cell.openingWidth / 2)
				throw "Reference gripper does not fit the actual doorway";
			var scene = SceneArtifact.decode(SceneArtifact.encode(AssemblyPreview.scene(source, "reach-arm")));
			var converted = AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition,
				AssemblyPhysicalPartView.fromSceneArtifact(scene));
			var carrier = converted.partLinks.get(TendingToolEnvelope.TCP_PART);
			if (carrier == null) throw "Reach tool has no robot link";
			var frame = converted.model.addFrame(new Frame("reach-tcp", converted.model.links[carrier.link]));
			var tcp = AssemblyFrames.compose(carrier.offset, AssemblyFrames.translation(0, 0, tool.length / 1000));
			frame.position = [tcp.x, tcp.y, tcp.z]; frame.rotation = [tcp.qx, tcp.qy, tcp.qz, tcp.qw];
			var arm = new Manipulator(converted.model, converted.model.links[0].id, frame.id);
			var clearance = new robotkit.collision.CollisionClearance(arm, [for (hull in converted.linkHulls)
				{name: hull.part, link: converted.model.links[hull.link].id, vertices: hull.vertices,
					tool: StringTools.startsWith(hull.part, "tool/")}], source.ready(), () -> new collisionkit.native.NativeCollisionWorld(), 0, 0);
			var standOff = tool.length + source.baseFlange.flangeDiameter + tool.depth + 20;
			var baseX = source.reference.d[3], baseY = cell.opening.y - standOff;
			var pitchX = tool.width + tool.openingTravel + 10;
			var pitchY = tool.depth + 10;
			var trayOffset = source.baseFlange.flangeDiameter / 2 + 2 * pitchX;
			// The doorway must pass the wrist as well as the tool. Keep its roll module
			// below the header; the vise approach can rise after it is inside.
			var wristPackage = tool.length + source.toolFlange.thickness + source.reference.d[5] + source.modules[5].length;
			var viaZ = Math.min(grasp.z + approach, cell.opening.z + cell.openingHeight - wristPackage - CollisionClearance.MARGIN * 1000);
			if (viaZ <= cell.opening.z + CollisionClearance.MARGIN * 1000) throw "Wrist and tool do not fit through the doorway";
			var allNames = names.copy(), allPoints = [points[0],
				[grasp.x, cell.opening.y - tool.depth, viaZ], [grasp.x, cell.opening.y + tool.depth, viaZ]];
			for (side in [-1, 1]) for (row in 0...2) for (column in 0...3) {
				allNames.push((side < 0 ? "infeed" : "outfeed") + '-$row-$column');
				allPoints.push([baseX + side * trayOffset + (column - 1) * pitchX,
					baseY + (row - 0.5) * pitchY, grasp.z]);
			}
			var options = new IkOptions(0.00001, 0.0001, 250, 0.001).reaching();
			for (level in 0...25) {
				var height = level * 50.0;
				var base = AssemblyFrames.compose(AssemblyFrames.translation(baseX, baseY, height),
					AssemblyFrames.fromRotationMatrix(0, 0, 0, [0.0, 1, 0, -1, 0, 0, 0, 0, 1]));
				var inverse = transform(base).inverse();
				var solved:Array<Array<Float>> = [], seed = source.ready();
				var margin = 1.0, wrist = 1.0, valid = true;
				for (point in allPoints) {
					var target = inverse.compose(new Transform3(new Vec3(point[0] / 1000, point[1] / 1000, point[2] / 1000),
						new Quat(1, 0, 0, 0)));
					var solution = arm.solve(target, seed, options);
					if (!solution.converged) { valid = false; break; }
					var q = solution.q;
					if (!withinMargins(arm, q) || clearance.violation(q) != null) { valid = false; break; }
					solved.push(q); seed = q.copy();
				}
				if (valid) {
					var bodies:Array<robotkit.manipulation.ClearanceBodyData> = [for (hull in converted.linkHulls)
						{name: hull.part, link: converted.model.links[hull.link].id, vertices: hull.vertices,
							tool: StringTools.startsWith(hull.part, "tool/")}];
					for (body in environment) {
						var vertices:Array<Float> = [];
						for (i in 0...Std.int(body.vertices.length / 3)) {
							var p = inverse.transformPoint(new Vec3(body.vertices[3 * i], body.vertices[3 * i + 1], body.vertices[3 * i + 2]));
							vertices.push(p.x); vertices.push(p.y); vertices.push(p.z);
						}
						bodies.push({name: "machine:" + body.name, link: converted.model.links[0].id, vertices: vertices, tool: false});
					}
					var full = new robotkit.collision.CollisionClearance(arm, bodies, source.ready(), () -> new collisionkit.native.NativeCollisionWorld());
					for (i in 0...solved.length) {
						var hit = full.violation(solved[i], i == 0);
						if (hit == null) continue;
						var point = allPoints[i];
						var target = inverse.compose(new Transform3(new Vec3(point[0] / 1000, point[1] / 1000, point[2] / 1000),
							new Quat(1, 0, 0, 0)));
						var alternative:Null<Array<Float>> = null;
						// Numeric IK has several shoulder, elbow and wrist branches. A colliding
						// first solution does not mean that the pose is unreachable.
						for (pan in [-Math.PI / 2, 0.0, Math.PI / 2, Math.PI]) {
							if (alternative != null) break;
							for (elbow in [-Math.PI / 2, Math.PI / 2]) {
								if (alternative != null) break;
								for (wristSeed in [-Math.PI / 2, Math.PI / 2]) {
									var attempt = arm.solve(target, [pan, -Math.PI / 2, elbow, -elbow, wristSeed, 0.0], options);
									if (attempt.converged && withinMargins(arm, attempt.q) && clearance.violation(attempt.q) == null &&
										full.violation(attempt.q, i == 0) == null) { alternative = attempt.q; break; }
								}
							}
						}
						if (alternative != null) solved[i] = alternative; else {
							trace('Reach candidate $cls/$height mm at ${allNames[i]}: ${hit.a}/${hit.b}, ${hit.distance} m');
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
							targets: [for (point in allPoints) toolDownFrame(point)]};
					}
				}
			}
		}
		throw "Neither tending arm class reaches every required pose with the joint and wrist margins";
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
	static function toolDownFrame(point:Array<Float>):AssemblyFrame
		return {x: point[0], y: point[1], z: point[2], qx: 1, qy: 0, qz: 0, qw: 0};
	static function transform(frame:AssemblyFrame):Transform3 return new Transform3(
		new Vec3(frame.x / 1000, frame.y / 1000, frame.z / 1000), new Quat(frame.qx, frame.qy, frame.qz, frame.qw));
}

function main():Void TendingReach.run();
