import collisionkit.native.NativeCollisionWorld;
import collisionkit.CollisionDescription;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionMargins;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import robotkit.collision.RobotCollision;
import robotkit.collision.RobotCollision.RobotCollisionOptions;
import motionkit.robot.CollisionPlannerSpace;
import motionkit.robot.ManipulatorPlannerSpace;
import motionkit.planner.RrtConnect;
import machinekit.robotics.CobotArm;
import machinekit.robotics.CobotClass;
import machinekit.robotics.DualArmGripperTool;
import machinekit.assembly.AssemblyPreview;
import materia.assembly.AssemblyFrames;
import materia.project.SceneArtifact;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import machinekit.assembly.MachineAssembly;
import machinekit.pneumatic.AirSupply;
import machinekit.control.DigitalSensor;
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import cadkit.modeling.Part;
import cadkit.modeling.AssemblyState;
import cadkit.parametric.GeometricConnectors;
import CncRouter.RouterPlate;
import robotkit.model.Frame;
import robotkit.manipulation.Manipulator;
import robotkit.manipulation.IkOptions;
import robotkit.manipulation.Grasp;
import robotkit.collision.CollisionClearance;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;

typedef TendingReachResult = {
	var cls:CobotClass;
	var riser:Float;
	/** Placement of the complete arm subassembly on the cell riser. */
	var armPlacement:AssemblyFrame;
	/** Base-flange part frame after the same fixed mate used by the assembled cell. */
	var armBaseFlange:AssemblyFrame;
	var jointMargin:Float;
	var wristSine:Float;
	/** The checked vertical lift clear of the seat and jaws plus two clearance margins, in millimetres. */
	var handlingApproachMm:Float;
	/** Joint pose that parks the arm facing away from the mill during CNC cutting. */
	var parkPose:Array<Float>;
	/** TCP height over a tray slot that clears the seated blank before transit. */
	var trayClearanceMm:Float;
	/** Reach-certified joint poses, each keyed by its station and actual gripper TCP. */
	var poses:Array<TendingReachPose>;
	/** The actual cell-fixed pocket trays and their seat-floor poses, all in millimetres. */
	var trays:Array<TendingTrayLayout>;
	var tableFrame:AssemblyFrame;
	var tableOffset:Float;
	var tableWidth:Float;
	var tableDepth:Float;
	var tableTop:Float;
}

typedef TendingReachPose = {
	var name:String;
	var tool:String;
	var position:Array<Float>;
	var target:AssemblyFrame;
	/** World-Z roll of this tool at the station, independent of the part heading. */
	var toolYaw:Float;
}
typedef TendingReachRequest = {var name:String; var tool:String; var point:Array<Float>; var toolOffset:Transform3;}
typedef TendingTraySlot = {var name:String; var frame:AssemblyFrame;}
typedef TendingTrayLayout = {
	var id:String;
	var frame:AssemblyFrame;
	var slots:Array<TendingTraySlot>;
	var width:Float;
	var depth:Float;
}
private typedef ReachClearanceCheck = {
	var space:ManipulatorPlannerSpace;
	var description:CollisionDescription;
	var build:collisionkit.CollisionBuild;
}

/** Static reach, with each physical gripper TCP at the vise, tray slots and both sides of the
 * doorway. Mill and blank geometry decide the poses; each gripper supplies its own envelope.
 * MotionKit certifies the actual timed joint paths when it admits the generated moveVia plans.
 */
class TendingReach {
	/** A pocket's free pitch accounts for the opposing tool envelopes and both clearance sides. */
	public static inline var TRAY_SLOT_CLEARANCE_MM:Float =
		2 * DualArmGripperTool.INTER_GRIPPER_CLEARANCE_MM + 2 * CollisionClearance.MARGIN * 1000;
	/** Assumed placement allowance beyond the general collision margin for fixed sensor heads. */
	public static inline var PRESENCE_HEAD_MOUNT_ALLOWANCE_MM:Float = 1.0;
	/** Assumed 6 x 40 mm replaceable pads leave handling clearance around the 60 mm stock face. */
	public static function tendingTool():DualArmGripperTool
		return new DualArmGripperTool(32, BenchMill.STOCK_WIDTH, 60, 16, false,
			BenchMill.STOCK_DEPTH - 4 * CollisionClearance.MARGIN * 1000,
			6, 10);

	public static function trayPitchX():Float return BenchMill.STOCK_WIDTH + TRAY_SLOT_CLEARANCE_MM;
	public static function trayPitchY():Float return BenchMill.STOCK_DEPTH + TRAY_SLOT_CLEARANCE_MM;

	/** Keep the sensor outside the blank edge with the global margin and its mount allowance. */
	public static function presenceHeadOffsetX():Float {
		return -(BenchMill.STOCK_WIDTH / 2 + DigitalSensor.BODY_WIDTH / 2 + CollisionClearance.MARGIN * 1000 +
			PRESENCE_HEAD_MOUNT_ALLOWANCE_MM);
	}

	/** Place the sensor beyond the gripper housing depth, toward the outside edge of its tray row. */
	public static function presenceHeadOffsetY():Float {
		var spec = tendingTool().spec;
		var gripperHalfDepth = Math.max(spec.bodyDepth, spec.fingerDepth) / 2;
		return gripperHalfDepth + DigitalSensor.BODY_DEPTH / 2 + CollisionClearance.MARGIN * 1000;
	}

	/** `slot` is in its tray frame; the sensor sits outward from either row. */
	public static function presenceHeadFrame(slot:AssemblyFrame):AssemblyFrame {
		var outwardY = slot.y < 0 ? -presenceHeadOffsetY() : presenceHeadOffsetY();
		return AssemblyFrames.compose(slot, AssemblyFrames.translation(presenceHeadOffsetX(), outwardY, 10));
	}

	public static function run():Void {
		var cell = new EnclosedBenchMill();
		var result = study(cell);
		if (result.cls != Reach1300 || result.riser != 550 || result.poses.length != 96 ||
			result.jointMargin < 0.1 || result.wristSine < 0.2)
			throw 'Tending reach changed class, riser, pose coverage or margins: ' +
				'${result.cls}/${result.riser}/${result.poses.length}, joint=${result.jointMargin}, wrist=${result.wristSine}';
		var viseA = toolPose(result, "vise", DualArmGripperTool.TOOL_CHANNEL_A);
		var viseB = toolPose(result, "vise", DualArmGripperTool.TOOL_CHANNEL_B);
		var infeed = [for (tray in result.trays) if (tray.id == "infeedTray") tray][0];
		if (infeed == null || infeed.slots.length != 6 ||
			Math.abs(infeed.slots[1].frame.x - infeed.slots[0].frame.x - trayPitchX()) > 1e-9 ||
			Math.abs(infeed.slots[3].frame.y - infeed.slots[0].frame.y - trayPitchY()) > 1e-9)
			throw "Reach poses do not use the occupied tray pitch derived from stock and the two gripper envelopes";
		for (tray in result.trays) for (slot in tray.slots) {
			var slotLocal = AssemblyFrames.compose(AssemblyFrames.inverse(tray.frame), slot.frame);
			var local = presenceHeadFrame(slotLocal);
			if (local.y * slotLocal.y <= 0)
				throw 'Presence head for ${tray.id}/${slot.name} is not on the outside of its tray row';
			if (local.x - DigitalSensor.BODY_WIDTH / 2 < -tray.width / 2 - 1e-9 ||
				local.x + DigitalSensor.BODY_WIDTH / 2 > tray.width / 2 + 1e-9 ||
				local.y - DigitalSensor.BODY_DEPTH / 2 < -tray.depth / 2 - 1e-9 ||
				local.y + DigitalSensor.BODY_DEPTH / 2 > tray.depth / 2 + 1e-9)
				throw 'Presence head for ${tray.id}/${slot.name} extends beyond the cell-fixed tray frame';
		}
		var toolPoseDelta = 0.0;
		for (joint in 0...viseA.length) toolPoseDelta = Math.max(toolPoseDelta, Math.abs(viseA[joint] - viseB[joint]));
		if (toolPoseDelta < 0.01) throw "The reach study did not account for the second gripper's TCP offset";
		var baseStandOff = cell.opening.y - result.armPlacement.y;
		trace('Tending reach: ${result.cls}, ${result.riser} mm riser, ${result.poses.length} poses, ' +
			'${baseStandOff} mm base stand-off, ' +
			'${result.jointMargin * 100}% joint margin, wrist |sin(q5)| ${result.wristSine}');
	}

	public static function toolPose(result:TendingReachResult, name:String, channel:String):Array<Float> {
		for (pose in result.poses) if (pose.name == name && pose.tool == channel) return pose.position;
		throw 'Tending reach has no checked pose "$name" for "$channel"';
	}

	public static function toolYaw(result:TendingReachResult, name:String, channel:String):Float {
		for (pose in result.poses) if (pose.name == name && pose.tool == channel) return pose.toolYaw;
		throw 'Tending reach has no checked orientation "$name" for "$channel"';
	}

	static function trayStock(station:String):Null<String> {
		if (StringTools.endsWith(station, "-approach")) return null;
		if (StringTools.startsWith(station, "infeed-"))
			return "infeedTray/blank-" + station.substr("infeed-".length);
		return null;
	}

	/** Main's description owns fixed geometry, articulation and pair-specific contact rules. */
	static function reachClearances(arm:Manipulator, data:Array<robotkit.manipulation.ClearanceBodyData>,
		reference:Array<Float>, contactNames:Array<String>, contact:Bool,
		interfaces:Array<cadbridge.AssemblySimulationBridge.AssemblyJointInterface>):Array<ReachClearanceCheck> {
		var description = new CollisionDescription();
		var model = robotkit.kinematics.RobotKinematics.compile(arm.robot);
		var state = new kinematicskit.KinematicState(model, arm.stateOf(reference).q);
		var bodies = collisionkit.kinematics.ModelBodies.describe(description, model, "reach-arm", 0, state, "reach reference", true);
		var rootPose = description.bodies[bodies.bodies[model.bodyIndex(arm.rootLink)]].pose;
		var margins = new CollisionMargins(data.length + 1, CollisionClearance.MARGIN);
		var objects = new Map<String, Array<Int>>();
		for (i in 0...data.length) {
			var hull = data[i], group = i + 1;
			var object:Int;
			if (StringTools.startsWith(hull.name, "machine:"))
				object = description.addFixed(hull.name, rootPose, CollisionGeometry.Convex(hull.vertices), group, 0, "cell hull");
			else {
				var link = model.bodyIndex(hull.link);
				if (link < 0) throw 'Reach hull "${hull.name}" lost its model link';
				var carrier = bodies.follow(link, hull.name + "#" + i, group);
				object = description.addObject(hull.name, carrier, collisionkit.CollisionPose.identity(), CollisionGeometry.Convex(hull.vertices));
			}
			var pieces = objects.get(hull.name);
			if (pieces == null) { pieces = []; objects.set(hull.name, pieces); }
			pieces.push(object);
		}
		for (pair in interfaces) {
			var first = objects.get(pair.first), second = objects.get(pair.second);
			if (first != null && second != null) for (a in first) for (b in second)
				margins.set(description.bodies[description.objects[a].body].group,
					description.bodies[description.objects[b].body].group, 0.001);
		}
		if (contact) for (hull in data) if (hull.tool) for (name in contactNames) {
			var active = objects.get(hull.name), targets = objects.get(name);
			if (active == null || targets == null) continue;
			for (a in active) for (b in targets) {
				margins.set(description.bodies[description.objects[a].body].group,
					description.bodies[description.objects[b].body].group, CollisionClearance.CONTACT_MARGIN);
				if (gripPad(hull.name) && StringTools.startsWith(name, "machine:infeedTray/blank-"))
					description.setObjectRule(a, b, CollisionPairRule.Allow, CollisionPairStatus.ProcessContact);
			}
		}
		var articulation = bodies.declare();
		var build = description.build(new NativeCollisionWorld());
		var space = new ManipulatorPlannerSpace(new CollisionPlannerSpace(description, build, bodies, margins), arm, reference);
		return [{space: space, description: description, build: build}];
	}

	static function gripPad(name:String):Bool
		return StringTools.endsWith(name, "/fingerLeft") || StringTools.endsWith(name, "/fingerRight");

	static function clearanceViolation(check:ReachClearanceCheck, q:Array<Float>):Null<robotkit.manipulation.ClearanceViolation> {
		var found = check.space.violation(q);
		if (found == null) return null;
		var a = check.build.objectOf(found.a), b = check.build.objectOf(found.b);
		if (a < 0 || b < 0) throw "Reach violation lost its described hull";
		return {a: check.description.objects[a].name, b: check.description.objects[b].name,
			distance: found.distance, required: found.required};
	}

	static function firstViolation(checks:Array<ReachClearanceCheck>, q:Array<Float>):Null<robotkit.manipulation.ClearanceViolation> {
		for (check in checks) {
			var found = clearanceViolation(check, q);
			if (found != null) return found;
		}
		return null;
	}

	static function disposeClearances(checks:Array<ReachClearanceCheck>):Void
		for (check in checks) check.space.dispose();

	public static function study(cell:EnclosedBenchMill):TendingReachResult {
		// Reach uses the same physical blank hull as machining, but the tending cell starts with an
		// empty vise. Study a loaded reference machine so the stock remains geometry input only.
		if (!cell.includesWorkpiece) return study(new EnclosedBenchMill());
		var failures:Array<String> = [];
		var armTool = tendingTool();
		var gripper = armTool.spec;
		var millScene = SceneArtifact.decode(SceneArtifact.encode(AssemblyPreview.scene(cell, "reach-mill")));
		var millPhysical = AssemblyPhysicalPartView.fromSceneArtifact(millScene);
		var flat = AssemblyDefinitionFlattener.flatten(millScene.assemblyDefinition);
		var toolWidth = armTool.envelopeWidthMm;
		var toolDepth = armTool.envelopeDepthMm;
		var toolLength = gripper.bodyLength + gripper.fingerLength + armTool.adapterThicknessMm - 12;
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
		var loadWorld = AssemblyFrames.compose(state.worldConnector("mill/vise/body", "load"), graspLocal);
		// Measure axial engagement from the hull in the grasp frame. The tool's
		// +Z points into the stock, opposite the outward approach direction.
		var graspInverse = AssemblyFrames.inverse(graspLocal);
		var minimumAxial = Math.POSITIVE_INFINITY, maximumAxial = Math.NEGATIVE_INFINITY;
		for (i in 0...Std.int(stockHull.length / 3)) {
			var point = AssemblyFrames.transformPoint(graspInverse, stockHull[i * 3], stockHull[i * 3 + 1], stockHull[i * 3 + 2]);
			minimumAxial = Math.min(minimumAxial, point.z);
			maximumAxial = Math.max(maximumAxial, point.z);
		}
		if (!Math.isFinite(minimumAxial + maximumAxial)) throw "Reach grasp has no carried-stock collision points";
		// Side pads constrain width but have no axial locating stop. A confirmed
		// grasp can settle anywhere along the stock's side face, so reserve its
		// full axial span when lifting the second gripper past a seated workpiece.
		var gripAxialSpan = maximumAxial - minimumAxial;
		// With no axial locator, bound the bottom at one full side-face span
		// below the TCP instead of treating tool-local Z as a world height.
		var payloadFloorOffset = -gripAxialSpan;
		// Lift the complete allowed payload envelope above tray lands and vise jaws.
		var vise = cell.mill.vise;
		if (vise == null) throw "Tending reach needs the mill's preset vise";
		var movingJaw = state.worldPose("mill/vise/movingJaw");
		var viseJawClearance = movingJaw.z + vise.movingJaw.height - blank.z + 2 * CollisionClearance.MARGIN * 1000;
		var approach = Math.max(PartTray.SEAT_Z + 2 * CollisionClearance.MARGIN * 1000, viseJawClearance) + gripAxialSpan;
		// The TCP is at the inserted finger tips on the blank's far face. Raise it
		// by the handling approach plus blank height and clearance so the fingers
		// clear the seated part before transit; the body extends behind the TCP.
		var trayClearanceMm = approach + gripAxialSpan + 2 * CollisionClearance.MARGIN * 1000;
		var viseNormal = AssemblyFrames.transformVector(blank, grasp.approachAxis.x,
			grasp.approachAxis.y, grasp.approachAxis.z);
		var environment = [for (occurrence in flat.occurrences) if (occurrence.id != "mill/stock") {
			var physical = [for (part in millPhysical.parts) if (part.id == occurrence.definition) part][0];
			var pose = state.worldPose(occurrence.id), vertices:Array<Float> = [];
			if (physical.collisionHull == null) throw "Reach environment has no collision hull";
			var hull = physical.collisionHull;
			for (i in 0...Std.int(hull.length / 3)) {
				var p = AssemblyFrames.transformPoint(pose, hull[3 * i], hull[3 * i + 1], hull[3 * i + 2]);
				vertices.push(p.x / 1000); vertices.push(p.y / 1000); vertices.push(p.z / 1000);
			}
			{name: occurrence.id, vertices: vertices};
		}];
		// Trays are used with the door closed; enclosure transit uses the open
		// leaf. Derive both poses from its joint, including every attached part.
		var openPoses = new Map<String, AssemblyFrame>();
		for (occurrence in flat.occurrences) openPoses.set(occurrence.id, state.worldPose(occurrence.id));
		state.setJoint("door", 0);
		for (occurrence in flat.occurrences) {
			var open = openPoses.get(occurrence.id), closed = state.worldPose(occurrence.id);
			if (open == null) throw "Reach door image lost an occurrence";
			if (Math.abs(open.x - closed.x) + Math.abs(open.y - closed.y) + Math.abs(open.z - closed.z) < 1e-6) continue;
			var physical = [for (part in millPhysical.parts) if (part.id == occurrence.definition) part][0];
			var hull = physical.collisionHull, vertices:Array<Float> = [];
			if (hull == null) throw "Reach closed door has no collision hull";
			for (i in 0...Std.int(hull.length / 3)) {
				var point = AssemblyFrames.transformPoint(closed, hull[3 * i], hull[3 * i + 1], hull[3 * i + 2]);
				vertices.push(point.x / 1000); vertices.push(point.y / 1000); vertices.push(point.z / 1000);
			}
			environment.push({name: "door-closed/" + occurrence.id, vertices: vertices});
		}
		state.setJoint("door", cell.doorStroke);
		// Build and check the real pocket geometry. The old tray envelope sat 10 mm
		// below the blank and missed the collision body the arm would actually meet.
		// Free blanks change pockets and follow the robot during a cycle; the mission
		// planner checks their live poses rather than baking one occupancy into this study.
		var trayAssembly = new PartTray(2, 3, BenchMill.STOCK_WIDTH, BenchMill.STOCK_DEPTH,
			trayPitchX(), trayPitchY());
		// The two tray banks are fixed to the tending cell, independent of tool or
		// arm selection. Their positions are derived from the cell opening and the
		// real blank-seat height. These same layouts become assembled cell parts.
		var trayWidth = trayAssembly.width, trayDepth = trayAssembly.depth;
		var tableWidth = trayWidth + 40;
		var tableDepth = trayDepth + 40;
		var trayOffset = cell.openingWidth / 2 + trayWidth / 2 + 20;
		var trayX = cell.opening.x;
		var trayY = cell.opening.y - (BenchMill.STOCK_DEPTH + cell.mill.column.width / 2 + 20);
		// Tray seats support the blank's bottom datum at the same height as the vise parallels.
		var traySeatZ = blank.z;
		var trayLayouts:Array<TendingTrayLayout> = [];
		for (side in [-1, 1]) {
			var centre = trayX + side * trayOffset;
			var id = side < 0 ? "infeedTray" : "outfeedTray";
			var frame = AssemblyFrames.translation(centre, trayY, traySeatZ - trayAssembly.seat(0, 0).z);
			var slots:Array<TendingTraySlot> = [for (slot in trayAssembly.seatFrames())
				{name: slot.name, frame: AssemblyFrames.compose(frame, slot.frame)}];
			trayLayouts.push({id: id, frame: frame, slots: slots,
				width: trayAssembly.width, depth: trayAssembly.depth});
			for (piece in trayAssembly.collisionPieces()) {
				var pieceFrame = AssemblyFrames.compose(frame, piece.frame);
				var vertices:Array<Float> = [];
				for (index in 0...Std.int(piece.vertices.length / 3)) {
					var point = AssemblyFrames.transformPoint(pieceFrame, piece.vertices[index * 3],
						piece.vertices[index * 3 + 1], piece.vertices[index * 3 + 2]);
					vertices.push(point.x / 1000); vertices.push(point.y / 1000); vertices.push(point.z / 1000);
				}
				environment.push({name: '$id/${piece.id}', vertices: vertices});
			}
			// Presence heads are fixed to the tray edge, outside the open finger sweep.
			// Include their real box hulls so the static reach study and runtime planner
			// evaluate the same sensor hardware.
			for (slot in slots) {
				var slotLocal = AssemblyFrames.compose(AssemblyFrames.inverse(frame), slot.frame);
				var sensorFrame = AssemblyFrames.compose(frame, presenceHeadFrame(slotLocal));
				var vertices:Array<Float> = [];
				for (x in [-DigitalSensor.BODY_WIDTH / 2, DigitalSensor.BODY_WIDTH / 2])
					for (y in [-DigitalSensor.BODY_DEPTH / 2, DigitalSensor.BODY_DEPTH / 2])
						for (z in [0.0, DigitalSensor.BODY_HEIGHT]) {
							var point = AssemblyFrames.transformPoint(sensorFrame, x, y, z);
							vertices.push(point.x / 1000); vertices.push(point.y / 1000); vertices.push(point.z / 1000);
						}
				environment.push({name: '$id/${slot.name}-present', vertices: vertices});
			}
		}
		var tableTop = traySeatZ - PartTray.SEAT_Z;
		var tableOffset = trayOffset;
		var tableFrame = AssemblyFrames.translation(cell.opening.x, trayY, 0);
		var table = new TendingCellTable(tableWidth, tableDepth, tableTop, tableOffset);
		for (piece in table.collisionPieces()) {
			var pieceFrame = AssemblyFrames.compose(tableFrame, piece.frame);
			var vertices:Array<Float> = [];
			for (index in 0...Std.int(piece.vertices.length / 3)) {
				var point = AssemblyFrames.transformPoint(pieceFrame, piece.vertices[index * 3],
					piece.vertices[index * 3 + 1], piece.vertices[index * 3 + 2]);
				vertices.push(point.x / 1000); vertices.push(point.y / 1000); vertices.push(point.z / 1000);
			}
			environment.push({name: 'cellTable/${piece.id}', vertices: vertices});
		}
		// Check the complete initial loose-stock layout when choosing a station roll.
		// Later moving and outfeed blanks are checked at their live poses by each mission plan.
		for (tray in trayLayouts) if (tray.id == "infeedTray") for (slot in tray.slots) {
			var vertices:Array<Float> = [];
			for (index in 0...Std.int(stockHull.length / 3)) {
				var point = AssemblyFrames.transformPoint(slot.frame, stockHull[index * 3],
					stockHull[index * 3 + 1], stockHull[index * 3 + 2]);
				vertices.push(point.x / 1000); vertices.push(point.y / 1000); vertices.push(point.z / 1000);
			}
			environment.push({name: '${tray.id}/blank-${slot.name.substr(5)}', vertices: vertices});
		}
	// The collision-screened design is the 1.3 m cobot on the cell's 550 mm riser.
	// `study` certifies its actual gripper and every assembled cell pose; the design
	// search is not repeated during each project load.
	for (cls in [Reach1300]) {
			var firstClassFailure:Null<String> = null, lastClassFailure:Null<String> = null;
			var riserFailures:Array<String> = [];
			var firstFailureByHeight = new Map<Int, String>();
			var reportedBaseX = new Map<String, Bool>();
			var source = new CobotArm(cls, armTool);
			var parkPose = source.ready();
			// The base joint turns the folded ready posture out of the enclosure. Keeping
			// this cell-specific park explicit avoids occupying the spindle's travel lane.
			parkPose[0] = Math.PI;
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
			var toolOccurrence = "arm/" + DualArmGripperTool.GRIPPER_A_BODY;
			var toolPrefix = "arm/tool/";
			var baseCarrier = converted.partLinks.get("arm/baseFlange");
			var carrier = converted.partLinks.get(toolOccurrence);
			if (baseCarrier == null || carrier == null) throw "Reach arm has no base or tool link";
			var toolDefinition:Null<String> = null, tcpConnector:Null<AssemblyFrame> = null;
			var toolAssembly = AssemblyDefinitionFlattener.flatten(scene.assemblyDefinition);
			var loadedHulls = sweepGripperClosure(converted, scene, gripper);
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
			var clearance = reachClearances(arm, [for (hull in converted.linkHulls)
				{name: hull.part, link: converted.model.links[hull.link].id, vertices: hull.vertices,
					tool: StringTools.startsWith(hull.part, toolPrefix)}], source.ready(), [], false, converted.jointInterfaces)[0];
			var toolStandOff = toolLength + source.baseFlange.flangeDiameter + toolDepth + 20;
			// The static tool envelope includes its assumed hose clearance. Keep the
			// runtime's full obstacle margin beyond that envelope at the door plane.
			var doorStandOff = toolDepth + CollisionClearance.MARGIN * 1000;
			var classBaseY = cell.opening.y - toolStandOff;
			// Keep the exterior holding pose at the checked lift height. The doorway
			// pass uses the lowest pose allowed by the upper wrist-package clearance;
			// stock and tool below the TCP are checked from their physical hulls.
			var wristPackage = toolLength + source.toolFlange.thickness + source.reference.d[5] + source.modules[5].length;
			var portalTopZ = cell.opening.z + cell.openingHeight - wristPackage - CollisionClearance.MARGIN * 1000;
			var stagingZ = Math.min(graspWorld.z + approach, portalTopZ);
			var payloadFloorZ = graspWorld.z + payloadFloorOffset;
			var floorClearance = CollisionClearance.MARGIN * 1000;
			var portalFloorZ = cell.opening.z + floorClearance - payloadFloorOffset;
			if (portalFloorZ >= portalTopZ)
				throw 'The carried stock and wrist package do not fit the doorway height ' +
					'(lower=$portalFloorZ, upper=$portalTopZ)';
			// Balance the physical stock clearance below the TCP and wrist-package
			// clearance above it. This gives the articulated links room at both the
			// sill and header while the arm crosses the opening plane.
			var doorwayZ = Math.min(stagingZ, (portalFloorZ + portalTopZ) / 2);
			if (stagingZ + payloadFloorOffset < cell.opening.z + floorClearance ||
				doorwayZ + payloadFloorOffset < cell.opening.z + floorClearance)
				throw 'The wrist, gripper and carried blank do not fit through the doorway ' +
					'(opening=${cell.opening.z}, stockBottom=$payloadFloorZ, staging=$stagingZ, ' +
					'portal=$doorwayZ, floorClearance=$floorClearance, portalTop=$portalTopZ)';
			var firstTrayTargets:Array<{name:String, point:Array<Float>}> = [];
			var remainingTrayTargets:Array<{name:String, point:Array<Float>}> = [];
			// Cross at the door centre, then move sideways to the off-centre vise inside.
			var portalX = cell.opening.x;
			var insideDoor = [portalX, cell.opening.y + doorStandOff, doorwayZ];
			var viseApproach = [graspWorld.x + viseNormal.x * approach,
				graspWorld.y + viseNormal.y * approach, graspWorld.z + viseNormal.z * approach];
			var loadApproach = [loadWorld.x + viseNormal.x * approach,
				loadWorld.y + viseNormal.y * approach, loadWorld.z + viseNormal.z * approach];
			var viseTransitZ = Math.max(insideDoor[2], Math.max(viseApproach[2], loadApproach[2]));
			if (viseTransitZ > portalTopZ) throw "The vise transfer plane does not fit under the enclosure header";
			function doorToVise(t:Float):Array<Float> return [
				insideDoor[0] + (viseApproach[0] - insideDoor[0]) * t,
				insideDoor[1] + (viseApproach[1] - insideDoor[1]) * t,
				viseTransitZ
			];
			var viseTransfer = doorToVise(0.5);
			var viseTransferInner = doorToVise(0.75);
			for (tray in trayLayouts) for (slot in tray.slots) {
				var name = tray.id == "infeedTray" ? StringTools.replace(slot.name, "slot-", "infeed-")
					: StringTools.replace(slot.name, "slot-", "outfeed-");
				var graspedSlot = AssemblyFrames.compose(slot.frame, graspLocal);
				var normal = AssemblyFrames.transformVector(slot.frame, grasp.approachAxis.x,
					grasp.approachAxis.y, grasp.approachAxis.z);
				var targets = tray.id == "infeedTray" && slot.name == "slot-0-0" ? firstTrayTargets : remainingTrayTargets;
				targets.push({name: name + "-clearance", point: [graspedSlot.x + normal.x * trayClearanceMm,
					graspedSlot.y + normal.y * trayClearanceMm, graspedSlot.z + normal.z * trayClearanceMm]});
				targets.push({name: name + "-approach", point: [graspedSlot.x + normal.x * approach,
					graspedSlot.y + normal.y * approach, graspedSlot.z + normal.z * approach]});
				targets.push({name: name, point: [graspedSlot.x, graspedSlot.y, graspedSlot.z]});
			}
			// The shared door and vise solutions follow the first real mission route
			// so IK chooses their branch from the infeed grasp, not from the vise.
			var baseTargets = firstTrayTargets.concat([
				{name: "outside staging", point: [portalX, cell.opening.y - toolStandOff, stagingZ]},
				{name: "outside door", point: [portalX, cell.opening.y - doorStandOff, doorwayZ]},
				{name: "doorway center", point: [portalX, cell.opening.y, doorwayZ]},
				{name: "doorway-inner", point: [portalX, cell.opening.y + doorStandOff / 4, doorwayZ]},
				{name: "inside doorway", point: [portalX, cell.opening.y + doorStandOff / 2, doorwayZ]},
				{name: "inside door", point: insideDoor},
				{name: "vise-transfer", point: viseTransfer},
				{name: "vise-transfer-inner", point: viseTransferInner},
				{name: "vise-load-approach", point: loadApproach},
				{name: "vise-load", point: [loadWorld.x, loadWorld.y, loadWorld.z]},
				{name: "vise-approach", point: viseApproach},
				{name: "vise", point: [graspWorld.x, graspWorld.y, graspWorld.z]}
			]).concat(remainingTrayTargets);
			var options = new IkOptions(0.00001, 0.0001, 250, 0.001).reaching();
			var mountedTools = new AssemblyState(reachAssembly.definition());
			var tcpA = mountedTools.worldConnector("arm/tool/gripperA/body", "tcp");
			var tcpB = mountedTools.worldConnector("arm/tool/gripperB/body", "tcp");
			var bInAFrame = AssemblyFrames.compose(AssemblyFrames.inverse(tcpA), tcpB);
			var bInA = transform(bInAFrame);
			var requests:Array<TendingReachRequest> = [];
			for (tool in [DualArmGripperTool.TOOL_CHANNEL_A, DualArmGripperTool.TOOL_CHANNEL_B]) {
				var offset = tool == DualArmGripperTool.TOOL_CHANNEL_A ? Transform3.identity() : bInA;
				for (entry in baseTargets) requests.push({name: entry.name, tool: tool, point: entry.point, toolOffset: offset});
			}
			// This is the pedestal location saved in the assembled cell.
			// A contact and its straight insertion must fit at one tool roll. Search
			// mounting offsets from the actual dual-tool envelope when the nominal
			// base position leaves the wrist against the retracted head.
			var lateralOffsets = [0.0, -toolWidth / 2, toolWidth / 2, -toolWidth, toolWidth,
				-toolWidth / 4, toolWidth / 4, -3 * toolWidth / 4, 3 * toolWidth / 4];
			// The physical cell fixes this riser height. MotionKit certifies each
			// generated polynomial separately when it admits the mission route.
			var riserHeights = [550.0];
			var candidates:Array<{height:Float, baseXOffset:Float, baseYOffset:Float}> = [];
			for (height in riserHeights) for (baseXOffset in lateralOffsets)
				for (baseYOffset in [0.0, -toolDepth / 2, toolDepth / 2, -toolDepth, -2 * toolDepth])
				candidates.push({height: height, baseXOffset: baseXOffset, baseYOffset: baseYOffset});
			for (candidate in candidates) {
				var height = candidate.height, baseXOffset = candidate.baseXOffset, baseYOffset = candidate.baseYOffset;
				var armPlacement = AssemblyFrames.compose(AssemblyFrames.translation(source.reference.d[3] + baseXOffset,
					classBaseY + baseYOffset, height),
					AssemblyFrames.fromRotationMatrix(0, 0, 0, [0.0, 1, 0, -1, 0, 0, 0, 0, 1]));
				var baseFlange = mountedBaseFlange(source, armPlacement);
				// AssemblySimulationBridge's root link is the arm assembly origin;
				// the base-flange part is offset within it and is checked separately.
				var inverse = transform(armPlacement).inverse();
				var solved:Array<Array<Float>> = [], targets:Array<AssemblyFrame> = [], yawsSolved:Array<Float> = [], seed = source.ready();
				var margin = 1.0, wrist = 1.0, valid = true;
				var failedAt:Null<String> = null, failedReason:Null<String> = null;
				var poseResults:Array<TendingReachPose> = [];
				for (pointIndex in 0...requests.length) {
					var request = requests[pointIndex], point = request.point;
					var selected:Null<Array<Float>> = null, selectedTarget:Null<AssemblyFrame> = null;
					var selectedYaw = 0.0;
					var selectionFailure = "no tool roll candidate";
					var yaws = [0.0, Math.PI / 2, Math.PI, 3 * Math.PI / 2];
					for (yaw in yaws) {
						var pose = graspFrame(point, graspWorld, yaw);
						var desiredTool = new Transform3(new Vec3(point[0] / 1000, point[1] / 1000, point[2] / 1000),
							new Quat(pose.qx, pose.qy, pose.qz, pose.qw));
						var target = inverse.compose(desiredTool.compose(request.toolOffset.inverse()));
						var solution = arm.solve(target, seed, options);
						if (!solution.converged) { selectionFailure = 'IK residual ${solution.positionError}/${solution.orientationError}'; continue; }
						if (!withinMargins(arm, solution.q)) {
							selectionFailure = 'joint margin ${jointMargin(arm, solution.q)}, wrist ${Math.abs(Math.sin(solution.q[4]))}';
							continue;
						}
						var selfHit = clearanceViolation(clearance, solution.q);
						if (selfHit != null) { selectionFailure = 'self ${selfHit.a}/${selfHit.b} ${selfHit.distance} < ${selfHit.required}'; continue; }
						selected = solution.q; selectedTarget = pose; selectedYaw = yaw; break;
					}
					// Numeric IK can converge on a self-colliding branch even when another
					// shoulder/elbow/wrist branch reaches the same frame. Search those branches
					// before rejecting the machine layout.
					if (selected == null || selectedTarget == null) for (yaw in yaws) {
						var pose = graspFrame(point, graspWorld, yaw);
						var desiredTool = new Transform3(new Vec3(point[0] / 1000, point[1] / 1000, point[2] / 1000),
							new Quat(pose.qx, pose.qy, pose.qz, pose.qw));
						var target = inverse.compose(desiredTool.compose(request.toolOffset.inverse()));
						for (pan in [-Math.PI / 2, 0.0, Math.PI / 2, Math.PI]) {
							if (selected != null) break;
							for (elbow in [-Math.PI / 2, Math.PI / 2]) {
								if (selected != null) break;
								for (wristSeed in [-Math.PI / 2, Math.PI / 2]) {
									var attempt = arm.solve(target,
										[pan, -Math.PI / 2, elbow, -elbow, wristSeed, 0.0], options);
									if (attempt.converged && withinMargins(arm, attempt.q) && clearanceViolation(clearance, attempt.q) == null) {
										selected = attempt.q; selectedTarget = pose; selectedYaw = yaw; break;
									}
								}
							}
						}
					}
					if (selected == null || selectedTarget == null) {
						valid = false; failedAt = request.name + "@" + request.tool; failedReason = selectionFailure; break;
					}
					solved.push(selected); targets.push(selectedTarget); yawsSolved.push(selectedYaw);
					seed = selected.copy();
				}
				if (valid) {
					var bodies:Array<robotkit.manipulation.ClearanceBodyData> = [for (hull in converted.linkHulls)
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
					// Only the active tool-to-vise or selected-slot-blank pairs get the
					// reduced work-contact margin at their grasp pose. Every other cell
					// body, including the idle gripper and neighbouring blanks, keeps 5 mm.
					var viseContactBodies = [for (body in environment)
						if (StringTools.startsWith(body.name, "mill/vise/") || body.name == "mill/stock") "machine:" + body.name];
					var full = reachClearances(arm, bodies, parkPose, [], false, converted.jointInterfaces)[0];
					var readyHit = clearanceViolation(full, parkPose);
					full.space.dispose();
					if (readyHit != null) {
						failedAt = "ready";
						failedReason = 'clearance ${readyHit.a}/${readyHit.b} (${readyHit.distance} m)';
						valid = false;
					}
					var transitSpaces = new Map<String, ManipulatorPlannerSpace>();
					for (tool in [DualArmGripperTool.TOOL_CHANNEL_A, DualArmGripperTool.TOOL_CHANNEL_B]) {
						var toolBody = tool == DualArmGripperTool.TOOL_CHANNEL_A
							? "arm/" + DualArmGripperTool.GRIPPER_A_BODY : "arm/" + DualArmGripperTool.GRIPPER_B_BODY;
						var toolCarrier = converted.partLinks.get(toolBody);
						if (toolCarrier == null) throw "Reach transit has no tool carrier";
						var payloadFrame = AssemblyFrames.compose(toolCarrier.offset, tcpLocal);
						var payload:Array<Float> = [];
						for (vertex in 0...Std.int(stockHull.length / 3)) {
							var graspPoint = AssemblyFrames.transformPoint(graspInverse, stockHull[3 * vertex],
								stockHull[3 * vertex + 1], stockHull[3 * vertex + 2]);
							var point = AssemblyFrames.transformPoint(payloadFrame, graspPoint.x / 1000,
								graspPoint.y / 1000, graspPoint.z / 1000);
							payload.push(point.x); payload.push(point.y); payload.push(point.z);
						}
						var activePrefix = toolBody.substr(0, toolBody.length - "body".length);
						var transitBodies:Array<robotkit.manipulation.ClearanceBodyData> = [];
						for (index in 0...bodies.length) {
							var body = bodies[index];
							if (body.name == "machine:infeedTray/blank-0-0" || StringTools.startsWith(body.name, "machine:door-closed/")) continue;
							var vertices = index < loadedHulls.length && StringTools.startsWith(body.name, activePrefix)
								? loadedHulls[index].vertices : body.vertices;
							transitBodies.push({name: body.name, link: body.link, vertices: vertices, tool: body.tool});
						}
						var payloadBody:robotkit.manipulation.ClearanceBodyData = {name: "reach-payload",
							link: converted.model.links[toolCarrier.link].id, vertices: payload, tool: true};
						transitBodies.push(payloadBody);
						// A payload is not structural. Use its actual grasp posture so the
						// articulation's reference-overlap rules cannot hide a folded-arm contact.
						var graspIndex = -1;
						for (index in 0...requests.length)
							if (requests[index].tool == tool && requests[index].name == "infeed-0-0") graspIndex = index;
						if (graspIndex < 0) throw "Reach transit has no initial grasp posture";
						transitSpaces.set(tool, transitSpace(arm, transitBodies, solved[graspIndex], toolBody.substr(0, toolBody.length - "/body".length)));
					}
					for (i in 0...solved.length) {
						if (!valid) break;
						var request = requests[i];
						var activeToolPrefix = request.tool == DualArmGripperTool.TOOL_CHANNEL_A
							? "arm/tool/gripperA/" : "arm/tool/gripperB/";
						// Keep the idle gripper in the scene collision check. Only the selected
						// fingers and jaw are allowed the reduced margin at the workpiece.
						var exterior = StringTools.startsWith(request.name, "infeed-") ||
							StringTools.startsWith(request.name, "outfeed-") || request.name == "outside staging";
						var pointBodies:Array<robotkit.manipulation.ClearanceBodyData> = [];
						for (index in 0...bodies.length) {
							var body = bodies[index];
							if (!exterior && StringTools.startsWith(body.name, "machine:door-closed/")) continue;
							var active = StringTools.startsWith(body.name, activeToolPrefix);
							var vertices = index < loadedHulls.length && active ? loadedHulls[index].vertices : body.vertices;
							pointBodies.push({name: body.name, link: body.link, vertices: vertices, tool: active});
						}
						var selectedStock = trayStock(request.name);
						var contactBodies = viseContactBodies.copy();
						if (selectedStock != null) contactBodies.push("machine:" + selectedStock);
						var contact = request.name == "vise" || request.name == "vise-load" || selectedStock != null;
						var pointClearance = reachClearances(arm, pointBodies, parkPose, contactBodies, contact,
							converted.jointInterfaces);
						var paired = -1;
						if (StringTools.endsWith(request.name, "-approach")) {
							var station = request.name.substr(0, request.name.length - "-approach".length);
							for (index in 0...requests.length)
								if (requests[index].name == station && requests[index].tool == request.tool) paired = index;
						}
						var pairedClearance:Array<ReachClearanceCheck> = [];
						if (paired >= 0) {
							var pairedStock = trayStock(requests[paired].name), pairedBodies = viseContactBodies.copy();
							if (pairedStock != null) pairedBodies.push("machine:" + pairedStock);
							pairedClearance = reachClearances(arm, pointBodies, parkPose, pairedBodies,
								requests[paired].name == "vise" || requests[paired].name == "vise-load" || pairedStock != null, converted.jointInterfaces);
						}
						var hit = firstViolation(pointClearance, solved[i]);
						var point = request.point;
						var alternative:Null<Array<Float>> = null;
						var alternativeTarget:Null<AssemblyFrame> = null;
						var pairedSolution:Null<Array<Float>> = null, pairedTarget:Null<AssemblyFrame> = null;
						var pairFailures:Array<String> = [];
						// Numeric IK has several shoulder, elbow and wrist branches. A colliding
						// first solution does not mean the pose is unreachable. Roll about the
						// approach axis to keep the physical valve and hose clear as well.
						var yaws = [0.0, Math.PI / 2, Math.PI, 3 * Math.PI / 2];
						if (i > 0 && requests[i - 1].tool == request.tool) {
							var precedingYaw = yawsSolved[i - 1];
							yaws = [precedingYaw].concat([for (yaw in yaws) if (yaw != precedingYaw) yaw]);
						}
						var alternativeYaw = 0.0;
						// Preserve the preceding route branch before trying other IK branches.
						var routeSeeds:Array<Array<Float>> = [];
						if (i > 0 && requests[i - 1].tool == request.tool) routeSeeds.push(solved[i - 1]);
						routeSeeds.push(solved[i]);
						for (pan in [-Math.PI / 2, 0.0, Math.PI / 2, Math.PI])
							for (elbow in [-Math.PI / 2, Math.PI / 2])
								for (wristSeed in [-Math.PI / 2, Math.PI / 2])
									routeSeeds.push([pan, -Math.PI / 2, elbow, -elbow, wristSeed, 0.0]);
						for (yaw in yaws) {
							if (alternative != null) break;
							var reportedPairFailure = false;
							var pose = graspFrame(point, graspWorld, yaw);
							var desiredTool = new Transform3(new Vec3(point[0] / 1000, point[1] / 1000, point[2] / 1000),
								new Quat(pose.qx, pose.qy, pose.qz, pose.qw));
							var target = inverse.compose(desiredTool.compose(request.toolOffset.inverse()));
							for (routeSeed in routeSeeds) {
								if (alternative != null) break;
								var attempt = arm.solve(target, routeSeed, options);
								if (attempt.converged && withinMargins(arm, attempt.q) &&
									clearanceViolation(clearance, attempt.q) == null &&
									firstViolation(pointClearance, attempt.q) == null) {
									if (i > 0 && requests[i - 1].tool == request.tool && transitWaypoint(request.name)) {
										var space = transitSpaces.get(request.tool);
										if (space == null) throw "Reach transit collision space is missing";
										var problem = space.invalid(attempt.q);
										if (problem != null) {
											if (!reportedPairFailure) {
												pairFailures.push('roll=$yaw transit: ' + (problem == null ? "edge not certified" : problem));
												reportedPairFailure = true;
											}
											continue;
										}
									}
									if (paired >= 0) {
										var destination = requests[paired];
										var seated = graspFrame(destination.point, graspWorld, yaw);
										var desired = new Transform3(new Vec3(seated.x / 1000, seated.y / 1000, seated.z / 1000),
											new Quat(seated.qx, seated.qy, seated.qz, seated.qw));
										var insertion = arm.solve(inverse.compose(desired.compose(destination.toolOffset.inverse())), attempt.q, options);
										var pairSelf = insertion.converged ? clearanceViolation(clearance, insertion.q) : null;
										var pairHit = insertion.converged ? firstViolation(pairedClearance, insertion.q) : null;
										if (!insertion.converged || !withinMargins(arm, insertion.q) || pairSelf != null || pairHit != null) {
											if (!reportedPairFailure) {
												pairFailures.push('roll=$yaw: ' + (!insertion.converged ? 'IK ${insertion.positionError}/${insertion.orientationError}'
													: !withinMargins(arm, insertion.q) ? 'joint=${jointMargin(arm, insertion.q)} wrist=${Math.abs(Math.sin(insertion.q[4]))}'
													: pairSelf != null ? 'self ${pairSelf.a}/${pairSelf.b} ${pairSelf.distance}'
													: pairHit != null ? 'cell ${pairHit.a}/${pairHit.b} ${pairHit.distance}' : "unknown"));
												reportedPairFailure = true;
											}
											continue;
										}
										pairedSolution = insertion.q; pairedTarget = seated;
									}
									alternative = attempt.q;
									alternativeTarget = pose;
									alternativeYaw = yaw;
									break;
								}
							}
						}
						disposeClearances(pointClearance);
						disposeClearances(pairedClearance);
						if (alternative != null && alternativeTarget != null) {
							solved[i] = alternative; targets[i] = alternativeTarget;
							yawsSolved[i] = alternativeYaw;
							if (paired >= 0 && pairedSolution != null && pairedTarget != null) {
								solved[paired] = pairedSolution; targets[paired] = pairedTarget;
								yawsSolved[paired] = alternativeYaw;
							}
						} else {
							failedAt = request.name + "@" + request.tool;
							failedReason = paired >= 0 ? "no clearance-safe approach/contact pair with a common tool roll: " + pairFailures.join("; ")
								: pairFailures.length > 0 ? pairFailures.join("; ")
								: hit == null ? "no clearance-safe pose"
								: 'clearance ${hit.a}/${hit.b} (${hit.distance} m)';
							valid = false; break;
						}
					}
					if (valid) for (index in 1...requests.length) {
						var request = requests[index];
						var preceding = transitPredecessor(request.name);
						if (preceding == null) continue;
						var previous = -1;
						for (candidate in 0...requests.length)
							if (requests[candidate].name == preceding && requests[candidate].tool == request.tool) previous = candidate;
						if (previous < 0) throw 'Reach transit has no predecessor "$preceding"';
						var space = transitSpaces.get(request.tool);
						if (space == null) throw "Reach transit collision space is missing";
						var from = solved[previous];
						var to = solved[index];
						var problem = space.invalid(to);
						var path = problem == null ? new RrtConnect(space, 17).plan(from, to) : null;
						if (problem != null || path == null || !path.found()) {
							valid = false; failedAt = request.name + "@" + request.tool;
							failedReason = problem == null ? path == null ? "transit path missing" : path.failure : problem;
							break;
						}
					}
					for (space in transitSpaces) space.dispose();
					if (valid) {
						clearance.space.dispose();
						margin = 1; wrist = 1;
						for (q in solved) {
							margin = Math.min(margin, jointMargin(arm, q));
							wrist = Math.min(wrist, Math.abs(Math.sin(q[4])));
						}
						poseResults = [for (index in 0...requests.length) {
							var request = requests[index];
							{name: request.name, tool: request.tool, position: solved[index], target: targets[index], toolYaw: yawsSolved[index]};
						}];
						return {cls: cls, riser: height, armPlacement: armPlacement, armBaseFlange: baseFlange,
							jointMargin: margin, wristSine: wrist,
							parkPose: parkPose.copy(), trayClearanceMm: trayClearanceMm,
							poses: poseResults, trays: trayLayouts,
							handlingApproachMm: approach,
							tableFrame: tableFrame, tableOffset: tableOffset, tableWidth: tableWidth,
							tableDepth: tableDepth, tableTop: tableTop};
					}
				}
				if (!valid && failedAt != null) {
					var failure = failedAt == "ready"
						? '$height mm, base X ${baseXOffset} mm / Y ${baseYOffset} mm at ready pose: $failedReason'
						: '$height mm, base X ${baseXOffset} mm / Y ${baseYOffset} mm at $failedAt: $failedReason';
					if (firstClassFailure == null) firstClassFailure = failure;
					lastClassFailure = failure;
					if (!firstFailureByHeight.exists(Std.int(height)))
						firstFailureByHeight.set(Std.int(height), failure);
					var xKey = '$height:$baseXOffset';
					if (height == 550 && !reportedBaseX.exists(xKey)) {
						reportedBaseX.set(xKey, true);
						riserFailures.push(failure);
					}
				}
			}
			clearance.space.dispose();
		if (firstClassFailure != null && lastClassFailure != null)
			failures.push('$cls first ${firstClassFailure}, 550 mm candidates [${riserFailures.join("; ")}], ' +
				'first failures by riser [${[for (height in riserHeights) if (firstFailureByHeight.exists(Std.int(height)))
					firstFailureByHeight.get(Std.int(height))].join("; ")}], last ${lastClassFailure}');
		}
		throw 'The selected Reach1300/550 mm cell layout does not reach every required pose with the joint and wrist margins' +
			(failures.length == 0 ? "" : '; candidates: ${failures.join("; ")}');
	}

	/** Ordered transport stations shared by the source study and the saved mission. */
	public static function viseTransitStations(station:String = "vise"):Array<String>
		return ["outside staging", "outside door", "doorway center", "doorway-inner", "inside doorway",
			"inside door", "vise-transfer", "vise-transfer-inner", station + "-approach"];

	static function transitPredecessor(name:String):Null<String> {
		for (station in ["vise", "vise-load"]) {
			var path = viseTransitStations(station), index = path.indexOf(name);
			if (index > 0) return path[index - 1];
		}
		return null;
	}

	static function transitWaypoint(name:String):Bool return transitPredecessor(name) != null;

	/** Conservatively cover every jaw position from open to contact with this blank.
	 * The study freezes process DOFs, so their real closing motion must be in the hull.
	 */
	static function sweepGripperClosure(converted:cadbridge.AssemblySimulationBridge.AssemblySimulationModel,
		scene:materia.project.SceneArtifact.SceneArtifactData, spec:machinekit.robotics.ParallelGripperSpec):Array<cadbridge.AssemblySimulationBridge.AssemblyLinkHull> {
		var definition = scene.assemblyDefinition;
		if (definition == null) throw "Gripper envelope needs the reach assembly";
		var open = new AssemblyState(definition), closed = new AssemblyState(definition);
		var flat = AssemblyDefinitionFlattener.flatten(definition);
		var travel = (spec.maximumPartWidth() - spec.expectedPartWidth) / 2;
		for (joint in flat.joints) {
			if (StringTools.endsWith(joint.id, "/jaw-left-slide")) closed.setJoint(joint.id, travel);
		}
		var result = [for (hull in converted.linkHulls) {link: hull.link, part: hull.part, vertices: hull.vertices.copy()}];
		for (hull in result) {
			var delta = AssemblyFrames.compose(AssemblyFrames.inverse(open.worldPose(hull.part)), closed.worldPose(hull.part));
			if (Math.abs(delta.x) + Math.abs(delta.y) + Math.abs(delta.z) < 1e-9) continue;
			var carrier = converted.partLinks.get(hull.part);
			if (carrier == null) throw "Gripper envelope lost its physical carrier";
			var metric:AssemblyFrame = {x: delta.x * scene.metresPerUnit, y: delta.y * scene.metresPerUnit,
				z: delta.z * scene.metresPerUnit, qx: delta.qx, qy: delta.qy, qz: delta.qz, qw: delta.qw};
			var shift = AssemblyFrames.compose(carrier.offset, AssemblyFrames.compose(metric, AssemblyFrames.inverse(carrier.offset)));
			var original = hull.vertices.copy();
			for (i in 0...Std.int(original.length / 3)) {
				var point = AssemblyFrames.transformPoint(shift, original[3 * i], original[3 * i + 1], original[3 * i + 2]);
				hull.vertices.push(point.x); hull.vertices.push(point.y); hull.vertices.push(point.z);
			}
		}
		return result;
	}

	static function transitSpace(arm:Manipulator, data:Array<robotkit.manipulation.ClearanceBodyData>,
			reference:Array<Float>, gripperPrefix:String):ManipulatorPlannerSpace {
		var description = new CollisionDescription();
		var options = new RobotCollisionOptions("reach-arm");
		options.reference = arm.stateOf(reference).q;
		options.referenceName = "infeed grasp";
		var described = RobotCollision.describe(description, arm.robot, options);
		var basePose = description.bodies[described.linkBody(arm.robot.links[0].id)].pose;
		var moving:Array<robotkit.manipulation.ClearanceBodyData> = [];
		for (body in data) {
			if (StringTools.startsWith(body.name, "machine:"))
				description.addFixed(body.name, basePose, CollisionGeometry.Convex(body.vertices),
					options.linkGroup, 0.0, "cell hull");
			else moving.push({name: body.name, link: body.link, vertices: body.vertices, tool: false});
		}
		var hulls = RobotCollision.describeClearanceBodies(description, described, moving);
		for (index in 0...moving.length) if (moving[index].name == "reach-payload") {
			var payload = hulls[index];
			for (other in 0...description.objects.length) {
				if (other == payload || StringTools.startsWith(description.objects[other].name,
					described.bodies.name + "/" + gripperPrefix + "/")) continue;
				// A loose workpiece never acquires structural rest-overlap permissions.
				description.setObjectRule(payload, other, CollisionPairRule.Check, CollisionPairStatus.Checked);
			}
		}
		var build = description.build(new NativeCollisionWorld());
		return new ManipulatorPlannerSpace(new CollisionPlannerSpace(description, build, described.bodies,
			CollisionMargins.uniform(CollisionClearance.MARGIN)), arm, reference);
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
	static function graspFrame(point:Array<Float>, grasp:AssemblyFrame, yaw:Float):AssemblyFrame {
		var roll = Quat.fromAxisAngle(new Vec3(0, 0, 1), yaw);
		var rotation = roll.multiply(new Quat(grasp.qx, grasp.qy, grasp.qz, grasp.qw));
		return {x: point[0], y: point[1], z: point[2], qx: rotation.x, qy: rotation.y,
			qz: rotation.z, qw: rotation.w};
	}
	/** Resolve the base-flange part frame created by the cell's fixed riser mate. */
	static function mountedBaseFlange(arm:CobotArm, placement:AssemblyFrame):AssemblyFrame {
		var localPoses = arm.solvedPoses();
		var localBase = localPoses.get("baseFlange");
		if (localBase == null) throw "Cobot arm has no solved base flange";
		return AssemblyFrames.compose(placement, localBase);
	}
	static function transform(frame:AssemblyFrame):Transform3 return new Transform3(
		new Vec3(frame.x / 1000, frame.y / 1000, frame.z / 1000), new Quat(frame.qx, frame.qy, frame.qz, frame.qw));
}

function main():Void TendingReach.run();
