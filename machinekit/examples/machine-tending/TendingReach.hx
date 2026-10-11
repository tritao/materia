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
import motionkit.robot.JointMotionPolicy;
import motionkit.robot.JointPathTiming;
import motionkit.robot.HandlingPlanRunner;
import motionkit.robot.StructuredJointPathPlanner;
import motionkit.robot.ManipulatorKinematics;
import motionkit.kinematics.PathRequest;
import motionkit.path.PosePath;
import motionkit.path.PoseLine;
import motionkit.path.PoseWaypoint;
import motionkit.path.PoseSampling;
import motionkit.path.OrientationPolicy;
import motionkit.planner.RrtConnect;
import TendingCyclePlan.TendingTransferDoor;
import TendingCyclePlan.TendingTransferRequest;
import TendingCyclePlan.TendingTransferRole;
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
import robotkit.model.JointType;
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
	/** Complete certified joint paths; live planning checks every edge again. */
	var transfers:Array<TendingReachTransfer>;
	/** The actual cell-fixed pocket trays and their seat-floor poses, all in millimetres. */
	var trays:Array<TendingTrayLayout>;
	var tableFrame:AssemblyFrame;
	var tableOffset:Float;
	var tableWidth:Float;
	var tableDepth:Float;
	var tableTop:Float;
}
typedef TendingReachTransfer = {
	var request:TendingTransferRequest;
	var positions:Array<Array<Float>>;
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
	var pitchX:Float;
	var pitchY:Float;
	var width:Float;
	var depth:Float;
}
private typedef ReachClearanceCheck = {
	var space:ManipulatorPlannerSpace;
	var description:CollisionDescription;
	var build:collisionkit.CollisionBuild;
}
private typedef ReachCandidate = {
	var approach:TendingReachPose;
	var contact:Null<TendingReachPose>;
	var staging:Null<TendingReachPose>;
	var clearance:Null<TendingReachPose>;
	var descent:Null<Array<Array<Float>>>;
	/** The entry actually certified and timed during station selection. */
	var entry:Null<Array<Array<Float>>>;
	var exchanges:Array<TendingReachTransfer>;
	var seconds:Float;
}
private typedef TrayHandlingProjection = {var radius:Float; var minimumZ:Float;}
private typedef ReachPayload = {
	var body:robotkit.manipulation.ClearanceBodyData;
	var holder:String;
}
private typedef ReachEnvironmentHull = {var name:String; var vertices:Array<Float>;}
private typedef ReachHoldingPath = {var position:Array<Float>; var entry:Array<Array<Float>>; var exterior:Array<Array<Float>>;}

/** One reversible route must be clear in each physical state in which it is used. */
private class ReachPlannerStates implements motionkit.planner.PlannerSpace {
	final states:Array<ManipulatorPlannerSpace>;
	public function new(states:Array<ManipulatorPlannerSpace>) {
		if (states.length == 0) throw "A reach route needs its physical states";
		this.states = states;
		var first = states[0], low = first.lower(), high = first.upper();
		for (state in states) {
			if (state.dimension() != first.dimension()) throw "Reach states have different joint dimensions";
			var otherLow = state.lower(), otherHigh = state.upper();
			for (joint in 0...low.length) if (low[joint] != otherLow[joint] || high[joint] != otherHigh[joint])
				throw "Reach states have different joint bounds";
		}
	}
	public function dimension():Int return states[0].dimension();
	public function lower():Array<Float> return states[0].lower();
	public function upper():Array<Float> return states[0].upper();
	public function invalid(q:Array<Float>):Null<String> {
		for (state in states) { var failure = state.invalid(q); if (failure != null) return failure; }
		return null;
	}
	public function edgesClear(edges:Array<Float>, count:Int):Array<Bool> {
		var clear = [for (_ in 0...count) true];
		for (state in states) {
			var checked = state.edgesClear(edges, count);
			for (index in 0...count) clear[index] = clear[index] && checked[index];
		}
		return clear;
	}
	public function dispose():Void for (state in states) state.dispose();
}

/** Static reach, with each physical gripper TCP at the vise, tray slots and both sides of the
 * doorway. Mill and blank geometry decide the poses; each gripper supplies its own envelope.
 * MotionKit certifies the actual timed joint paths when it admits the generated moveVia plans.
 */
class TendingReach {
	static inline var MIN_JOINT_MARGIN:Float = 0.1;
	// Improvements use a smaller search budget after a feasible incumbent is certified.
	static inline var APPROACH_SEARCH_ITERATIONS:Int = 256;
	/** Assumed fixture placement allowance on each side, beyond collision clearance. */
	public static inline var TRAY_PLACEMENT_ALLOWANCE_MM:Float = 1.0;
	/** Assumed placement allowance beyond the general collision margin for fixed sensor heads. */
	public static inline var PRESENCE_HEAD_MOUNT_ALLOWANCE_MM:Float = 1.0;
	/** Assumed 6 x 40 mm replaceable pads leave handling clearance around the 60 mm stock face. */
	public static function tendingTool():DualArmGripperTool
		return new DualArmGripperTool(32, BenchMill.STOCK_WIDTH, 60, 16, false,
			BenchMill.STOCK_DEPTH - 4 * CollisionClearance.MARGIN * 1000,
			6, 10);

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
		var production = TendingCyclePlan.bearingBlocks();
		var result = study(cell, production);
		for (needed in production.transferRequests()) {
			var path = transferPositions(result, needed.station, needed.tool, needed.fromTool);
			if (path.length < 2) throw 'Tending production has no complete path for ${needed.station}';
		}
		if (result.cls != Reach1300 || result.riser != 550 || result.poses.length != 96 ||
			result.jointMargin < 0.1 || result.wristSine < 0.2)
			throw 'Tending reach changed class, riser, pose coverage or margins: ' +
				'${result.cls}/${result.riser}/${result.poses.length}, joint=${result.jointMargin}, wrist=${result.wristSine}';
		var viseA = toolPose(result, "vise", DualArmGripperTool.TOOL_CHANNEL_A);
		var viseB = toolPose(result, "vise", DualArmGripperTool.TOOL_CHANNEL_B);
		var infeed = [for (tray in result.trays) if (tray.id == "infeedTray") tray][0];
		if (infeed == null || infeed.slots.length != 6 ||
			infeed.slots[1].frame.x - infeed.slots[0].frame.x <= BenchMill.STOCK_WIDTH ||
			infeed.slots[3].frame.y - infeed.slots[0].frame.y <= BenchMill.STOCK_DEPTH)
			throw "Reach tray slots overlap their stock footprints";
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
			'${result.jointMargin * 100}% joint margin, wrist |sin(q5)| ${result.wristSine}, ' +
			'tray pitch ${infeed.pitchX} x ${infeed.pitchY} mm');
	}

	public static function toolPose(result:TendingReachResult, name:String, channel:String):Array<Float> {
		for (pose in result.poses) if (pose.name == name && pose.tool == channel) return pose.position;
		throw 'Tending reach has no checked pose "$name" for "$channel"';
	}

	public static function toolYaw(result:TendingReachResult, name:String, channel:String):Float {
		for (pose in result.poses) if (pose.name == name && pose.tool == channel) return pose.toolYaw;
		throw 'Tending reach has no checked orientation "$name" for "$channel"';
	}

	public static function transferPositions(result:TendingReachResult, station:String, tool:String, fromTool:String):Array<Array<Float>> {
		for (path in result.transfers) if (path.request.station == station && path.request.tool == tool && path.request.fromTool == fromTool)
			return [for (position in path.positions) position.copy()];
		throw 'No certified tending transfer for $station with $tool';
	}

	/** Project the real tool at each grasp. Only parts near the pocket hardware
	 * constrain pitch; their complete projection conservatively covers the jaw sweep.
	 * A radial bound permits any roll without reaching a neighbouring blank or sensor.
	 */
	static function traySpacing(scene:materia.project.SceneArtifact.SceneArtifactData, physical:cadbridge.AssemblySimulationBridge.AssemblyPhysicalData,
		spec:machinekit.robotics.ParallelGripperSpec,
		grasp:AssemblyFrame):{x:Float, y:Float} {
		var definition = scene.assemblyDefinition;
		if (definition == null) throw "Tray sizing needs the tool's assembly definition";
		var flat = AssemblyDefinitionFlattener.flatten(definition);
		var state = new AssemblyState(definition);
		var allowance = CollisionClearance.MARGIN * 1000 + TRAY_PLACEMENT_ALLOWANCE_MM;
		var hardwareTop = Math.max(BenchMill.STOCK_HEIGHT, 10 + DigitalSensor.BODY_HEIGHT) + allowance;
		var radius = 0.0;
		for (gripper in ["arm/tool/gripperA", "arm/tool/gripperB"]) {
			var projections = new Map<String, TrayHandlingProjection>();
			for (closed in [false, true]) {
				var travel = closed ? (spec.maximumPartWidth() - spec.expectedPartWidth) / 2 : 0.0;
				for (jaw in ["arm/tool/gripperA", "arm/tool/gripperB"]) state.setJoint(jaw + "/jaw-left-slide", travel);
				var placement = AssemblyFrames.compose(grasp,
					AssemblyFrames.inverse(state.worldConnector(gripper + "/body", "tcp")));
				for (occurrence in flat.occurrences) if (StringTools.startsWith(occurrence.id, "arm/tool/")) {
					var part = [for (part in physical.parts) if (part.id == occurrence.definition) part][0];
					var hull = part.collisionHull;
					if (hull == null) throw 'Tray sizing lost the hull for ${occurrence.id}';
					var pose = AssemblyFrames.compose(placement, state.worldPose(occurrence.id));
					var minimumZ = Math.POSITIVE_INFINITY, projected = 0.0;
					for (vertex in 0...Std.int(hull.length / 3)) {
						var point = AssemblyFrames.transformPoint(pose, hull[3 * vertex], hull[3 * vertex + 1], hull[3 * vertex + 2]);
						minimumZ = Math.min(minimumZ, point.z);
						projected = Math.max(projected, Math.sqrt(point.x * point.x + point.y * point.y));
					}
					var previous = projections.get(occurrence.id);
					projections.set(occurrence.id, {radius: previous == null ? projected : Math.max(previous.radius, projected),
						minimumZ: previous == null ? minimumZ : Math.min(previous.minimumZ, minimumZ)});
				}
			}
			// The jaws translate affinely. Endpoint bounds enclose their whole
			// sweep, including a part that crosses the hardware-height plane.
			for (projection in projections)
				if (projection.minimumZ <= hardwareTop) radius = Math.max(radius, projection.radius);
		}
		if (!Math.isFinite(radius) || radius <= 0) throw "Tray sizing has no finite handling envelope";
		var neighbourX = Math.max(BenchMill.STOCK_WIDTH / 2,
			Math.abs(presenceHeadOffsetX()) + DigitalSensor.BODY_WIDTH / 2);
		var neighbourY = Math.max(BenchMill.STOCK_DEPTH / 2,
			presenceHeadOffsetY() + DigitalSensor.BODY_DEPTH / 2);
		return {x: Math.max(BenchMill.STOCK_WIDTH + PartTray.PAD_EDGE_WIDTH, radius + neighbourX + allowance),
			y: Math.max(BenchMill.STOCK_DEPTH + PartTray.PAD_EDGE_DEPTH, radius + neighbourY + allowance)};
	}

	static function trayStock(station:String):Null<String> {
		if (StringTools.endsWith(station, "-approach") || StringTools.endsWith(station, "-clearance")) return null;
		if (StringTools.startsWith(station, "infeed-"))
			return "infeedTray/blank-" + station.substr("infeed-".length);
		return null;
	}

	/** Main's description owns fixed geometry, articulation and pair-specific contact rules. */
	static function reachClearances(arm:Manipulator, data:Array<robotkit.manipulation.ClearanceBodyData>,
		reference:Array<Float>, contactNames:Array<String>, contact:Bool,
		interfaces:Array<cadbridge.AssemblySimulationBridge.AssemblyJointInterface>,
		?payloads:Array<ReachPayload>):Array<ReachClearanceCheck> {
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
		if (payloads != null) for (payload in payloads) {
			var pieces = objects.get(payload.body.name);
			if (pieces == null) throw "Reach payload has no described collision object";
			for (piece in pieces) for (other in 0...description.objects.length) {
				if (piece == other) continue;
				var target = description.objects[other];
				var holderContact = StringTools.startsWith(target.name, payload.holder + "/");
				var workContact = payload.body.tool && contact && contactNames.indexOf(target.name) >= 0;
				margins.set(description.bodies[description.objects[piece].body].group,
					description.bodies[target.body].group, holderContact || workContact ? 0.0 : CollisionClearance.MARGIN);
				// Loose stock never inherits a structural reference-overlap exclusion.
				description.setObjectRule(piece, other, holderContact || workContact ? CollisionPairRule.Touch : CollisionPairRule.Check,
					CollisionPairStatus.Checked);
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

	public static function study(cell:EnclosedBenchMill, production:TendingCyclePlan):TendingReachResult {
		// Reach uses the same physical blank hull as machining, but the tending cell starts with an
		// empty vise. Study a loaded reference machine so the stock remains geometry input only.
		if (!cell.includesWorkpiece) return study(new EnclosedBenchMill(), production);
		var transferRequests = production.transferRequests();
		var trayEntries = new Map<String, TendingTransferRequest>();
		for (needed in transferRequests) if (needed.role == Inventory)
			trayEntries.set(needed.station + "-approach@" + needed.tool, needed);
		var failures:Array<String> = [];
		var firstSeat = production.rowOf(production.firstSlot) + "-" + production.columnOf(production.firstSlot);
		var firstStation = "infeed-" + firstSeat;
		var armTool = tendingTool();
		var gripper = armTool.spec;
		var source = new CobotArm(Reach1300, armTool);
		var millScene = SceneArtifact.decode(SceneArtifact.encode(AssemblyPreview.scene(cell, "reach-mill")));
		var millPhysical = AssemblyPhysicalPartView.fromSceneArtifact(millScene);
		var flat = AssemblyDefinitionFlattener.flatten(millScene.assemblyDefinition);
		var toolWidth = armTool.envelopeWidthMm;
		var toolDepth = armTool.envelopeDepthMm;
		var toolLength = gripper.bodyLength + gripper.fingerLength + armTool.adapterThicknessMm - 12;
		var state = cell.state();
		for (i in 0...3) state.setJoint("mill/" + cell.mill.specs[i].id, cell.loadPosition[i]);
		state.setJoint("door", cell.doorStroke);
		// Loading and unloading occur with the jaw retracted. The loaded reference
		// assembly starts clamped; using that pose makes the load clearance penetrate it.
		state.setJoint("mill/vise/jaw", 0);
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
		var graspYaws:Array<Float> = [];
		for (yaw in [0.0, Math.PI / 2, Math.PI, 3 * Math.PI / 2]) {
			var rolled = graspFrame([graspWorld.x, graspWorld.y, graspWorld.z], graspWorld, yaw);
			var inStock = AssemblyFrames.inverse(AssemblyFrames.compose(AssemblyFrames.inverse(blank), rolled));
			var low = Math.POSITIVE_INFINITY, high = Math.NEGATIVE_INFINITY;
			for (index in 0...Std.int(stockHull.length / 3)) {
				var point = AssemblyFrames.transformPoint(inStock, stockHull[3 * index], stockHull[3 * index + 1], stockHull[3 * index + 2]);
				low = Math.min(low, point.x); high = Math.max(high, point.x);
			}
			var width = high - low;
			if (width >= gripper.minimumPartWidth() && width <= gripper.maximumPartWidth()) graspYaws.push(yaw);
		}
		if (graspYaws.length == 0) throw "Reach grasp has no roll supported by the gripper width range";
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
		var environment:Array<ReachEnvironmentHull> = [for (occurrence in flat.occurrences) if (occurrence.id != "mill/stock") {
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
		var doorSweeps:Array<ReachEnvironmentHull> = [];
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
			var openHull = [for (body in environment) if (body.name == occurrence.id) body][0];
			// A prismatic door translates each convex part without rotation. Its
			// endpoint hull union is the complete sweep, not just two pose checks.
			doorSweeps.push({name: "door-sweep/" + occurrence.id, vertices: openHull.vertices.concat(vertices)});
		}
		state.setJoint("door", cell.doorStroke);
		// Build and check the real pocket geometry. The old tray envelope sat 10 mm
		// below the blank and missed the collision body the arm would actually meet.
		// Free blanks change pockets and follow the robot during a cycle; the mission
		// planner checks their live poses rather than baking one occupancy into this study.
		var reachAssembly = new MachineAssembly();
		reachAssembly.include("arm", source);
		reachAssembly.addComponent("airSupply", new AirSupply(), AssemblyFrames.translation(1200, -500, 0));
		reachAssembly.connectPorts("arm-air", "airSupply", "air", "arm", "compressedAir");
		if (reachAssembly.validate().length != 0) throw "Reach arm pneumatic service does not validate";
		var armScene = SceneArtifact.decode(SceneArtifact.encode(AssemblyPreview.scene(reachAssembly, "reach-arm")));
		var armPhysical = AssemblyPhysicalPartView.fromSceneArtifact(armScene);
		var spacing = traySpacing(armScene, armPhysical, gripper, graspLocal);
		var trayAssembly = new PartTray(production.rows, production.columns, BenchMill.STOCK_WIDTH, BenchMill.STOCK_DEPTH,
			spacing.x, spacing.y);
		// The two tray banks are fixed to the tending cell, independent of tool or
		// arm selection. Their positions are derived from the cell opening and the
		// real blank-seat height. These same layouts become assembled cell parts.
		var trayWidth = trayAssembly.width, trayDepth = trayAssembly.depth;
		var tableWidth = trayWidth + 40;
		var tableDepth = trayDepth + 40;
		var trayOffset = cell.openingWidth / 2 + trayWidth / 2 + 20;
		var trayX = cell.opening.x;
		var fixtureStart = environment.length;
		// Tray seats support the blank's bottom datum at the same height as the vise parallels.
		var traySeatZ = blank.z;
		var trayLayouts:Array<TendingTrayLayout> = [];
		for (side in [-1, 1]) {
			var centre = trayX + side * trayOffset;
			var id = side < 0 ? "infeedTray" : "outfeedTray";
			var frame = AssemblyFrames.translation(centre, 0, traySeatZ - trayAssembly.seat(0, 0).z);
			var slots:Array<TendingTraySlot> = [for (slot in trayAssembly.seatFrames())
				{name: slot.name, frame: AssemblyFrames.compose(frame, slot.frame)}];
			trayLayouts.push({id: id, frame: frame, slots: slots,
				pitchX: trayAssembly.pitchX, pitchY: trayAssembly.pitchY,
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
		var tableFrame = AssemblyFrames.translation(cell.opening.x, 0, 0);
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
		var fixtureBack = Math.NEGATIVE_INFINITY, fixtureLow = Math.POSITIVE_INFINITY, fixtureHigh = Math.NEGATIVE_INFINITY;
		for (index in fixtureStart...environment.length) {
			var vertices = environment[index].vertices;
			for (point in 0...Std.int(vertices.length / 3)) {
				fixtureBack = Math.max(fixtureBack, vertices[3 * point + 1]);
				fixtureLow = Math.min(fixtureLow, vertices[3 * point + 2]);
				fixtureHigh = Math.max(fixtureHigh, vertices[3 * point + 2]);
			}
		}
		var doorFront = Math.POSITIVE_INFINITY;
		for (body in doorSweeps) {
			var low = Math.POSITIVE_INFINITY, high = Math.NEGATIVE_INFINITY, front = Math.POSITIVE_INFINITY;
			for (point in 0...Std.int(body.vertices.length / 3)) {
				front = Math.min(front, body.vertices[3 * point + 1]);
				low = Math.min(low, body.vertices[3 * point + 2]);
				high = Math.max(high, body.vertices[3 * point + 2]);
			}
			if (low <= fixtureHigh + CollisionClearance.MARGIN && high >= fixtureLow - CollisionClearance.MARGIN)
				doorFront = Math.min(doorFront, front);
		}
		if (!Math.isFinite(doorFront + fixtureBack + fixtureLow + fixtureHigh))
			throw "Tray placement needs finite fixture and overlapping door sweep bounds";
		var trayY = (doorFront - fixtureBack - CollisionClearance.MARGIN) * 1000 - TRAY_PLACEMENT_ALLOWANCE_MM;
		for (index in fixtureStart...environment.length) {
			var vertices = environment[index].vertices;
			for (point in 0...Std.int(vertices.length / 3)) vertices[3 * point + 1] += trayY / 1000;
		}
		for (tray in trayLayouts) {
			tray.frame.y += trayY;
			for (slot in tray.slots) slot.frame.y += trayY;
		}
		tableFrame.y = trayY;
		checkFixtureDoorSweep(doorSweeps, environment.slice(fixtureStart));
	// The collision-screened design is the 1.3 m cobot on the cell's 550 mm riser.
	// `study` certifies its actual gripper and every assembled cell pose; the design
	// search is not repeated during each project load.
	for (cls in [Reach1300]) {
			var firstClassFailure:Null<String> = null, lastClassFailure:Null<String> = null;
			var riserFailures:Array<String> = [];
			var firstFailureByHeight = new Map<Int, String>();
			var reportedBaseX = new Map<String, Bool>();
			var parkPose = source.ready();
			// The base joint turns the folded ready posture out of the enclosure. Keeping
			// this cell-specific park explicit avoids occupying the spindle's travel lane.
			parkPose[0] = Math.PI;
			if (graspWorld.x - toolWidth / 2 < cell.opening.x - cell.openingWidth / 2 ||
				graspWorld.x + toolWidth / 2 > cell.opening.x + cell.openingWidth / 2)
				throw "Reference gripper does not fit the actual doorway";
			var scene = armScene;
			var converted = AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition, armPhysical);
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
			var exchangePayloads:Array<ReachPayload> = [];
			for (body in [DualArmGripperTool.GRIPPER_A_BODY, DualArmGripperTool.GRIPPER_B_BODY]) {
				var occurrence = "arm/" + body;
				var holder = converted.partLinks.get(occurrence);
				if (holder == null) throw "Exchange envelope lost its physical gripper carrier";
				var payloadFrame = AssemblyFrames.compose(holder.offset, tcpLocal);
				var vertices:Array<Float> = [];
				for (vertex in 0...Std.int(stockHull.length / 3)) {
					var local = AssemblyFrames.transformPoint(graspInverse, stockHull[3 * vertex],
						stockHull[3 * vertex + 1], stockHull[3 * vertex + 2]);
					var point = AssemblyFrames.transformPoint(payloadFrame, local.x / 1000, local.y / 1000, local.z / 1000);
					vertices.push(point.x); vertices.push(point.y); vertices.push(point.z);
				}
				exchangePayloads.push({body: {name: "payload:" + occurrence, link: converted.model.links[holder.link].id,
					vertices: vertices, tool: false}, holder: occurrence.substr(0, occurrence.length - "/body".length)});
			}
			var rootLink = converted.model.links[baseCarrier.link].id;
			var arm = new Manipulator(converted.model, rootLink, frame.id);
			var policy = JointMotionPolicy.mission(), motionJoints = arm.jointIndices();
			var timing = new JointPathTiming([for (index in 0...motionJoints.length)
				policy.rates(arm.robot.joints[motionJoints[index]].type, arm.group.limitsOf(index))]);
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
				var targets = tray.id == "infeedTray" && slot.name == "slot-" + firstSeat ? firstTrayTargets : remainingTrayTargets;
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
			// Exact contact interfaces need IK accuracy below their numerical tolerance.
			var options = new IkOptions(1e-10, 1e-10, 250, 0.001).reaching();
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
			// Both tools' staging choices must be final before a tray branch is
			// selected from the posture that actually precedes it in production.
			var selectionOrder = [for (index in 0...requests.length) if (!trayPose(requests[index].name)) index]
				.concat([for (index in 0...requests.length) if (trayPose(requests[index].name) && StringTools.endsWith(requests[index].name, "-approach")) index])
				.concat([for (index in 0...requests.length) if (trayPose(requests[index].name) && !StringTools.endsWith(requests[index].name, "-approach")) index]);
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
					var yaws = graspYaws.copy();
					for (yaw in yaws) {
						var pose = graspFrame(point, graspWorld, yaw);
						var desiredTool = new Transform3(new Vec3(point[0] / 1000, point[1] / 1000, point[2] / 1000),
							new Quat(pose.qx, pose.qy, pose.qz, pose.qw));
						var target = inverse.compose(desiredTool.compose(request.toolOffset.inverse()));
						var solution = arm.solve(target, seed, options);
						preferNearbyTurns(arm, solution.q, seed);
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
									preferNearbyTurns(arm, attempt.q, seed);
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
					for (body in environmentBodies(environment, inverse, rootLink)) bodies.push(body);
					var sweptDoorBodies = environmentBodies(doorSweeps, inverse, rootLink);
					// After loading, the new blank stays at the parallels while the
					// inactive tool carries the finish. A moving payload cannot stand in
					// for that independent seated obstacle during the retreat.
					var seatedVertices:Array<Float> = [];
					for (vertex in 0...Std.int(stockHull.length / 3)) {
						var point = AssemblyFrames.transformPoint(blank, stockHull[vertex * 3],
							stockHull[vertex * 3 + 1], stockHull[vertex * 3 + 2]);
						var local = inverse.transformPoint(new Vec3(point.x / 1000, point.y / 1000, point.z / 1000));
						seatedVertices.push(local.x); seatedVertices.push(local.y); seatedVertices.push(local.z);
					}
					var seatedBody:robotkit.manipulation.ClearanceBodyData = {name: "machine:mill/seated-stock",
						link: rootLink, vertices: seatedVertices, tool: false};
					var clampedBodies:Array<robotkit.manipulation.ClearanceBodyData> = [];
					var clampedParts = new Map<String, Bool>();
					var openVise = new Map<String, AssemblyFrame>();
					for (occurrence in flat.occurrences) openVise.set(occurrence.id, state.worldPose(occurrence.id));
					state.setJoint("mill/vise/jaw", vise.grippedPosition);
					for (occurrence in flat.occurrences) {
						var open = openVise.get(occurrence.id), closed = state.worldPose(occurrence.id);
						if (open == null) throw "Clamped vise lost its open geometry";
						if (Math.abs(open.x - closed.x) + Math.abs(open.y - closed.y) + Math.abs(open.z - closed.z) < 1e-6) continue;
						var physical = [for (part in millPhysical.parts) if (part.id == occurrence.definition) part][0];
						var hull = physical.collisionHull, vertices:Array<Float> = [];
						if (hull == null) throw "Clamped vise lost its CAD hull";
						for (vertex in 0...Std.int(hull.length / 3)) {
							var point = AssemblyFrames.transformPoint(closed, hull[3 * vertex], hull[3 * vertex + 1], hull[3 * vertex + 2]);
							var local = inverse.transformPoint(new Vec3(point.x / 1000, point.y / 1000, point.z / 1000));
							vertices.push(local.x); vertices.push(local.y); vertices.push(local.z);
						}
						clampedParts.set("machine:" + occurrence.id, true);
						clampedBodies.push({name: "machine:vise-clamped/" + occurrence.id,
							link: rootLink, vertices: vertices, tool: false});
					}
					state.setJoint("mill/vise/jaw", 0);
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
					var closedTransitSpaces = new Map<String, ManipulatorPlannerSpace>();
					var exchangeSpaces = new Map<String, ManipulatorPlannerSpace>();
					var viseSpaces = new Map<String, ReachPlannerStates>();
					var startupSpace:Null<ManipulatorPlannerSpace> = null;
					var exteriorAnchors = new Map<String, Array<Float>>();
					var certifiedEntries = new Map<String, TendingReachTransfer>();
					var retainedPoses = new Map<Int, Bool>();
					var resolvedPoses = new Map<Int, Bool>();
					var closedDoorParts = new Map<String, Bool>();
					for (body in bodies) if (StringTools.startsWith(body.name, "machine:door-closed/"))
						closedDoorParts.set("machine:" + body.name.substr("machine:door-closed/".length), true);
					for (tool in [DualArmGripperTool.TOOL_CHANNEL_A, DualArmGripperTool.TOOL_CHANNEL_B]) {
						var transitBodies:Array<robotkit.manipulation.ClearanceBodyData> = [];
						for (index in 0...bodies.length) {
							var body = bodies[index];
							if (body.name == "machine:infeedTray/blank-" + firstSeat || StringTools.startsWith(body.name, "machine:door-closed/")) continue;
								var vertices = index < loadedHulls.length
								? loadedHulls[index].vertices : body.vertices;
							transitBodies.push({name: body.name, link: body.link, vertices: vertices, tool: body.tool});
						}
						for (payload in exchangePayloads) transitBodies.push(payload.body);
						transitBodies.push(seatedBody);
						for (body in clampedBodies) transitBodies.push(body);
						// A payload is not structural. Use its actual grasp posture so the
						// articulation's reference-overlap rules cannot hide a folded-arm contact.
						var graspIndex = -1;
						for (index in 0...requests.length)
							if (requests[index].tool == tool && requests[index].name == firstStation) graspIndex = index;
						if (graspIndex < 0) throw "Reach transit has no initial grasp posture";
						transitSpaces.set(tool, transitSpace(arm, transitBodies, solved[graspIndex], exchangePayloads));
						// Unload waits for the open jaw and removes the seated part before
						// exchange. Both real held parts remain; the empty vise is not a
						// third workpiece or a second, clamped copy of the moving jaw.
						var exchangeBodies = [for (body in transitBodies) if (body.name != seatedBody.name &&
							!StringTools.startsWith(body.name, "machine:vise-clamped/")) body];
						exchangeSpaces.set(tool, transitSpace(arm, exchangeBodies, solved[graspIndex], exchangePayloads));
						var holder = tool == DualArmGripperTool.TOOL_CHANNEL_A ? "arm/tool/gripperA" : "arm/tool/gripperB";
						var activePayloads = [for (payload in exchangePayloads) if (payload.holder == holder) payload];
						var inactivePayloads = [for (payload in exchangePayloads) if (payload.holder != holder) payload];
						// Unload entry and post-load retreat have an empty active gripper,
						// a seated part, and the opposite gripper's payload. Initial load
						// entry instead carries the active blank into an empty, open vise.
						// Check those states separately: combining them creates a phantom
						// duplicate of the same workpiece and blocks otherwise clear paths.
						var loadedBodies = [for (body in transitBodies) if (body.name != activePayloads[0].body.name && !clampedParts.exists(body.name)) body];
						var loadBodies = [for (body in exchangeBodies) if (body.name != inactivePayloads[0].body.name) body];
						var unloadStates = [transitSpace(arm, loadedBodies, solved[graspIndex], inactivePayloads)];
						if (production.retreatsAfterUnload(tool))
							unloadStates.push(transitSpace(arm, loadBodies, solved[graspIndex], activePayloads));
						viseSpaces.set("vise@" + tool, new ReachPlannerStates(unloadStates));
						viseSpaces.set("vise-load@" + tool, new ReachPlannerStates([
							transitSpace(arm, loadBodies, solved[graspIndex], activePayloads),
							transitSpace(arm, loadedBodies, solved[graspIndex], inactivePayloads)]));
						// Closed-door handling carries one part: the next blank or the finish.
						// Both payloads are present together only during the open-door exchange.
						var closedPayloads = [for (payload in exchangePayloads) if (payload.holder == holder) payload];
						var closedBodies = [for (body in transitBodies)
							if (!StringTools.startsWith(body.name, "payload:") && !closedDoorParts.exists(body.name)) body];
						for (payload in closedPayloads) closedBodies.push(payload.body);
						// Outside handling may overlap closure. Certify the complete door
						// sweep rather than just either parked endpoint.
						for (body in sweptDoorBodies) closedBodies.push(body);
						closedTransitSpaces.set(tool, transitSpace(arm, closedBodies, solved[graspIndex], closedPayloads));
						for (needed in transferRequests) if (needed.role == Startup && needed.tool == tool) {
							var emptyBodies = [for (body in closedBodies) if (!StringTools.startsWith(body.name, "payload:")) body];
							for (body in bodies) if (body.name == "machine:infeedTray/blank-" + firstSeat) emptyBodies.push(body);
							startupSpace = transitSpace(arm, emptyBodies, solved[graspIndex], []);
						}
						for (index in 0...requests.length)
							if (requests[index].tool == tool && requests[index].name == "outside staging")
								exteriorAnchors.set(tool, solved[index].copy());
					}
					for (i in selectionOrder) {
						if (!valid) break;
						if (retainedPoses.exists(i)) continue;
						var request = requests[i];
						var incoming = trayEntries.get(request.name + "@" + request.tool);
						var clearanceIndex = -1;
						if (incoming != null) for (index in 0...requests.length)
							if (requests[index].tool == request.tool && requests[index].name == incoming.toPose) clearanceIndex = index;
						if (incoming != null && clearanceIndex < 0) throw "Tray task lost its clearance endpoint";
						// Door routes share one bounded turn convention. Ranking against
						// successive intermediate poses can accumulate a full wrist turn
						// that remains after those poses are pruned from the mission.
						var turnReference = solved[i];
						if (transitWaypoint(request.name) || request.name == "vise" || request.name == "vise-load")
							for (index in 0...requests.length)
								if (requests[index].tool == request.tool && requests[index].name == "outside staging")
									turnReference = solved[index];
						if (incoming != null) {
							var from = -1;
							for (index in 0...requests.length)
								if (requests[index].tool == incoming.fromTool && requests[index].name == incoming.fromPose) from = index;
							if (from < 0) throw "Tray selection lost its production entry posture";
							turnReference = solved[from];
						}
						preferNearbyTurns(arm, solved[i], turnReference);
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
						// At the exchange both TCPs carry stock. Certify the inactive part as
						// well as the selected gripper at insertion, retreat and the doorway.
						var pointPayloads:Array<ReachPayload> = [];
						if (!StringTools.startsWith(request.name, "infeed-") &&
							!StringTools.startsWith(request.name, "outfeed-")) {
							for (payload in exchangePayloads) {
								var active = StringTools.startsWith(payload.holder + "/", activeToolPrefix);
								var body = payload.body;
								var carried:ReachPayload = {body: {name: body.name, link: body.link,
									vertices: body.vertices, tool: active}, holder: payload.holder};
								pointBodies.push(carried.body); pointPayloads.push(carried);
							}
							// At contact the selected payload represents the stock being
							// transferred; at transport poses check the seated instance too.
							if (request.name != "vise" && request.name != "vise-load") {
								pointBodies.push(seatedBody);
								for (body in clampedBodies) pointBodies.push(body);
							}
						}
						if (request.name == "outside staging")
							for (body in sweptDoorBodies) pointBodies.push(body);
						var pointClearance = reachClearances(arm, pointBodies, parkPose, contactBodies, contact,
							converted.jointInterfaces, pointPayloads);
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
							var contactGeometry = [for (body in pointBodies) if (body.name != seatedBody.name &&
								!StringTools.startsWith(body.name, "machine:vise-clamped/")) body];
							pairedClearance = reachClearances(arm, contactGeometry, parkPose, pairedBodies,
								requests[paired].name == "vise" || requests[paired].name == "vise-load" || pairedStock != null,
								converted.jointInterfaces, pointPayloads);
						}
						var hit = firstViolation(pointClearance, solved[i]);
						var point = request.point;
						var selected:Null<ReachCandidate> = null;
						var pairFailures:Array<String> = [];
						// Numeric IK has several shoulder, elbow and wrist branches. A colliding
						// first solution does not mean the pose is unreachable. Roll about the
						// approach axis to keep the physical valve and hose clear as well.
						var yaws = graspYaws.copy();
						if (i > 0 && requests[i - 1].tool == request.tool) {
							var precedingYaw = yawsSolved[i - 1];
							yaws = [precedingYaw].concat([for (yaw in yaws) if (yaw != precedingYaw) yaw]);
						}
						if (incoming != null) {
							// A contact gripper cannot change roll relative to its held part.
							// The vise fixes the machined part heading, so its grasp owns the
							// roll for the corresponding inventory task as well.
							var visePose = StringTools.startsWith(request.name, "infeed-") ? "vise-load-approach" : "vise-approach";
							var viseIndex = -1;
							for (index in 0...requests.length) if (requests[index].tool == request.tool && requests[index].name == visePose) viseIndex = index;
							if (viseIndex < 0 || !resolvedPoses.exists(viseIndex)) throw "Inventory grasp lost its resolved vise heading";
							yaws = [yawsSolved[viseIndex]];
						}
						var rankApproach = request.name == "vise-approach" || request.name == "vise-load-approach";
						var rankEntry = rankApproach || incoming != null;
						// Rank an exchange when its other endpoint is already final. The
						// selected path is retained, so export cannot choose a different detour.
						var exchangeRequests:Array<TendingTransferRequest> = [];
						for (needed in transferRequests) if (needed.role == Exchange &&
							((needed.tool == request.tool && needed.toPose == request.name) ||
							(needed.fromTool == request.tool && needed.fromPose == request.name))) {
							var otherTool = needed.tool == request.tool ? needed.fromTool : needed.tool;
							var otherPose = needed.tool == request.tool ? needed.fromPose : needed.toPose;
							for (index in 0...requests.length) if (requests[index].tool == otherTool &&
								requests[index].name == otherPose && resolvedPoses.exists(index)) exchangeRequests.push(needed);
						}
						var exchangeSeeds:Array<Array<Float>> = [];
						var exchangeYaws:Array<Float> = [];
						for (needed in exchangeRequests) {
							var otherTool = needed.tool == request.tool ? needed.fromTool : needed.tool;
							var otherPose = needed.tool == request.tool ? needed.fromPose : needed.toPose;
							for (index in 0...requests.length) if (requests[index].tool == otherTool && requests[index].name == otherPose) {
								exchangeSeeds.push(solved[index]); exchangeYaws.push(yawsSolved[index]);
							}
						}
						if (exchangeYaws.length > 0) yaws = exchangeYaws.concat([for (yaw in yaws) if (!exchangeYaws.contains(yaw)) yaw]);
						var prepareStaging = request.name == "vise-load-approach", stagingIndex = -1;
						for (index in 0...requests.length)
							if (requests[index].tool == request.tool && requests[index].name == "outside staging") stagingIndex = index;
						var stagingChecks:Array<ReachClearanceCheck> = [];
						if (prepareStaging) {
							if (stagingIndex < 0) throw "Approach ranking lost its cell-derived staging pose";
							var stagingBodies = pointBodies.copy();
							for (body in bodies) if (StringTools.startsWith(body.name, "machine:door-closed/")) stagingBodies.push(body);
							for (body in sweptDoorBodies) stagingBodies.push(body);
							stagingChecks = reachClearances(arm, stagingBodies, parkPose, [], false,
								converted.jointInterfaces, pointPayloads);
						}
						var evaluated:Array<Array<Float>> = [];
						// Preserve the preceding route branch before trying other IK branches.
						var routeSeeds:Array<Array<Float>> = exchangeSeeds.copy();
						if (incoming != null) routeSeeds.push(turnReference);
						if (i > 0 && requests[i - 1].tool == request.tool) routeSeeds.push(solved[i - 1]);
						routeSeeds.push(solved[i]);
						for (pan in [-Math.PI / 2, 0.0, Math.PI / 2, Math.PI])
							for (elbow in [-Math.PI / 2, Math.PI / 2])
								for (wristSeed in [-Math.PI / 2, Math.PI / 2])
									routeSeeds.push([pan, -Math.PI / 2, elbow, -elbow, wristSeed, 0.0]);
						for (yaw in yaws) {
							if (selected != null && !rankEntry) break;
							var reportedPairFailure = false;
							var pose = graspFrame(point, graspWorld, yaw);
							var desiredTool = new Transform3(new Vec3(point[0] / 1000, point[1] / 1000, point[2] / 1000),
								new Quat(pose.qx, pose.qy, pose.qz, pose.qw));
							var target = inverse.compose(desiredTool.compose(request.toolOffset.inverse()));
							for (routeSeed in routeSeeds) {
								if (selected != null && !rankEntry) break;
								var attempt = arm.solve(target, routeSeed, options);
								preferNearbyTurns(arm, attempt.q, exchangeSeeds.length > 0 ? exchangeSeeds[0] : turnReference);
								if (attempt.converged && withinMargins(arm, attempt.q) &&
									clearanceViolation(clearance, attempt.q) == null &&
									firstViolation(pointClearance, attempt.q) == null) {
									var candidateSeconds = 0.0, candidateFrom = turnReference;
									var candidateContact:Null<TendingReachPose> = null;
									var candidateStaging:Null<TendingReachPose> = null;
									var candidateEntry:Null<Array<Array<Float>>> = null;
									var candidateClearance:Null<TendingReachPose> = null;
									var candidateDescent:Null<Array<Array<Float>>> = null;
									var candidateExchanges:Array<TendingReachTransfer> = [];
									if (rankEntry) {
										var duplicate = false;
										for (previous in evaluated) {
											var separation = 0.0;
											for (joint in 0...attempt.q.length) separation += Math.abs(previous[joint] - attempt.q[joint]);
											if (separation < 1e-6) duplicate = true;
										}
										if (duplicate) continue;
										evaluated.push(attempt.q.copy());
										if (prepareStaging) {
											var staging = requests[stagingIndex], stagingPose = graspFrame(staging.point, graspWorld, yaw);
											var stagingTool = new Transform3(new Vec3(stagingPose.x / 1000, stagingPose.y / 1000, stagingPose.z / 1000),
												new Quat(stagingPose.qx, stagingPose.qy, stagingPose.qz, stagingPose.qw));
											var staged = arm.solve(inverse.compose(stagingTool.compose(staging.toolOffset.inverse())), attempt.q, options);
											preferNearbyTurns(arm, staged.q, attempt.q);
											if (staged.converged && withinMargins(arm, staged.q) &&
												firstViolation(stagingChecks, staged.q) == null) {
												candidateStaging = {name: staging.name, tool: staging.tool, position: staged.q,
													target: stagingPose, toolYaw: yaw};
												candidateFrom = staged.q;
											}
										}
									}
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
										preferNearbyTurns(arm, insertion.q, attempt.q);
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
										candidateContact = {name: destination.name, tool: destination.tool, position: insertion.q,
											target: seated, toolYaw: yaw};
										if (rankEntry) {
											var insertionProblem = insertionFailure(arm, attempt.q, insertion.q);
											if (insertionProblem != null) {
												if (!reportedPairFailure) {
													pairFailures.push('roll=$yaw insertion: ' + insertionProblem);
													reportedPairFailure = true;
												}
												continue;
											}
										}
									}
									if (rankEntry) {
										// Independent rest-to-rest joint minima remain admissible for
										// curved paths; synchronized straight-edge timing does not.
										var lowerSeconds = 2 * timing.lowerBound(candidateFrom, attempt.q);
										if (candidateStaging != null)
											lowerSeconds = Math.min(lowerSeconds, 2 * timing.lowerBound(turnReference, attempt.q));
										// Holding-posture search can move the start endpoint, so its
										// original endpoint minimum cannot prune that search.
										if (!prepareStaging && selected != null && lowerSeconds >= selected.seconds)
											continue;
										var stagingCandidate = candidateStaging;
										if (stagingCandidate != null) {
											var anchor = exteriorAnchors.get(request.tool), connected = true;
											if (anchor == null) throw "Staging lost its exterior transport anchor";
											for (closedSpace in closedTransitSpaces) {
												var outsidePlanner = new RrtConnect(closedSpace, 17);
												if (selected != null) outsidePlanner.maxIterations = APPROACH_SEARCH_ITERATIONS;
												// This is a connectivity witness, not an executed route.
												// Its certified edges need no shortening before disposal.
												outsidePlanner.shortcutRounds = 0;
												if (!outsidePlanner.plan(anchor, stagingCandidate.position).found()) { connected = false; break; }
											}
											if (!connected) { candidateStaging = null; candidateFrom = turnReference; }
										}
										var space:motionkit.planner.PlannerSpace = incoming != null ? closedTransitSpaces.get(request.tool) :
											rankApproach ? viseSpaces.get(request.name.substr(0, request.name.length - "-approach".length) + "@" + request.tool) : transitSpaces.get(request.tool);
										if (space == null) throw "Station ranking lost its payload collision space";
										var entryPlanner = new RrtConnect(space, 17);
										// Establish a certified incumbent with the full feasibility
										// budget. Only attempts to improve it use the smaller budget.
										if (selected != null) entryPlanner.maxIterations = APPROACH_SEARCH_ITERATIONS;
										var entryGoal = attempt.q;
										if (incoming != null) {
											var lifted = requests[clearanceIndex], liftedPose = graspFrame(lifted.point, graspWorld, yaw);
											var liftedTool = transform(liftedPose);
											var lift = arm.solve(inverse.compose(liftedTool.compose(request.toolOffset.inverse())), attempt.q, options);
											preferNearbyTurns(arm, lift.q, attempt.q);
											if (!lift.converged || !withinMargins(arm, lift.q) || space.invalid(lift.q) != null) continue;
											candidateClearance = {name: lifted.name, tool: lifted.tool, position: lift.q,
												target: liftedPose, toolYaw: yaw};
											entryGoal = lift.q;
											var descent = entryPlanner.plan(lift.q, attempt.q);
											if (descent.waypoints == null) {
												if (!reportedPairFailure) { pairFailures.push('roll=$yaw tray approach: ' + descent.failure); reportedPairFailure = true; }
												continue;
											}
											candidateDescent = timing.refine(space, descent.waypoints);
										}
										var entry = entryPlanner.plan(candidateFrom, entryGoal);
										if (entry.waypoints == null && candidateStaging != null) {
											// A prepared posture is an alternative, not a replacement
											// for the already checked exterior staging posture.
											candidateStaging = null;
											entry = entryPlanner.plan(turnReference, attempt.q);
										}
										if (entry.waypoints == null) {
											if (!reportedPairFailure) {
												pairFailures.push('roll=$yaw entry: ' + entry.failure);
												reportedPairFailure = true;
											}
											continue;
										}
										candidateEntry = timing.refine(space, entry.waypoints);
										if (prepareStaging) {
											var anchor = exteriorAnchors.get(request.tool);
											if (anchor == null) throw "Holding posture lost its exterior transport anchor";
											for (holding in holdingPaths(arm, candidateEntry, pointBodies, transform(armPlacement),
												cell.opening.y - CollisionClearance.MARGIN * 1000, stagingChecks)) {
												if (timing.duration(holding.entry) >= timing.duration(candidateEntry)) continue;
												var connected = true;
												for (closedSpace in closedTransitSpaces) {
													// Reuse the remainder of the known retreat as a connectivity
													// witness, after checking it with the closed door and payload.
													var witness = holding.exterior.copy();
													if (!samePosition(witness[witness.length - 1], anchor)) witness.push(anchor);
													var edges:Array<Float> = [], validWitness = true;
													for (point in witness) if (closedSpace.invalid(point) != null) { validWitness = false; break; }
													if (validWitness) {
														for (index in 1...witness.length) {
															for (value in witness[index - 1]) edges.push(value);
															for (value in witness[index]) edges.push(value);
														}
														if (witness.length > 1) for (clear in closedSpace.edgesClear(edges, witness.length - 1)) if (!clear) validWitness = false;
													}
													if (validWitness) continue;
													var outsidePlanner = new RrtConnect(closedSpace, 17);
													if (selected != null) outsidePlanner.maxIterations = APPROACH_SEARCH_ITERATIONS;
													outsidePlanner.shortcutRounds = 0;
													if (!outsidePlanner.plan(anchor, holding.position).found()) { connected = false; break; }
												}
												if (!connected) continue;
												var world = transform(armPlacement).compose(arm.forwardKinematics(holding.position).compose(request.toolOffset));
												candidateStaging = {name: requests[stagingIndex].name, tool: request.tool, position: holding.position,
													target: {x: world.translation.x * 1000, y: world.translation.y * 1000, z: world.translation.z * 1000,
														qx: world.rotation.x, qy: world.rotation.y, qz: world.rotation.z, qw: world.rotation.w}, toolYaw: yaw};
												candidateEntry = timing.refine(space, holding.entry);
												break;
											}
										}
										// Static collision rules and rest-to-rest limits are reversible.
										candidateSeconds = 2 * timing.duration(candidateEntry);
										if (candidateDescent != null) candidateSeconds += 2 * timing.duration(candidateDescent);
										var exchangesValid = true;
										for (needed in exchangeRequests) {
											var other = -1;
											var destination = needed.tool == request.tool;
											for (index in 0...requests.length) if (requests[index].tool == (destination ? needed.fromTool : needed.tool) &&
												requests[index].name == (destination ? needed.fromPose : needed.toPose)) other = index;
											if (other < 0) throw "Exchange ranking lost its other endpoint";
											var exchangeSpace = exchangeSpaces.get(needed.tool);
											if (exchangeSpace == null) throw "Exchange ranking lost its two-payload collision space";
											var exchangePlanner = new RrtConnect(exchangeSpace, 17);
											if (selected != null) exchangePlanner.maxIterations = APPROACH_SEARCH_ITERATIONS;
											var exchange = exchangePlanner.plan(destination ? solved[other] : attempt.q, destination ? attempt.q : solved[other]);
											if (exchange.waypoints == null) { exchangesValid = false; break; }
											var exchangePath = timing.refine(exchangeSpace, exchange.waypoints);
											candidateExchanges.push({request: needed, positions: exchangePath});
											candidateSeconds += production.exchangeCount(needed.fromTool, needed.tool) * timing.duration(exchangePath);
										}
										if (!exchangesValid || (selected != null && candidateSeconds >= selected.seconds)) continue;
									}
									selected = {approach: {name: request.name, tool: request.tool,
										position: attempt.q, target: pose, toolYaw: yaw}, contact: candidateContact,
										staging: candidateStaging, clearance: candidateClearance, descent: candidateDescent,
										entry: candidateEntry, exchanges: candidateExchanges, seconds: candidateSeconds};
									if (!rankEntry) break;
								}
							}
						}
						disposeClearances(pointClearance);
						disposeClearances(pairedClearance);
						disposeClearances(stagingChecks);
						if (selected != null) {
							resolvedPoses.set(i, true);
							for (exchange in selected.exchanges) certifiedEntries.set(certificateKey(exchange.request), exchange);
							solved[i] = selected.approach.position; targets[i] = selected.approach.target;
							yawsSolved[i] = selected.approach.toolYaw;
							if (request.name == "outside staging") exteriorAnchors.set(request.tool, solved[i].copy());
							var contact = selected.contact, staging = selected.staging;
							if (paired >= 0 && contact != null) {
								solved[paired] = contact.position; targets[paired] = contact.target;
								yawsSolved[paired] = contact.toolYaw;
								retainedPoses.set(paired, true);
							}
							if (staging != null) {
								solved[stagingIndex] = staging.position; targets[stagingIndex] = staging.target;
								yawsSolved[stagingIndex] = staging.toolYaw;
							}
							if (rankEntry) {
								var entry = selected.entry;
								if (entry == null) throw "A selected production station must own its certified entry";
								var owned:Null<TendingTransferRequest> = incoming;
								if (owned == null) for (needed in transferRequests)
									if (needed.role == ViseEntry && needed.tool == request.tool && needed.toPose == request.name) owned = needed;
								if (owned != null) certifiedEntries.set(certificateKey(owned), {request: owned, positions: entry});
								if (incoming != null) {
									var lifted = selected.clearance, descent = selected.descent;
									if (lifted == null || descent == null) throw "Selected tray task lost its lift certificate";
									solved[clearanceIndex] = lifted.position; targets[clearanceIndex] = lifted.target;
									yawsSolved[clearanceIndex] = lifted.toolYaw; retainedPoses.set(clearanceIndex, true);
									for (needed in transferRequests) if (needed.role == TrayApproach && needed.tool == request.tool && needed.toPose == request.name)
										certifiedEntries.set(certificateKey(needed), {request: needed, positions: descent});
								}
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
					// Guide poses retain their endpoint coverage checks above. Certify
					// the complete routes exported to the mission, not an unused guide chain.
					var transfers:Array<TendingReachTransfer> = [];
					// The production sequence supplies the complete paths it will use.
					// Unused tool/station combinations cannot refuse a valid program.
					if (valid) for (needed in transferRequests) {
						var space:motionkit.planner.PlannerSpace = needed.role == Startup ? startupSpace : needed.role == Exchange ? exchangeSpaces.get(needed.tool) :
							needed.role == ViseEntry ? viseSpaces.get(needed.station + "@" + needed.tool) :
							needed.door == Closed ? closedTransitSpaces.get(needed.tool) : transitSpaces.get(needed.tool);
						if (space == null) throw "Production transfer lost its checked payload space";
						var from = -1, to = -1;
						for (index in 0...requests.length) {
							var pose = requests[index];
							if (pose.tool == needed.fromTool && pose.name == needed.fromPose) from = index;
							if (pose.tool == needed.tool && pose.name == needed.toPose) to = index;
						}
						if ((from < 0 && needed.role != Startup) || to < 0) throw 'Production transfer lost ${needed.station}';
						var fromPosition = needed.role == Startup ? parkPose : solved[from];
						var retained = certifiedEntries.get(certificateKey(needed));
						var positions = retained == null ? null : retained.positions;
						var failure = "";
						if (positions == null) {
							var transfer = new RrtConnect(space, 17).plan(fromPosition, solved[to]);
							positions = transfer.waypoints == null ? null : timing.refine(space, transfer.waypoints); failure = transfer.failure;
						} else if (!samePosition(positions[0], fromPosition) || !samePosition(positions[positions.length - 1], solved[to]))
							throw "Approach selection changed an endpoint of its certified entry";
						if (positions == null) {
							valid = false; failedAt = needed.station + "@" + needed.tool + " from " + needed.fromTool;
							failedReason = failure; break;
						}
						transfers.push({request: needed, positions: positions});
					}
					for (space in transitSpaces) space.dispose();
					for (space in exchangeSpaces) space.dispose();
					for (space in viseSpaces) space.dispose();
					for (space in closedTransitSpaces) space.dispose();
					if (startupSpace != null) startupSpace.dispose();
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
							poses: poseResults, transfers: transfers, trays: trayLayouts,
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
		throw 'The selected Reach1300/550 mm cell layout (tray pitch ${spacing.x} x ${spacing.y} mm) ' +
			'does not reach every required pose with the joint and wrist margins' +
			(failures.length == 0 ? "" : '; candidates: ${failures.join("; ")}');
	}

	/** Candidate holding postures along an already certified retreat, nearest the vise first.
	 * Orientation is free here: every moving hull must be outside the mill front plane,
	 * and the complete native scene/door sweep must clear. The grid only selects candidates;
	 * it never replaces a continuous edge certificate.
	 */
	static function holdingPaths(arm:Manipulator, entry:Array<Array<Float>>,
		data:Array<robotkit.manipulation.ClearanceBodyData>, placement:Transform3, frontMm:Float,
		checks:Array<ReachClearanceCheck>):Array<ReachHoldingPath> {
		var moving = [for (body in data) if (!StringTools.startsWith(body.name, "machine:")) body];
		var links = [for (body in moving) body.link], result:Array<ReachHoldingPath> = [];
		for (offset in 0...entry.length - 1) {
			var edge = entry.length - 2 - offset, from = entry[edge], to = entry[edge + 1];
			for (sample in 0...17) {
				var fraction = 1.0 - sample / 16.0;
				var q = sample == 0 ? to.copy() : sample == 16 ? from.copy() :
					[for (joint in 0...from.length) from[joint] + fraction * (to[joint] - from[joint])];
				if (!withinMargins(arm, q)) continue;
				var poses = arm.linkPoses(q, links), outside = true;
				for (index in 0...moving.length) {
					var world = placement.compose(poses[index]), hull = moving[index].vertices;
					for (vertex in 0...Std.int(hull.length / 3)) {
						var point = world.transformPoint(new Vec3(hull[3 * vertex], hull[3 * vertex + 1], hull[3 * vertex + 2]));
						if (point.y * 1000 > frontMm) { outside = false; break; }
					}
					if (!outside) break;
				}
				if (!outside || firstViolation(checks, q) != null) continue;
				var tail = entry.slice(edge + 1);
				if (samePosition(q, tail[0])) tail = tail.slice(1);
				if (tail.length > 0) {
					var outside = entry.slice(0, edge + 1); outside.reverse();
					if (outside.length > 0 && samePosition(q, outside[0])) outside = outside.slice(1);
					result.push({position: q, entry: [q].concat(tail), exterior: [q].concat(outside)});
				}
			}
		}
		return result;
	}

	/** A production edge owns its certificate, including cross-tool edges. */
	static function certificateKey(request:TendingTransferRequest):String
		return request.station + "@" + request.tool + "@" + request.fromTool;

	/** Check actual convex hulls independently of the placement's separating bounds. */
	static function checkFixtureDoorSweep(sweeps:Array<ReachEnvironmentHull>, fixture:Array<ReachEnvironmentHull>):Void {
		var description = new CollisionDescription();
		var doorObjects = [for (body in sweeps) description.addFixed(body.name, collisionkit.CollisionPose.identity(),
			CollisionGeometry.Convex(body.vertices), 0, 0, "prismatic door sweep")];
		var fixtureObjects = [for (body in fixture) description.addFixed(body.name, collisionkit.CollisionPose.identity(),
			CollisionGeometry.Convex(body.vertices), 1, 0, "installed cell fixture hull")];
		for (door in doorObjects) for (part in fixtureObjects)
			description.setObjectRule(door, part, CollisionPairRule.Check, CollisionPairStatus.Checked);
		var world = new NativeCollisionWorld();
		var problem:Null<String> = null;
		try {
			var build = description.build(world);
			var hit = world.violation(new CollisionMargins(2, CollisionClearance.MARGIN));
			if (hit != null) problem = 'Cell fixture intersects the door sweep: ' +
				description.objects[build.objectOf(hit.a)].name + "/" + description.objects[build.objectOf(hit.b)].name +
				' (${hit.distance} m, needs ${hit.required} m)';
		} catch (error:Dynamic) {
			world.dispose();
			throw error;
		}
		world.dispose();
		if (problem != null) throw problem;
	}

	/** Transform world hulls into the same root frame as the articulation. */
	static function environmentBodies(data:Array<ReachEnvironmentHull>, inverse:Transform3, rootLink:String):Array<robotkit.manipulation.ClearanceBodyData> {
		return [for (body in data) {
			var vertices:Array<Float> = [];
			for (index in 0...Std.int(body.vertices.length / 3)) {
				var point = inverse.transformPoint(new Vec3(body.vertices[3 * index], body.vertices[3 * index + 1], body.vertices[3 * index + 2]));
				vertices.push(point.x); vertices.push(point.y); vertices.push(point.z);
			}
			{name: "machine:" + body.name, link: rootLink, vertices: vertices, tool: false};
		}];
	}

	/** Tray endpoints are selected after both production staging postures. */
	static function trayPose(name:String):Bool
		return StringTools.startsWith(name, "infeed-") || StringTools.startsWith(name, "outfeed-");

	/** Doorway guide poses retained for endpoint reach coverage. */
	static function doorwayGuidePoses(station:String):Array<String>
		return ["outside staging", "outside door", "doorway center", "doorway-inner", "inside doorway",
			"inside door", "vise-transfer", "vise-transfer-inner", station + "-approach"];

	static function transitWaypoint(name:String):Bool {
		for (station in ["vise", "vise-load"]) {
			var poses = doorwayGuidePoses(station);
			if (poses.indexOf(name) > 0) return true;
		}
		return false;
	}

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
			reference:Array<Float>, payloads:Array<ReachPayload>):ManipulatorPlannerSpace {
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
		for (carried in payloads) for (index in 0...moving.length) if (moving[index].name == carried.body.name) {
			var payload = hulls[index];
			for (other in 0...description.objects.length) {
				if (other == payload) continue;
				if (StringTools.startsWith(description.objects[other].name,
					described.bodies.name + "/" + carried.holder + "/")) {
					// Point checks certify this contact before travel. The grasp transform
					// and jaw sweep are fixed in arm FK throughout a transport edge.
					description.setObjectRule(payload, other, CollisionPairRule.Allow, CollisionPairStatus.DeclaredContact);
					continue;
				}
				// A loose workpiece never acquires structural rest-overlap permissions.
				description.setObjectRule(payload, other, CollisionPairRule.Check, CollisionPairStatus.Checked);
			}
		}
		var build = description.build(new NativeCollisionWorld());
		return new ManipulatorPlannerSpace(new CollisionPlannerSpace(description, build, described.bodies,
			CollisionMargins.uniform(CollisionClearance.MARGIN)), arm, reference);
	}

	static function samePosition(a:Array<Float>, b:Array<Float>):Bool {
		if (a.length != b.length) return false;
		for (joint in 0...a.length) if (a[joint] != b[joint]) return false;
		return true;
	}

	static function jointMargin(arm:Manipulator, q:Array<Float>):Float {
		var margin = 1.0;
		for (i in 0...6) {
			var limit = arm.group.limitsOf(i), span = limit.upper - limit.lower;
			margin = Math.min(margin, Math.min(q[i] - limit.lower, limit.upper - q[i]) / span);
		}
		return margin;
	}

	/** Rank equivalent revolute turns before certifying their actual bounded path. */
	static function preferNearbyTurns(arm:Manipulator, q:Array<Float>, reference:Array<Float>):Void {
		var joints = arm.jointIndices(), period = 2 * Math.PI;
		for (index in 0...q.length) {
			if (arm.robot.joints[joints[index]].type != JointType.Revolute) continue;
			var limit = arm.group.limitsOf(index), span = limit.upper - limit.lower;
			var lower = limit.lower + MIN_JOINT_MARGIN * span;
			var upper = limit.upper - MIN_JOINT_MARGIN * span;
			var first = Math.ceil((lower - q[index]) / period), last = Math.floor((upper - q[index]) / period);
			var selected = q[index], distance = Math.abs(selected - reference[index]);
			for (turn in first...last + 1) {
				var candidate = q[index] + turn * period;
				if (Math.abs(candidate - reference[index]) < distance || selected < lower || selected > upper) {
					selected = candidate; distance = Math.abs(candidate - reference[index]);
				}
			}
			q[index] = selected;
		}
	}
	static function withinMargins(arm:Manipulator, q:Array<Float>):Bool
		return jointMargin(arm, q) >= MIN_JOINT_MARGIN && Math.abs(Math.sin(q[4])) >= 0.2;

	/** Endpoints can solve on branches that cannot follow the intervening Cartesian task. */
	static function insertionFailure(arm:Manipulator, from:Array<Float>, to:Array<Float>):Null<String> {
		var solver = new ManipulatorKinematics(arm, 1e-8), tolerance = HandlingPlanRunner.gripperTolerance();
		var line = new PoseLine(new PoseWaypoint(solver.forward(from), tolerance.position, tolerance.orientation),
			new PoseWaypoint(solver.forward(to), tolerance.position, tolerance.orientation), OrientationPolicy.Fixed, 0.1, 0.08);
		var path = new PosePath(HandlingPlanRunner.FRAME, [line]);
		var pieces = PoseSampling.pieces(line, HandlingPlanRunner.CARTESIAN_RESOLUTION, HandlingPlanRunner.ANGULAR_RESOLUTION);
		var distances = [for (index in 0...pieces + 1) index == pieces ? path.length() : path.length() * index / pieces];
		var joints = arm.jointIndices(), policy = JointMotionPolicy.mission();
		var velocity = [for (index in 0...joints.length) policy.rates(arm.robot.joints[joints[index]].type, arm.group.limitsOf(index)).velocity];
		var request = new PathRequest(distances, [for (distance in distances) path.poseAt(distance)], from, tolerance,
			[for (_ in joints) HandlingPlanRunner.MAX_JOINT_JUMP], velocity, 48,
			[for (_ in distances) OrientationPolicy.Fixed]);
		try {
			var curve = new StructuredJointPathPlanner(arm).plan(path, request, true);
			for (q in curve.q) if (!withinMargins(arm, q)) return "Cartesian insertion loses joint or wrist margin";
		} catch (error:Dynamic) return Std.string(error);
		return null;
	}
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
