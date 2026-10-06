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
import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge;
import cadkit.modeling.Part;
import cadkit.modeling.AssemblyState;
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
	/** Placement of the complete arm subassembly on the cell riser. */
	var armPlacement:AssemblyFrame;
	/** Base-flange part frame after the same fixed mate used by the assembled cell. */
	var armBaseFlange:AssemblyFrame;
	var jointMargin:Float;
	var wristSine:Float;
	/** The checked vertical lift clear of a tray seat plus two clearance margins, in millimetres. */
	var handlingApproachMm:Float;
	/** Joint pose that parks the arm facing away from the mill during CNC cutting. */
	var parkPose:Array<Float>;
	/** TCP height over a tray slot derived from the complete gripper envelope and blank. */
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
typedef TendingTrayLayout = {var id:String; var frame:AssemblyFrame; var slots:Array<TendingTraySlot>;}

/** Static reach, with each physical gripper TCP at the vise, tray slots and both sides of the
 * doorway. Mill and blank geometry decide the poses; each gripper supplies its own envelope.
 */
class TendingReach {
	/** A pocket's free pitch accounts for the opposing tool envelopes and both clearance sides. */
	public static inline var TRAY_SLOT_CLEARANCE_MM:Float =
		2 * DualArmGripperTool.INTER_GRIPPER_CLEARANCE_MM + 2 * ArmClearance.MARGIN * 1000;
	/** Assumed 6 x 40 mm replaceable pads leave handling clearance around the 60 mm stock face. */
	public static function tendingTool():DualArmGripperTool
		return new DualArmGripperTool(32, BenchMill.STOCK_WIDTH, 60, 16, false,
			BenchMill.STOCK_DEPTH - 4 * ArmClearance.MARGIN * 1000,
			6);

	public static function trayPitchX():Float return BenchMill.STOCK_WIDTH + TRAY_SLOT_CLEARANCE_MM;
	public static function trayPitchY():Float return BenchMill.STOCK_DEPTH + TRAY_SLOT_CLEARANCE_MM;

	public static function run():Void {
		var result = study(new EnclosedBenchMill());
		if (result.cls != Reach850 || result.riser != 550 || result.poses.length != 80 ||
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
		var toolPoseDelta = 0.0;
		for (joint in 0...viseA.length) toolPoseDelta = Math.max(toolPoseDelta, Math.abs(viseA[joint] - viseB[joint]));
		if (toolPoseDelta < 0.01) throw "The reach study did not account for the second gripper's TCP offset";
		trace('Tending reach: ${result.cls}, ${result.riser} mm riser, ${result.poses.length} poses, ' +
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
		// Lift the stock above both the tray seat lands and the vise jaws. The jaw
		// top is 20 mm above the vise datum blank's underside, so jaw clearance can
		// govern even when the tray seat is the lower obstacle.
		var vise = cell.mill.vise;
		if (vise == null) throw "Tending reach needs the mill's preset vise";
		var movingJaw = state.worldPose("mill/vise/movingJaw");
		var viseJawClearance = movingJaw.z + vise.movingJaw.height - blank.z + 2 * ArmClearance.MARGIN * 1000;
		var approach = Math.max(PartTray.SEAT_Z + 2 * ArmClearance.MARGIN * 1000, viseJawClearance);
		var trayClearanceMm = toolDepth + BenchMill.STOCK_HEIGHT + 2 * ArmClearance.MARGIN * 1000;
		var viseNormal = AssemblyFrames.transformVector(blank, grasp.approachAxis.x,
			grasp.approachAxis.y, grasp.approachAxis.z);
		var names = ["vise", "vise-approach", "outside door", "inside door"];

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
			trayLayouts.push({id: id, frame: frame, slots: slots});
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
		for (cls in [Reach850, Reach1300]) {
			var firstClassFailure:Null<String> = null, lastClassFailure:Null<String> = null;
			var riserFailures:Array<String> = [];
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
			var toolStandOff = toolLength + source.baseFlange.flangeDiameter + toolDepth + 20;
			var classBaseY = cell.opening.y - toolStandOff;
			// The doorway must pass the wrist as well as the tool. Keep its roll module
			// below the header; the vise approach can rise after it is inside.
			var wristPackage = toolLength + source.toolFlange.thickness + source.reference.d[5] + source.modules[5].length;
			var viaZ = Math.min(graspWorld.z + approach, cell.opening.z + cell.openingHeight - wristPackage - ArmClearance.MARGIN * 1000);
			if (viaZ <= cell.opening.z + ArmClearance.MARGIN * 1000) throw "Wrist and tool do not fit through the doorway";
			var baseTargets:Array<{name:String, point:Array<Float>}> = [
				{name: "vise", point: [graspWorld.x, graspWorld.y, graspWorld.z]},
				{name: "vise-approach", point: [graspWorld.x + viseNormal.x * approach,
					graspWorld.y + viseNormal.y * approach, graspWorld.z + viseNormal.z * approach]},
				{name: "outside door", point: [graspWorld.x, cell.opening.y - toolDepth, viaZ]},
				{name: "inside door", point: [graspWorld.x, cell.opening.y + toolDepth, viaZ]}];
			for (tray in trayLayouts) for (slot in tray.slots) {
				var name = tray.id == "infeedTray" ? StringTools.replace(slot.name, "slot-", "infeed-")
					: StringTools.replace(slot.name, "slot-", "outfeed-");
				var graspedSlot = AssemblyFrames.compose(slot.frame, graspLocal);
				var normal = AssemblyFrames.transformVector(slot.frame, grasp.approachAxis.x,
					grasp.approachAxis.y, grasp.approachAxis.z);
				baseTargets.push({name: name + "-clearance", point: [graspedSlot.x + normal.x * trayClearanceMm,
					graspedSlot.y + normal.y * trayClearanceMm, graspedSlot.z + normal.z * trayClearanceMm]});
				baseTargets.push({name: name + "-approach", point: [graspedSlot.x + normal.x * approach,
					graspedSlot.y + normal.y * approach, graspedSlot.z + normal.z * approach]});
				baseTargets.push({name: name, point: [graspedSlot.x, graspedSlot.y, graspedSlot.z]});
			}
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
			var baseShift = source.baseFlange.flangeDiameter / 2 + ArmClearance.MARGIN * 1000;
			// Search the geometry-derived base stand-off at half-radius increments,
			// checking the selected 550 mm riser before adjacent 50 mm fabrication sizes.
			var baseOffsets = [0.0, baseShift * 0.5, baseShift, baseShift * 1.5, 2 * baseShift];
			var riserHeights = [550.0];
			for (step in 1...25) {
				var lower = 550 - step * 50, upper = 550 + step * 50;
				if (lower >= 0) riserHeights.push(lower);
				if (upper <= 1200) riserHeights.push(upper);
			}
			for (baseOffset in baseOffsets) {
			for (height in riserHeights) {
				var armPlacement = AssemblyFrames.compose(AssemblyFrames.translation(source.reference.d[3], classBaseY + baseOffset, height),
					AssemblyFrames.fromRotationMatrix(0, 0, 0, [0.0, 1, 0, -1, 0, 0, 0, 0, 1]));
				var baseFlange = mountedBaseFlange(source, armPlacement);
				var inverse = transform(baseFlange).inverse();
				var solved:Array<Array<Float>> = [], targets:Array<AssemblyFrame> = [], yawsSolved:Array<Float> = [], seed = source.ready();
				var margin = 1.0, wrist = 1.0, valid = true;
				var failedAt:Null<String> = null, failedReason:Null<String> = null;
				var poseResults:Array<TendingReachPose> = [];
				for (pointIndex in 0...requests.length) {
					var request = requests[pointIndex], point = request.point;
					var selected:Null<Array<Float>> = null, selectedTarget:Null<AssemblyFrame> = null;
					var selectedYaw = 0.0;
					var selectionFailure = "no tool roll candidate";
					var yaws = request.name == "vise"
						? [0.0, Math.PI] : [0.0, Math.PI / 2, Math.PI, 3 * Math.PI / 2];
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
						var selfHit = clearance.violation(solution.q);
						if (selfHit != null) { selectionFailure = 'self ${selfHit.a}/${selfHit.b}'; continue; }
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
									if (attempt.converged && withinMargins(arm, attempt.q) && clearance.violation(attempt.q) == null) {
										selected = attempt.q; selectedTarget = pose; selectedYaw = yaw; break;
									}
								}
							}
						}
					}
					if (selected == null || selectedTarget == null) {
						valid = false; failedAt = request.name + "@" + request.tool; failedReason = selectionFailure; break;
					}
					solved.push(selected); targets.push(selectedTarget); yawsSolved.push(selectedYaw); seed = selected.copy();
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
					// Only the active tool-to-vise or selected-slot-blank pairs get the
					// reduced work-contact margin at their grasp pose. Every other cell
					// body, including the idle gripper and neighbouring blanks, keeps 5 mm.
					var viseContactBodies = [for (body in environment)
						if (StringTools.startsWith(body.name, "mill/vise/")) "machine:" + body.name];
					var full = new ArmClearance(arm, bodies, parkPose,
						ArmClearance.MARGIN, ArmClearance.CONTACT_MARGIN, viseContactBodies, interfaces);
					var readyHit = full.violation(parkPose);
					if (readyHit != null) {
						failedAt = "ready";
						failedReason = 'clearance ${readyHit.a}/${readyHit.b} (${readyHit.distance} m)';
						valid = false;
					}
					for (i in 0...solved.length) {
						if (!valid) break;
						var request = requests[i];
						var activeToolPrefix = request.tool == DualArmGripperTool.TOOL_CHANNEL_A
							? "arm/tool/gripperA/" : "arm/tool/gripperB/";
						var activeToolBodies = [for (body in bodies) {name: body.name, link: body.link,
							vertices: body.vertices, tool: StringTools.startsWith(body.name, activeToolPrefix)}];
						var selectedStock = trayStock(request.name);
						var contactBodies = viseContactBodies.copy();
						if (selectedStock != null) contactBodies.push("machine:" + selectedStock);
						var pointClearance = new ArmClearance(arm, activeToolBodies, parkPose,
							ArmClearance.MARGIN, ArmClearance.CONTACT_MARGIN, contactBodies, interfaces);
						var contact = request.name == "vise" || selectedStock != null;
						var hit = pointClearance.violation(solved[i], contact);
						if (hit == null) continue;
						var point = request.point;
						var alternative:Null<Array<Float>> = null;
						var alternativeTarget:Null<AssemblyFrame> = null;
						// Numeric IK has several shoulder, elbow and wrist branches. A colliding
						// first solution does not mean the pose is unreachable. Roll about the
						// approach axis to keep the physical valve and hose clear as well.
						var yaws = request.name == "vise"
							? [0.0, Math.PI] : [0.0, Math.PI / 2, Math.PI, 3 * Math.PI / 2];
						var alternativeYaw = 0.0;
						for (yaw in yaws) {
							if (alternative != null) break;
							var pose = graspFrame(point, graspWorld, yaw);
							var desiredTool = new Transform3(new Vec3(point[0] / 1000, point[1] / 1000, point[2] / 1000),
								new Quat(pose.qx, pose.qy, pose.qz, pose.qw));
							var target = inverse.compose(desiredTool.compose(request.toolOffset.inverse()));
							for (pan in [-Math.PI / 2, 0.0, Math.PI / 2, Math.PI]) {
								if (alternative != null) break;
								for (elbow in [-Math.PI / 2, Math.PI / 2]) {
									if (alternative != null) break;
									for (wristSeed in [-Math.PI / 2, Math.PI / 2]) {
										var attempt = arm.solve(target, [pan, -Math.PI / 2, elbow, -elbow, wristSeed, 0.0], options);
									if (attempt.converged && withinMargins(arm, attempt.q) && clearance.violation(attempt.q) == null &&
										pointClearance.violation(attempt.q, contact) == null) {
										alternative = attempt.q; alternativeTarget = pose; alternativeYaw = yaw; break;
										}
									}
								}
							}
						}
						if (alternative != null && alternativeTarget != null) {
							solved[i] = alternative; targets[i] = alternativeTarget;
							yawsSolved[i] = alternativeYaw;
						} else {
							failedAt = request.name + "@" + request.tool;
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
						? '$height mm, base Y ${baseOffset} mm at ready pose: $failedReason'
						: '$height mm, base Y ${baseOffset} mm at $failedAt: $failedReason';
					if (firstClassFailure == null) firstClassFailure = failure;
					lastClassFailure = failure;
					if (height == 550) riserFailures.push(failure);
				}
			}
			}
			if (firstClassFailure != null && lastClassFailure != null)
				failures.push('$cls first ${firstClassFailure}, 550 mm candidates [${riserFailures.join("; ")}], last ${lastClassFailure}');
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
