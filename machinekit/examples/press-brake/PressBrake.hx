import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.Transmission;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.motion.BallScrew;
import machinekit.motion.BallNut;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.LinearScale;
import machinekit.motion.MotorDriver;
import machinekit.motion.PowerSupply;
import machinekit.motion.ServoMotor;
import machinekit.motion.ScrewSupportUnit;
import machinekit.pressbrake.HydraulicCylinder;
import machinekit.pressbrake.GaugeFinger;
import machinekit.pressbrake.HydraulicSupply;
import machinekit.pressbrake.HydraulicValve;
import machinekit.pressbrake.HydraulicManifold;
import machinekit.fluid.CylinderPhaseSpeeds;
import machinekit.pressbrake.PressBrakeProfile;
import machinekit.pressbrake.ToolClamp;
import machinekit.pressbrake.ToolLayout;
import machinekit.pressbrake.ToolRow;
import machinekit.pressbrake.ToolSegment;
import machinekit.pressbrake.ToolStation;
import machinekit.pressbrake.ToolingCatalogue;
import machinekit.pressbrake.planning.PressGeometrySource;
import machinekit.pressbrake.planning.PressGeometryDatums;
import machinekit.pressbrake.planning.PressToolDatum;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinition.AssemblyVector;

/** An assumed fabricated block. Its lower face is z=0 and its section is centred. */
class PressBlock extends MachineComponent {
	public final width:Float;
	public final depth:Float;
	public final height:Float;
	public function new(name:String, width:Float, depth:Float, height:Float) {
		for (value in [width, depth, height]) if (!Math.isFinite(value) || value <= 0) throw "Invalid press block dimensions";
		super('$name-${Dimension.format(width)}x${Dimension.format(depth)}x${Dimension.format(height)}', name, "steel", true);
		this.width = width; this.depth = depth; this.height = height;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Solids.named(Part.box(width, depth, height), "body");
}

/** Side plate with its throat open toward -Y; the punch line is y=0. */
class PressSideFrame extends MachineComponent {
	public final thickness:Float;
	public final throat:Float;
	public final bedHeight:Float;
	public final openHeight:Float;
	public function new(thickness:Float, throat:Float, bedHeight:Float, openHeight:Float) {
		for (value in [thickness, throat, bedHeight, openHeight])
			if (!Math.isFinite(value) || value <= 0) throw "Invalid press side-frame dimensions";
		if (throat >= 500 || bedHeight + openHeight >= 1600) throw "Press side-frame opening consumes its rear or upper web";
		super('PRESS-SIDE-FRAME-${Dimension.format(thickness)}-${Dimension.format(throat)}-${Dimension.format(bedHeight)}-${Dimension.format(openHeight)}',
			"Assumed WC67Y-style side plate", "steel", true);
		this.thickness = thickness; this.throat = throat; this.bedHeight = bedHeight; this.openHeight = openHeight;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("throat", Face, Solids.axial(0, throat, bedHeight));
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.building([], parts -> {
			parts.push(Solids.named(Part.box(thickness, 600, 1600).translated(new Vector(0, 200, 0)), "side-plate"));
			parts.push(Solids.named(Part.box(thickness + 2, throat + 101, openHeight)
				.translated(new Vector(0, (throat - 101) / 2, bedHeight)), "throat-opening"));
			return Solids.cut(parts[0], [parts[1]]);
		});
}

/** Reference light-curtain receiver envelope and its clear-field signal. No safety rating is inferred. */
class PressLightCurtain extends MachineComponent {
	public function new() {
		super("PRESS-LIGHT-CURTAIN", "Assumed light curtain interface", "aluminium 6061", true);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "clear", kind: Signal, role: Supply, iface: Plug("generic", 3), required: false});
	}
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part return Solids.named(Part.box(30, 30, 600), "receiver");
}

/** Reference 40 t torsion-bar press. X is along the bed, Y toward the backgauge, Z upward.
 * Joint Y1/Y2 values are downward ram travel; backgauge X is distance behind the punch line.
 */
class PressBrake extends MachineAssembly implements PressGeometrySource {
	public final profile:PressBrakeProfile = StopControlled;
	public final bedLength:Float = 1600;
	public final bedHeight:Float = 800;
	public final openHeight:Float = 320;
	public final stroke:Float = 100;
	public final throat:Float = 230;
	public final capacityN:Float = 40 * 1000 * 9.80665;
	public final gaugeWidth:Float = 800;
	public final fingerHalfSpacing:Float;
	public final gaugeTravel:Float = 500;
	public final gaugeRTravel:Float = 120;
	public final toolLayout:ToolLayout;
	public final cylinder:HydraulicCylinder;
	public final phaseSpeeds:CylinderPhaseSpeeds;
	public final supplyPressurePa:Float;
	public final withR:Bool;
	public final gaugeBaseHeight:Float;
	final poses:Map<String, AssemblyFrame> = [];
	final toolDatums:Array<PressToolDatum> = [];

	public function pressAssembly():MachineAssembly return this;
	public function pressProfile():PressBrakeProfile return profile;
	public function pressCapacityN():Float return capacityN;
	public function pressGeometryDatums():PressGeometryDatums return {
		bed: "bed", ram: "ram", throat: "frameLeft", fingers: ["fingerLeft", "fingerRight"],
		ramJoint: "Y1", gaugeXJoint: "X", gaugeRJoint: withR ? "R" : null, tools: toolDatums.copy()
	};

	public function new(withR:Bool = true, ?layout:ToolLayout, ?phases:CylinderPhaseSpeeds, fingerHalfSpacing:Float = 300) {
		super();
		this.withR = withR;
		if (!Math.isFinite(fingerHalfSpacing) || fingerHalfSpacing <= GaugeFinger.WIDTH_MM / 2
			|| fingerHalfSpacing > (gaugeWidth - GaugeFinger.WIDTH_MM) / 2)
			throw "Manual finger spacing must separate both fingers and keep them on the backgauge carrier";
		this.fingerHalfSpacing = fingerHalfSpacing;
		cylinder = new HydraulicCylinder(160, 100, stroke);
		// Reference setup: 80 mm open tool gap, 1.5 mm blank and 10 mm approach clearance.
		// Other setups supply their own explicit change travel; thickness is never inferred.
		phaseSpeeds = phases == null ? new CylinderPhaseSpeeds(stroke, 80, 8, 70, 68.5) : phases;
		if (Math.abs(phaseSpeeds.stroke - stroke) > 1e-6) throw "Press flow setup must match the ram stroke";
		supplyPressurePa = capacityN / (2 * cylinder.pistonArea());
		var punch = ToolingCatalogue.straight(), die = ToolingCatalogue.dies()[1];
		var punchRow = new ToolRow([new ToolSegment(Punch(punch), 100), new ToolSegment(Punch(punch), 50)], [0]);
		var dieRow = new ToolRow([new ToolSegment(SingleV(die), 200)], []);
		toolLayout = layout == null ? new ToolLayout(bedLength, [new ToolStation("V12", punchRow, dieRow, (bedLength - 200) / 2)]) : layout;
		if (toolLayout.bedLength != bedLength || toolLayout.stations().length == 0) throw "Reference press needs installed tooling on its 1600 mm bed";
		var minimumDie = Math.POSITIVE_INFINITY;
		for (station in toolLayout.stations()) minimumDie = Math.min(minimumDie, machinekit.pressbrake.ToolProfiles.height(station.dies.profile));
		gaugeBaseHeight = bedHeight + minimumDie;
		place("bed", new PressBlock("Press bed", bedLength, 180, 200), at(0, 0, bedHeight - 200));
		addMemberConnector("bed", "left", at(-bedLength / 2, 0, 200));
		addMemberConnector("bed", "right", at(bedLength / 2, 0, 200));
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right", guide = 'guide$name';
			var x = side * (bedLength / 2 + 25);
			fixed('frame$name', new PressSideFrame(50, throat, bedHeight, openHeight), at(x, 0, 0), "bed");
			slide(side < 0 ? "Y1" : "Y2", guide, new PressBlock("Ram guide", 40, 100, 160),
				at(x, -160, bedHeight + openHeight), 'frame$name', {x: 0, y: 0, z: -1}, stroke, at(side * 650, 0, bedHeight + openHeight + 220));
			fixed('scale$name', new LinearScale(stroke + 40, 1000),
				at(x, 180, bedHeight + openHeight - stroke), 'frame$name');
			addEncoder('ram$name', side < 0 ? "Y1" : "Y2", 'scale$name');
			var tip = bedHeight + openHeight + 220;
			var bodyPose:AssemblyFrame = {x: side * 650, y: 0, z: tip + cylinder.bodyLength + cylinder.stemLength,
				qx: 1, qy: 0, qz: 0, qw: 0};
			fixed('cylinder$name', side < 0 ? cylinder : new HydraulicCylinder(160, 100, stroke), bodyPose, 'frame$name');
			fixed('rod$name', cylinder.movingRod(), {x: side * 650, y: 0, z: tip, qx: 1, qy: 0, qz: 0, qw: 0}, guide);
		}
		addCoupling("torsion-bar", "Y1", "Y2", 1);
		fixed("hydraulicSupply", new HydraulicSupply(supplyPressurePa), at(500, 350, 500), "bed");
		fixed("ramValve", new HydraulicValve(), at(500, 350, 740), "bed");
		var hose:machinekit.component.BomItem = {partNumber: "ASSUMED-HYDRAULIC-G1/2-HOSE",
			description: "Assumed G1/2 hydraulic hose assembly", quantity: 1, material: null};
		connectPorts("hydraulic-feed", "hydraulicSupply", "P", "ramValve", "P", hose);
		connectPorts("hydraulic-return", "ramValve", "T", "hydraulicSupply", "T", hose);
		for (chamber in ["A", "B"]) {
			fixed('ramDistribution$chamber', new HydraulicManifold(2), at(chamber == "A" ? 400 : 650, 350, 740), "bed");
			connectPorts('ramDistribution$chamber-feed', "ramValve", chamber, 'ramDistribution$chamber', "input", hose);
		}
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right";
			var outlet = side < 0 ? "out1" : "out2";
			connectPorts('ram$name-A', "ramDistributionA", outlet, 'cylinder$name', "A", hose);
			connectPorts('ram$name-B', "ramDistributionB", outlet, 'cylinder$name', "B", hose);
			addCylinder('hydraulic$name', side < 0 ? "Y1" : "Y2", 'cylinder$name', "ramValve", phaseSpeeds);
		}
		// Assumed 0.1 mm switch windows and release hysteresis; coordinates are measured ram travel.
		addSwitch("ram-top", "Y1", {lower: -0.1, upper: 0.1, velocity: null, effort: null}, 0.1);
		addSwitch("ram-speed-change", "Y1", {lower: phaseSpeeds.changePoint, upper: stroke,
			velocity: null, effort: null}, 0.1);
		addSwitch("ram-bottom", "Y1", {lower: stroke - 0.1, upper: stroke + 0.1,
			velocity: null, effort: null}, 0.1);
		fixed("ram", new PressBlock("Ram beam", bedLength, 100, 220), at(0, 0, bedHeight + openHeight), "guideLeft");
		for (side in [-1, 1]) fixed(side < 0 ? "ramBracketLeft" : "ramBracketRight",
			new PressBlock("Ram guide bracket", 160, 180, 80), at(side * 750, -80, bedHeight + openHeight + 80), "ram");
		// The torsion-bar coupling owns the equality of both guide coordinates. Adding
		// the same relation as a rigid closure would incorrectly introduce a solved input.
		addMemberConnector("ram", "tool-mount", at(0, 0, 0));
		fixed("torsionBar", new PressBlock("Torsion bar", bedLength + 50, 30, 30), at(0, -160, bedHeight + openHeight + 100), "ram");
		installTools();
		for (side in [-1, 1]) {
			fixed(side < 0 ? "supportLeft" : "supportRight", new PressBlock("Front sheet support", 80, 300, 20),
				at(side * 450, -240, bedHeight - 20), "bed");
			fixed(side < 0 ? "curtainLeft" : "curtainRight", new PressLightCurtain(), at(side * 900, -300, 600), "bed");
		}
		fixed("power", new PowerSupply(48, 20, 3), at(0, 420, 500), "bed");
		// The stop is independent of the ram. Its face descends by the commanded stopping travel.
		slide("depth", "depthStop", new PressBlock("Mechanical depth stop", 100, 80, 40),
			at(0, 300, bedHeight + openHeight), "bed", {x: 0, y: 0, z: -1}, stroke);
		servoScrew("depth", "depthStop", "depth", new BallScrew(300), at(0, 300, bedHeight + openHeight - 180),
			at(0, 300, bedHeight + openHeight - 20), {x: 0, y: 0, z: -1}, {x: 0, y: 0, z: 1}, "GENERIC-SERVO-50W", 1);
		slide("X", "gaugeX", new PressBlock("Backgauge carriage", gaugeWidth, 80, 60), at(0, 0, 660), "bed", {x: 0, y: 1, z: 0}, gaugeTravel);
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right", block = LinearRailBlock.metric("MGN12C");
			var railZ = 660 - (block.spec.blockHeight - block.spec.railHeight);
			var rotation = Math.PI / 4;
			var railPose:AssemblyFrame = {x: side * 200, y: 600, z: railZ, qx: Math.sin(rotation), qy: 0, qz: 0, qw: Math.cos(rotation)};
			fixed('gaugeRail$name', LinearRail.metric("MGN12C", 650), railPose, "bed");
			fixed('gaugeBlock$name', block, {x: side * 200, y: 0, z: railZ,
				qx: railPose.qx, qy: 0, qz: 0, qw: railPose.qw}, "gaugeX");
		}
		var alongY:AssemblyFrame = {x: 0, y: -70, z: 640, qx: -Math.sin(Math.PI / 4), qy: 0, qz: 0, qw: Math.cos(Math.PI / 4)};
		var nutY:AssemblyFrame = {x: 0, y: 10, z: 640, qx: alongY.qx, qy: 0, qz: 0, qw: alongY.qw};
		servoScrew("X", "gaugeX", "gaugeX", new BallScrew(650), alongY, nutY,
			{x: 0, y: 1, z: 0}, {x: 0, y: 1, z: 0}, "GENERIC-SERVO-200W", 2);
		var gaugeParent = "gaugeX";
		if (withR) {
			slide("R", "gaugeR", new PressBlock("Backgauge lift", gaugeWidth, 40, 40), at(0, 40, gaugeBaseHeight - 40),
				"gaugeX", {x: 0, y: 0, z: 1}, gaugeRTravel);
			for (side in [-1, 1]) {
				var name = side < 0 ? "Left" : "Right";
				fixed('gaugeRRail$name', LinearRail.metric("MGN12C", 250), at(side * 200, 40, 730), "gaugeX");
				fixed('gaugeRBlock$name', LinearRailBlock.metric("MGN12C"), at(side * 200, 40, gaugeBaseHeight - 50), "gaugeR");
			}
			servoScrew("R", "gaugeR", "gaugeR", new BallScrew(300), at(0, 60, gaugeBaseHeight - 130), at(0, 60, gaugeBaseHeight - 30),
				{x: 0, y: 0, z: 1}, {x: 0, y: 0, z: 1}, "GENERIC-SERVO-100W", 3, "gaugeX");
			gaugeParent = "gaugeR";
		}
		for (side in [-1, 1]) {
			var id = side < 0 ? "fingerLeft" : "fingerRight";
			place(id, new GaugeFinger(), at(side * fingerHalfSpacing, 0, gaugeBaseHeight));
			meeting(gaugeParent, id, at(side * fingerHalfSpacing, GaugeFinger.PIVOT_DEPTH_MM,
				gaugeBaseHeight - GaugeFinger.DATUM_HEIGHT_MM));
			var joint = '$id-hinge';
			addMateOnAxis(joint, "revolute", gaugeParent, 'to-$id', id, 'attach-$id',
				{x: -1, y: 0, z: 0}, 0,
				{lower: -GaugeFinger.DROP_ANGLE, upper: 0, velocity: null, effort: null,
					overtravel: GaugeFinger.OVERTRAVEL});
			setPassive(joint);
			setJointStop(joint, GaugeFinger.stopResponse());
			addSwitch('$id-contact', joint,
				{lower: -GaugeFinger.SWITCH_WINDOW, upper: 0,
					velocity: null, effort: null}, GaugeFinger.SWITCH_HYSTERESIS);
		}
	}

	function installTools():Void {
		for (station in toolLayout.stations()) {
			var firstDie:Null<String> = null, firstPunch:Null<String> = null;
			for (placement in station.dies.placements(station.offset - bedLength / 2)) {
				var id = '${station.id}-die-${Dimension.format(placement.start)}';
				if (firstDie == null) firstDie = id;
				fixed(id, placement.segment,
					at(placement.start, 0, bedHeight + machinekit.pressbrake.ToolProfiles.height(placement.segment.profile)), "bed");
			}
			for (placement in station.punches.placements(station.offset - bedLength / 2)) {
				var id = '${station.id}-punch-${Dimension.format(placement.start)}';
				if (firstPunch == null) firstPunch = id;
				var punchHeight = machinekit.pressbrake.ToolProfiles.height(placement.segment.profile);
				var dieHeight = machinekit.pressbrake.ToolProfiles.height(station.dies.profile);
				var holderHeight = openHeight - punchHeight - dieHeight - 80;
				if (holderHeight <= 0) throw "Tool stack leaves no space for the upper clamp";
				fixed('$id-clamp', new ToolClamp(Manual, placement.segment.length, 40, holderHeight),
					at(placement.start + placement.segment.length / 2, 0, bedHeight + openHeight - holderHeight), "ram");
				fixed(id, placement.segment, at(placement.start, 0, bedHeight + dieHeight + 80), '$id-clamp');
			}
			if (firstPunch == null || firstDie == null) throw 'Station "${station.id}" has no physical contact members';
			toolDatums.push({station: station.id, punch: firstPunch, die: firstDie});
		}
	}

	function servoScrew(joint:String, carriage:String, id:String, screw:BallScrew, screwPose:AssemblyFrame,
			nutPose:AssemblyFrame, axis:AssemblyVector, along:AssemblyVector, motorName:String, outlet:Int, parent:String = "bed"):Void {
		var motor = ServoMotor.model(motorName);
		var motorPose = {x: screwPose.x, y: screwPose.y, z: screwPose.z,
			qx: screwPose.qx, qy: screwPose.qy, qz: screwPose.qz, qw: screwPose.qw};
		fixed('$id-motor', motor, motorPose, parent);
		place('$id-screw', screw, screwPose);
		meeting('$id-motor', '$id-screw');
		addMateOnAxis('$id-turn', "continuous", '$id-motor', 'to-$id-screw', '$id-screw', 'attach-$id-screw', along);
		fixed('$id-nut', new BallNut(), nutPose, carriage);
		var near = ScrewSupportUnit.bk12(), far = ScrewSupportUnit.bf12();
		var nearPosition = BallScrew.INPUT_JOURNAL_LENGTH / 2;
		var farPosition = screw.totalLength - BallScrew.JOURNAL_LENGTH / 2;
		fixed('$id-supportNear', near, AssemblyFrames.compose(screwPose, at(0, 0, nearPosition)), parent);
		fixed('$id-supportFar', far, AssemblyFrames.compose(screwPose, at(0, 0, farPosition)), parent);
		var sense = machinekit.assembly.Sense.SenseTools.fromAlignment(axis.x * along.x + axis.y * along.y + axis.z * along.z);
		var ratio = addTransmission('$id-lead', joint, '$id-turn', Transmission.LeadScrew('$id-screw', '$id-nut'), sense);
		var speedCap = supportScrew('$id-lead', near.support, far.support, farPosition - nearPosition);
		fixed('$id-driver', new MotorDriver("GENERIC-SERVO-AMP", 3), at(-200 + outlet * 100, 420, 600), "bed");
		connectPorts('$id-power', "power", 'power$outlet', '$id-driver', "power");
		addMotor('$id-drive', '$id-turn', '$id-motor', '$id-driver');
	}

	static function at(x:Float, y:Float, z:Float):AssemblyFrame return AssemblyFrames.translation(x, y, z);
	function place(id:String, part:MachineComponent, pose:AssemblyFrame):Void {
		addComponent(id, part, pose); poses.set(id, pose);
	}
	function fixed(id:String, part:MachineComponent, pose:AssemblyFrame, parent:String):Void {
		place(id, part, pose); meeting(parent, id);
		addMate('$id-fixed', "fixed", parent, 'to-$id', id, 'attach-$id');
	}
	function slide(joint:String, id:String, part:MachineComponent, pose:AssemblyFrame, parent:String, axis:AssemblyVector, travel:Float,
			?datum:AssemblyFrame):Void {
		place(id, part, pose); meeting(parent, id, datum);
		addMateOnAxis(joint, "prismatic", parent, 'to-$id', id, 'attach-$id', axis, 0,
			{lower: 0, upper: travel, velocity: null, effort: null, overtravel: 1});
	}
	function pose(id:String):AssemblyFrame {
		var result = poses.get(id);
		if (result == null) throw 'Press has no member "$id"';
		return result;
	}
	function meeting(parent:String, child:String, ?datum:AssemblyFrame):Void {
		var p = pose(child), meeting = datum == null ? at(p.x, p.y, p.z) : datum;
		addMemberConnector(parent, 'to-$child', AssemblyFrames.compose(AssemblyFrames.inverse(pose(parent)), meeting));
		addMemberConnector(child, 'attach-$child', AssemblyFrames.compose(AssemblyFrames.inverse(p), meeting));
	}
}
