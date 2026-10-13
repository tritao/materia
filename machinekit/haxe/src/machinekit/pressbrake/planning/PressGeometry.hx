package machinekit.pressbrake.planning;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.pressbrake.PressBrakeProfile;
import machinekit.pressbrake.ToolLayout;
import machinekit.pressbrake.ToolStation;
import machinekit.pressbrake.ToolProfiles;
import cadkit.sheetmetal.SheetDiagnosis;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Rebuilt plain-number press geometry. The assembly remains the owner of physical facts. */
class PressGeometry {
	public final bedLength:Float;
	public final bedTopHeight:Float;
	public final openHeight:Float;
	public final throatY:Float;
	public final capacityN:Float;
	public final profile:PressBrakeProfile;
	public final installedTools:ToolLayout;
	public final ramEnvelope:PressEnvelope;
	public final ramTravel:PressTravel;
	public final backgaugeX:PressTravel;
	public final backgaugeR:PressTravel;
	final toolStations:Array<PressStationGeometry>;
	final fingerShapes:Array<PressEnvelope>;
	final contactDatums:Array<PressFingerDatum>;
	final otherVolumes:Array<PressMachineMember>;

	function new(bedLength:Float, bedTopHeight:Float, openHeight:Float, throatY:Float, capacityN:Float,
			profile:PressBrakeProfile, installedTools:ToolLayout, ramEnvelope:PressEnvelope, ramTravel:PressTravel, backgaugeX:PressTravel,
			backgaugeR:PressTravel, stations:Array<PressStationGeometry>, fingers:Array<PressEnvelope>, contacts:Array<PressFingerDatum>,
			members:Array<PressMachineMember>) {
		if (profile == null || !(bedLength > 0 && openHeight > 0 && capacityN > 0) || !Math.isFinite(bedLength + bedTopHeight + openHeight + throatY + capacityN))
			throw "Invalid press geometry dimensions or capacity";
		this.bedLength = bedLength; this.bedTopHeight = bedTopHeight; this.openHeight = openHeight;
		this.throatY = throatY; this.capacityN = capacityN; this.profile = profile;
		this.ramEnvelope = ramEnvelope; this.ramTravel = ramTravel; this.backgaugeX = backgaugeX; this.backgaugeR = backgaugeR;
		this.installedTools = installedTools;
		if (installedTools == null || installedTools.bedLength != bedLength || installedTools.stations().length != stations.length)
			throw "Installed tooling disagrees with evaluated press stations";
		toolStations = stations.copy(); fingerShapes = fingers.copy(); contactDatums = contacts.copy();
		otherVolumes = members.copy();
		for (installed in installedTools.stations()) { var contact = station(installed.id); }
		for (i in 0...toolStations.length) for (j in 0...i)
			if (toolStations[i].id == toolStations[j].id) throw 'Duplicate press station "${toolStations[i].id}"';
	}
	public function stations():Array<PressStationGeometry> return toolStations.copy();
	public function fingers():Array<PressEnvelope> return fingerShapes.copy();
	public function fingerContacts():Array<PressFingerDatum> return contactDatums.copy();
	public function machineMembers():Array<PressMachineMember> return otherVolumes.copy();
	public function installedStation(id:String):ToolStation {
		for (station in installedTools.stations()) if (station.id == id) return station;
		throw new SheetDiagnosis("press.station-not-installed", "No installed tool station has the requested identity", [id]);
	}
	public function station(id:String):PressStationGeometry {
		for (station in toolStations) if (station.id == id) return station;
		throw 'Press geometry has no installed station "$id"';
	}

	/** Derive geometry at joint zero, regardless of the editor's current pose. */
	public static function of(source:PressGeometrySource):PressGeometry {
		if (source == null) throw "Press geometry needs an assembly source";
		var assembly = source.pressAssembly(), datums = source.pressGeometryDatums();
		if (assembly == null || datums == null || datums.tools == null || datums.tools.length == 0
			|| datums.fingers == null || datums.fingers.length == 0) throw "Press geometry needs installed tooling and finger datums";
		var model = new AssemblyModel("mm"); assembly.addTo(model, "");
		var state = new AssemblyState(model.definition("press-geometry"));
		state.setJoint(datums.ramJoint, 0); state.setJoint(datums.gaugeXJoint, 0);
		if (datums.gaugeRJoint != null) state.setJoint(datums.gaugeRJoint, 0);
		state.forwardKinematics();
		var left = state.worldConnector(datums.bed, "left"), right = state.worldConnector(datums.bed, "right");
		if (Math.abs(left.y - right.y) > 1e-6 || Math.abs(left.z - right.z) > 1e-6)
			throw "Press bed endpoints are not aligned with the press X axis";
		var ramMount = state.worldConnector(datums.ram, "tool-mount");
		var throat = state.worldConnector(datums.throat, "throat");
		var stations:Array<PressStationGeometry> = [];
		for (tool in datums.tools) {
			var punch = state.worldConnector(tool.punch, "contact"), die = state.worldConnector(tool.die, "contact");
			if (Math.abs(punch.y - die.y) > 1e-6) throw 'Station "${tool.station}" punch and die contact lines disagree';
			stations.push(new PressStationGeometry(tool.station, die.z, punch.y, punch.z));
		}
		var fingers:Array<PressEnvelope> = [];
		var contacts:Array<PressFingerDatum> = [];
		for (id in datums.fingers) {
			fingers.push(envelope(assembly, state, id));
			var point = state.worldConnector(id, "contact");
			contacts.push(new PressFingerDatum(id, point.x, point.y, point.z));
		}
		var members:Array<PressMachineMember> = [];
		for (frame in datums.frames) {
			var pose = state.worldPose(frame.member), pieces:Array<ConvexPrism> = [];
			for (box in frame.boxes) {
				var polygon = [point(pose, box.minX, box.minY, box.minZ), point(pose, box.maxX, box.minY, box.minZ),
					point(pose, box.maxX, box.maxY, box.minZ), point(pose, box.minX, box.maxY, box.minZ)];
				pieces.push(new ConvexPrism(frame.member + ":volume:" + pieces.length, polygon,
					point(pose, box.minX, box.minY, box.maxZ).subtract(polygon[0])));
			}
			members.push(new PressMachineMember(frame.member, Fixed, pieces));
		}
		memberBoxes(members, assembly, state, datums.fixedMembers, Fixed);
		memberBoxes(members, assembly, state, datums.ramMembers, Ram);
		memberBoxes(members, assembly, state, datums.gaugeXMembers, BackgaugeX);
		memberBoxes(members, assembly, state, datums.gaugeXRMembers, BackgaugeXR);
		return new PressGeometry(Math.abs(right.x - left.x), left.z, ramMount.z - left.z, throat.y,
			source.pressCapacityN(), source.pressProfile(), source.pressToolLayout(), envelope(assembly, state, datums.ram), travel(state, datums.ramJoint),
			travel(state, datums.gaugeXJoint), datums.gaugeRJoint == null ? new PressTravel(0, 0) : travel(state, datums.gaugeRJoint), stations, fingers, contacts, members);
	}
	static function point(pose:AssemblyFrame, x:Float, y:Float, z:Float):Vector {
		var value = AssemblyFrames.transformPoint(pose, x, y, z); return new Vector(value.x, value.y, value.z);
	}
	static function memberBoxes(result:Array<PressMachineMember>, assembly:MachineAssembly, state:AssemblyState,
			ids:Array<String>, motion:PressMemberMotion):Void {
		for (id in ids) result.push(new PressMachineMember(id, motion,
			PressCollisionGeometry.box(MachineMember(id), id, envelope(assembly, state, id), new Vector()).pieces()));
	}
	static function travel(state:AssemblyState, id:String):PressTravel {
		for (joint in state.definition.joints) if (joint.id == id) {
			if (joint.type != AssemblyJointType.Prismatic || joint.limits.lower == null || joint.limits.upper == null)
				throw 'Press joint "$id" needs finite prismatic travel limits';
			return new PressTravel(joint.limits.lower, joint.limits.upper);
		}
		throw 'Press geometry has no joint "$id"';
	}
	static function envelope(assembly:MachineAssembly, state:AssemblyState, id:String):PressEnvelope {
		for (entry in assembly.components()) if (entry.id == id) {
			var part = entry.component.geometry();
			try { var result = PressEnvelope.of(part, state.worldPose(id)); part.close(); return result; }
			catch (error:Dynamic) { part.close(); throw error; }
		}
		throw 'Press geometry has no member "$id"';
	}
	/** Independent reference fixture for the fixed WC67Y-class dimensions in the plan. */
	public static function reference(layout:ToolLayout, withR:Bool = true, fingerHalfSpacing:Float = 300):PressGeometry {
		if (layout == null || layout.bedLength != 1600 || layout.stations().length == 0)
			throw "Reference press geometry needs tooling on the 1600 mm bed";
		var stations:Array<PressStationGeometry> = [];
		var fingerTop = Math.POSITIVE_INFINITY;
		for (station in layout.stations()) {
			var dieTop = 800 + ToolProfiles.height(station.dies.profile);
			fingerTop = Math.min(fingerTop, dieTop);
			stations.push(new PressStationGeometry(station.id, dieTop, 0, dieTop + 80));
		}
		var members = [new PressMachineMember("bed", Fixed, PressCollisionGeometry.box(MachineMember("bed"), "bed",
			new PressEnvelope(-800, -90, 600, 800, 90, 800), new Vector()).pieces()),
			new PressMachineMember("gaugeX", BackgaugeX, PressCollisionGeometry.box(MachineMember("gaugeX"), "gaugeX",
				new PressEnvelope(-400, -40, 660, 400, 40, 720), new Vector()).pieces())];
		if (withR) members.push(new PressMachineMember("gaugeR", BackgaugeXR,
			PressCollisionGeometry.box(MachineMember("gaugeR"), "gaugeR",
				new PressEnvelope(-400, 20, fingerTop - 40, 400, 60, fingerTop), new Vector()).pieces()));
		for (side in [-1, 1]) {
			var id = side < 0 ? "frameLeft" : "frameRight", center = side * 825;
			var boxes = [new PressEnvelope(center - 25, -100, 0, center + 25, 230, 800),
				new PressEnvelope(center - 25, 230, 0, center + 25, 500, 1600),
				new PressEnvelope(center - 25, -100, 1120, center + 25, 230, 1600)];
			var pieces:Array<ConvexPrism> = [];
			for (box in boxes) for (piece in PressCollisionGeometry.box(MachineMember(id), id + ":volume:" + pieces.length, box, new Vector()).pieces()) pieces.push(piece);
			members.push(new PressMachineMember(id, Fixed, pieces));
		}
		return new PressGeometry(1600, 800, 320, 230, 40 * 1000 * 9.80665, StopControlled, layout,
			new PressEnvelope(-800, -50, 1120, 800, 50, 1340), new PressTravel(0, 100), new PressTravel(0, 500),
			new PressTravel(0, withR ? 120 : 0), stations,
			[new PressEnvelope(-fingerHalfSpacing - 10, 0, fingerTop - 20, -fingerHalfSpacing + 10, 60, fingerTop + 1),
				new PressEnvelope(fingerHalfSpacing - 10, 0, fingerTop - 20, fingerHalfSpacing + 10, 60, fingerTop + 1)],
			[new PressFingerDatum("fingerLeft", -fingerHalfSpacing, 0, fingerTop),
				new PressFingerDatum("fingerRight", fingerHalfSpacing, 0, fingerTop)], members);
	}
}
