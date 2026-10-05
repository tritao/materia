package machinekit.pressbrake.planning;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.MachineAssembly;
import machinekit.pressbrake.PressBrakeProfile;
import machinekit.pressbrake.ToolLayout;
import machinekit.pressbrake.ToolProfiles;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

/** Rebuilt plain-number press geometry. The assembly remains the owner of physical facts. */
class PressGeometry {
	public final bedLength:Float;
	public final bedTopHeight:Float;
	public final openHeight:Float;
	public final throatY:Float;
	public final capacityN:Float;
	public final profile:PressBrakeProfile;
	public final ramEnvelope:PressEnvelope;
	public final ramTravel:PressTravel;
	public final backgaugeX:PressTravel;
	public final backgaugeR:PressTravel;
	final toolStations:Array<PressStationGeometry>;
	final fingerShapes:Array<PressEnvelope>;

	function new(bedLength:Float, bedTopHeight:Float, openHeight:Float, throatY:Float, capacityN:Float,
			profile:PressBrakeProfile, ramEnvelope:PressEnvelope, ramTravel:PressTravel, backgaugeX:PressTravel,
			backgaugeR:PressTravel, stations:Array<PressStationGeometry>, fingers:Array<PressEnvelope>) {
		if (profile == null || !(bedLength > 0 && openHeight > 0 && capacityN > 0) || !Math.isFinite(bedLength + bedTopHeight + openHeight + throatY + capacityN))
			throw "Invalid press geometry dimensions or capacity";
		this.bedLength = bedLength; this.bedTopHeight = bedTopHeight; this.openHeight = openHeight;
		this.throatY = throatY; this.capacityN = capacityN; this.profile = profile;
		this.ramEnvelope = ramEnvelope; this.ramTravel = ramTravel; this.backgaugeX = backgaugeX; this.backgaugeR = backgaugeR;
		toolStations = stations.copy(); fingerShapes = fingers.copy();
		for (i in 0...toolStations.length) for (j in 0...i)
			if (toolStations[i].id == toolStations[j].id) throw 'Duplicate press station "${toolStations[i].id}"';
	}
	public function stations():Array<PressStationGeometry> return toolStations.copy();
	public function fingers():Array<PressEnvelope> return fingerShapes.copy();
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
		for (id in datums.fingers) fingers.push(envelope(assembly, state, id));
		return new PressGeometry(Math.abs(right.x - left.x), left.z, ramMount.z - left.z, throat.y,
			source.pressCapacityN(), source.pressProfile(), envelope(assembly, state, datums.ram), travel(state, datums.ramJoint),
			travel(state, datums.gaugeXJoint), datums.gaugeRJoint == null ? new PressTravel(0, 0) : travel(state, datums.gaugeRJoint), stations, fingers);
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
	public static function reference(layout:ToolLayout, withR:Bool = true):PressGeometry {
		if (layout == null || layout.bedLength != 1600 || layout.stations().length == 0)
			throw "Reference press geometry needs tooling on the 1600 mm bed";
		var stations:Array<PressStationGeometry> = [];
		var fingerTop = Math.POSITIVE_INFINITY;
		for (station in layout.stations()) {
			var dieTop = 800 + ToolProfiles.height(station.dies.profile);
			fingerTop = Math.min(fingerTop, dieTop);
			stations.push(new PressStationGeometry(station.id, dieTop, 0, dieTop + 80));
		}
		return new PressGeometry(1600, 800, 320, 230, 40 * 1000 * 9.80665, StopControlled,
			new PressEnvelope(-800, -50, 1120, 800, 50, 1340), new PressTravel(0, 100), new PressTravel(0, 500),
			new PressTravel(0, withR ? 120 : 0), stations,
			[new PressEnvelope(-310, 0, fingerTop - 20, -290, 60, fingerTop), new PressEnvelope(290, 0, fingerTop - 20, 310, 60, fingerTop)]);
	}
}
