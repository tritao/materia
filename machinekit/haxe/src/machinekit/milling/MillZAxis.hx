package machinekit.milling;

import machinekit.assembly.AxisBuilder;
import machinekit.assembly.Transmission;
import machinekit.component.ComponentValues;
import machinekit.motion.*;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Mechanical Z assembly with one real prismatic joint and a coupled screw.
 * Includes the stationary motor, brake, coupling, two supported bearings and
 * moving carriage/head. Electrical control and spindle payload attach outside.
 */
class MillZAxis extends AxisBuilder {
	public final layout:MillZAxisLayout;
	public final carriage:MillZCarriage;
	public final head:MillZHead;

	public function new(layout:MillZAxisLayout) {
		super();
		if (layout == null) throw "Z assembly needs a layout";
		this.layout = layout;
		var l = layout, f = l.frame, width = f.zRailSpacing + f.seatWidth;
		var upper = new MillZBearingPlate(new ComponentValues().setNumber("width", width)
			.setNumber("height", l.upperHeight).setNumber("thickness", l.mountThickness)
			.setNumber("bearingZ", l.fixedZ - l.upperBottom).setNumber("anchorZ", 10).setNumber("anchorSpacing", f.zRailSpacing));
		place("fixedMount", upper, F.translation(0, f.columnFront - l.mountThickness / 2, l.upperBottom));
		var lower = new MillZBearingPlate(new ComponentValues().setNumber("width", width)
			.setNumber("height", l.lowerHeight).setNumber("thickness", l.mountThickness)
			.setNumber("bearingZ", l.lowerHeight / 2).setNumber("anchorZ", 10).setNumber("anchorSpacing", f.zRailSpacing));
		attach("floatingMount", lower, F.translation(0, f.columnFront - l.mountThickness / 2, f.railPlane), "fixedMount");
		attach("fixedBearing", l.fixed, down(0, l.screwY, l.fixedZ), "fixedMount");
		attach("floatingBearing", l.floating, down(0, l.screwY, l.floatingZ), "floatingMount");
		var stand = new MillZMotorStand(new ComponentValues().setNumber("width", l.motor.rating.bodyDiameter + 20)
			.setNumber("depth", l.motor.rating.bodyDiameter + 20).setNumber("height", l.motorStandHeight)
			.setNumber("plate", l.motorPlateThickness).setNumber("shaftBore", l.motor.rating.shaftDiameter + 3)
			.setNumber("boltPitch", l.motor.rating.bodyDiameter / 2));
		attach("motorStand", stand, F.translation(0, l.screwY, l.motorStandBottom), "fixedMount");
		attach("motor", l.motor, down(0, l.screwY, l.motorFace), "motorStand");
		place("screw", l.screw, down(0, l.screwY, l.screwTop));
		connect("fixedBearing", "screw");
		addMateOnAxis("screw-turn", "continuous", "fixedBearing", "to-screw", "screw", "attach-screw",
			{x: 0, y: 0, z: -1});
		attach("motorShaft", l.motor.shaft(), down(0, l.screwY, l.motorFace), "screw");
		attach("coupling", l.coupling, down(0, l.screwY, l.screwTop + l.coupling.length / 2), "screw");
		var brakePose = down(0, l.screwY, l.motorFace - l.motorPlateThickness);
		for (role in ["front", "shell", "back"]) for (sector in 0...ShaftBrakePart.sectorCount(l.brake.bore, role))
			attach('brake-$role-$sector', new ShaftBrakePart(l.brake.bore, role, sector), brakePose, "motorStand");
		for (role in ["hub", "disc"]) attach('brake-$role', new ShaftBrakePart(l.brake.bore, role), brakePose, "screw");

		var railPose = F.fromRotationMatrix(0, 0, 0, [-1,0,0, 0,-1,0, 0,0,1]);
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right";
			var pose = F.compose(F.translation(side * f.zRailSpacing / 2, f.columnFront - l.block.spec.railHeight,
				f.railPlane + f.zRailStart), railPose);
			attach('rail$name', LinearRail.metric(l.profile, f.zRailLength), pose, "fixedMount");
		}
		carriage = new MillZCarriage(new ComponentValues().setNumber("width", l.carriageWidth).setNumber("height", l.carriageHeight)
			.setNumber("thickness", l.carriageThickness).setNumber("nutReach", l.screwY - l.carriageBack)
			.setNumber("nutCentreZ", l.carriageHeight / 2).setNumber("nutPlate", l.nutPlateThickness)
			.setNumber("headBoltX", l.carriageWidth / 2 - 12)
			.setNumber("railSpacing", f.zRailSpacing).setNumber("blockSpan", l.blockSpan).setToken("profile", l.profile));
		var blockPoses:Array<AssemblyFrame> = [];
		for (side in [-1, 1]) for (span in [-l.blockSpan, l.blockSpan]) blockPoses.push(
			F.compose(F.translation(side * f.zRailSpacing / 2, f.columnFront - l.block.spec.railHeight, l.headCentre + span), railPose));
		slide({id: "z", lower: l.lower, upper: 0, initial: 0}, "railLeft", "blockLeftLower", l.block, blockPoses[0],
			{x: 0, y: 0, z: 1}, blockPoses.slice(1));
		attach("carriage", carriage, F.translation(0, l.carriageBack, l.headCentre - l.carriageHeight / 2), "blockLeftLower");
		var names = ["blockLeftLower", "blockLeftUpper", "blockRightLower", "blockRightUpper"];
		for (i in 1...names.length) attach(names[i], l.block, blockPoses[i], "carriage");
		attach("nut", l.nut, down(0, l.screwY, l.headCentre + l.nut.bodyLength / 2), "carriage");
		head = new MillZHead(new ComponentValues().setNumber("width", 180).setNumber("height", l.headHeight)
			.setNumber("rearWidth", l.carriageWidth).setNumber("boltX", l.carriageWidth / 2 - 12)
			.setNumber("reach", l.headBack - l.spindleY + SpindleCartridge.DIAMETER / 2 + 10)
			.setNumber("spindleReach", l.headBack - l.spindleY));
		attach("head", head, F.translation(0, l.headBack, l.headBottom), "carriage");
		for (side in [-1, 1]) for (row in [-1, 1]) attach('headBolt$side$row', SocketHeadCapScrew.metric("M6", 16),
			F.fromRotationMatrix(side * head.parameterNumber("boltX"), l.headBack - head.parameterNumber("rearPlate") + MillZHead.COUNTERBORE_DEPTH,
				l.headCentre + row * head.parameterNumber("boltZ"),
				[1,0,0, 0,0,-1, 0,1,0]), "head");
		addTransmission("z-lead", "z", "screw-turn", Transmission.LeadScrew("screw", "nut"), screwSense("z", [0.0, 0.0, -1.0]));
		supportScrew("z-lead", l.fixed.support, l.floating.support);
		exposeConnector("mount", "fixedMount", "mount");
		exposeConnector("head", "head", "mount");
		exposeConnector("spindle", "head", "spindle");
	}

	public static function isTranslating(id:String):Bool
		return id == "carriage" || id == "head" || id == "nut" || StringTools.startsWith(id, "block") || StringTools.startsWith(id, "headBolt");

	static function down(x:Float, y:Float, z:Float):AssemblyFrame
		return F.fromRotationMatrix(x, y, z, [-1,0,0, 0,1,0, 0,0,-1]);
}
