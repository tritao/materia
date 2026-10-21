package machinekit.milling;

import machinekit.assembly.AxisBuilder;
import machinekit.component.ComponentValues;
import machinekit.motion.*;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Mechanical Z assembly with one real prismatic joint and a coupled screw.
 * Includes the stationary braked motor, integrated fixed housing, coupling and
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
		var upper = new MillZDriveAdapter(new ComponentValues().setNumber("motorOffset",l.motorFace-l.upperBottom).setNumber("stemHeight",Math.max(60,114-(l.motorFace-l.upperBottom))).setNumber("width",width).setNumber("spacing",f.zRailSpacing).setNumber("thickness",l.mountThickness));
		place("fixedMount", upper, F.translation(0, f.columnFront - l.mountThickness / 2, l.upperBottom));
		var lower = new MillZBearingPlate(new ComponentValues().setNumber("bearingPitch",2*l.drive.floatingBoltX).setNumber("bearingHole",5).setNumber("bearingTapDepth",15).setNumber("width", width)
			.setNumber("height", l.lowerHeight).setNumber("thickness", l.mountThickness)
			.setBoolean("guideRelief",l.crossDriveZ!=null).setNumber("guideSpacing",f.yRailSpacing).setNumber("guideWidth",l.block.spec.railWidth)
			.setNumber("crossShaftZ", l.crossDriveZ == null ? 0 : l.crossDriveZ - f.railPlane)
			.setNumber("bearingZ", l.lowerHeight / 2).setNumber("anchorZ", 10).setNumber("anchorSpacing", f.zRailSpacing));
		attach("floatingMount", lower, F.translation(0, f.columnFront - l.mountThickness / 2, f.railPlane), "fixedMount");
		attach("motorMount", l.fixed, down(0,f.columnFront,l.motorFace),"fixedMount");
		attach("floatingBearing", l.floating, down(0,f.columnFront,l.floatingZ+l.drive.floatingBoltZ),"floatingMount");
		for(side in [-1,1]) {
			attach('floatingBolt$side',SocketHeadCapScrew.metric("M6",45),F.fromRotationMatrix(side*l.drive.floatingBoltX,f.columnFront+l.drive.floatingBoltSeat,l.floatingZ,[1,0,0,0,0,1,0,-1,0]),"floatingMount");
		}
		var mounting=l.motor.mounting();if(mounting==null)throw "Z motor needs catalog mounting data";
		attach("motor",l.motor,down(0,l.screwY,l.motorFace),"motorMount");
		for(p in mounting.holes)attach('motorBolt${p.x}-${p.y}',SocketHeadCapScrew.metric("M6",16),
			F.translation(-p.x,l.screwY+p.y,l.motorFace+mounting.flangeThickness),"motorMount");
		for(i in 0...l.fixed.spec.baseHoles.length){var h=l.fixed.spec.baseHoles[i];
			attach('mountBolt$i',SocketHeadCapScrew.metric("M6",45),
			F.fromRotationMatrix(-h.x,f.columnFront+l.drive.fixedBoltSeat,l.motorFace-h.z,[1,0,0, 0,0,1, 0,-1,0]),"motorMount");}
		place("screw", l.screw, down(0, l.screwY, l.screwTop));
		connect("motorMount", "screw");
		addMateOnAxis("screw-turn", "continuous", "motorMount", "to-screw", "screw", "attach-screw",
			{x: 0, y: 0, z: -1});
		attach("motorShaft", l.motor.shaft(), down(0, l.screwY, l.motorFace), "screw");
		attach("coupling", l.coupling, down(0, l.screwY, l.motorFace-l.drive.couplingSeat), "screw");

		var railPose = F.fromRotationMatrix(0, 0, 0, [-1,0,0, 0,-1,0, 0,0,1]);
		for (placed in MillRailPlacement.z(l))
			attach(placed.memberId,placed.rail,placed.pose,"fixedMount");
		carriage = new MillZCarriage(new ComponentValues().setNumber("width", l.carriageWidth).setNumber("height", l.carriageHeight)
			.setToken("nutModel",l.nut.spec.model).setNumber("thickness", l.carriageThickness).setNumber("nutReach", l.screwY - l.carriageBack)
			.setNumber("nutCentreZ", l.carriageHeight / 2).setNumber("nutPlate", l.nutPlateThickness)
			.setNumber("headBoltX", l.headValues.number("boltX")).setNumber("headBoltZ", l.headValues.get("boltZ") == null ? 42 : l.headValues.number("boltZ"))
			.setNumber("railSpacing", f.zRailSpacing).setNumber("blockSpan", l.blockSpan).setToken("profile", l.profile));
		var blockPoses:Array<AssemblyFrame> = [];
		for (side in [-1, 1]) for (span in [-l.blockSpan, l.blockSpan]) blockPoses.push(
			F.compose(F.translation(side * f.zRailSpacing / 2, f.columnFront - l.block.spec.railHeight, l.headCentre + span), railPose));
		slide({id: "z", lower: l.lower, upper: 0, initial: 0}, "railLeft", "blockLeftLower", l.block, blockPoses[0],
			{x: 0, y: 0, z: 1}, blockPoses.slice(1), l.overtravel);
		attach("carriage", carriage, F.translation(0, l.carriageBack, l.headCentre - l.carriageHeight / 2), "blockLeftLower");
		var names = ["blockLeftLower", "blockLeftUpper", "blockRightLower", "blockRightUpper"];
		for (i in 1...names.length) attach(names[i], l.block, blockPoses[i], "carriage");
		attach("nut", l.nut, down(0, l.screwY, l.headCentre + l.nut.bodyLength / 2), "carriage");
		head = new MillZHead(l.headValues);
		attach("head", head, F.translation(0, l.headBack, l.headBottom), "carriage");
		for (side in [-1, 1]) for (row in [-1, 1]) attach('headBolt$side$row', SocketHeadCapScrew.metric("M6", l.headBoltLength),
			F.fromRotationMatrix(side * head.parameterNumber("boltX"), l.headBack - head.parameterNumber("rearPlate") + MillZHead.COUNTERBORE_DEPTH,
				l.headCentre + row * head.parameterNumber("boltZ"),
				[1,0,0, 0,0,-1, 0,1,0]), "head");
		addCoupling("z-lead","z","screw-turn",-2*Math.PI/l.screw.thread.signedLead());
		// Supplier bearing stiffness/preload is not published: kinematic coupling only.
		exposeConnector("mount", "fixedMount", "mount");
		exposeConnector("head", "head", "mount");
		exposeConnector("spindle", "head", "spindle");
	}

	public static function isTranslating(id:String):Bool
		return id == "carriage" || id == "head" || id == "nut" || StringTools.startsWith(id, "block") || StringTools.startsWith(id, "headBolt");

	static function down(x:Float, y:Float, z:Float):AssemblyFrame
		return F.fromRotationMatrix(x, y, z, [-1,0,0, 0,1,0, 0,0,-1]);
}
