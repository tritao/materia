import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import machinekit.milling.MillFrame;
import machinekit.milling.MillFrameLayout;
import machinekit.milling.MillZAxis;
import machinekit.milling.MillZAxisLayout;
import machinekit.milling.MillZAxisIntegrity;
import machinekit.milling.MillXYStage;
import machinekit.milling.MillXYStageIntegrity;
import machinekit.milling.MillXYStageLayout;
import machinekit.motion.LinearRail;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

/** Mechanical XYZ assembly on the casting.
 * The legacy BenchMill keeps its independent, working motion/control model.
 */
class MillFramePreview {
	public static function layout(reference:BenchMill):MillFrameLayout {
		var state:Map<String, AssemblyFrame> = reference.solvedPoses();
		var y:AssemblyFrame = state.get("railYLeft"), z:AssemblyFrame = state.get("railZLeft");
		var column:AssemblyFrame = state.get("column");
		var yRail:LinearRail = cast reference.components().filter(row -> row.id == "railYLeft")[0].component;
		var zRail:LinearRail = cast reference.components().filter(row -> row.id == "railZLeft")[0].component;
		var rise = stageRise(reference);
		return new MillFrameLayout({yRailSpacing: reference.base.railSpacing, yRailStart: y.y - 100, yRailLength: yRail.length + 80,
			columnFront: column.y - reference.column.depth / 2, columnHeight: reference.column.height + rise,
			zRailSpacing: Math.max(180, reference.column.railSpacing),
			zRailStart: z.z - reference.base.height, zRailLength: zRail.length + rise}, {baseWidth: 520, frontExtension: 40, footPockets: true});
	}

	public static function stageRise(reference:BenchMill):Float
		return MillXYStageLayout.stackHeight(BenchMill.RAIL) - (reference.tableTop - reference.base.height);

	public static function zLayout(reference:BenchMill, layout:MillFrameLayout, guideBlock:Null<machinekit.motion.LinearRailBlock> = null):MillZAxisLayout {
		var poses:Map<String, AssemblyFrame> = reference.solvedPoses();
		var head:AssemblyFrame = poses.get("head"), spindle:AssemblyFrame = poses.get("spindle");
		return new MillZAxisLayout(layout, head.z + layout.railPlane - reference.base.height + stageRise(reference),
			reference.head.height, spindle.y, reference.specs[2].lower, BenchMill.RAIL, guideBlock);
	}

	/** Both the axis-only study and full concept use this same mechanical assembly. */
	public static function structure(frame:MillFrame):MachineAssembly {
		if (frame.zAxis == null) throw "Z study needs a frame with drive interfaces";
		var axis = new MillZAxis(frame.zAxis), result = new MachineAssembly();
		result.include("frame", frame); result.include("zAxis", axis);
		var framePoses:Map<String, AssemblyFrame> = frame.solvedPoses();
		var plate = axis.zeroPose("fixedMount");
		result.addMemberConnector("frame/column/driveUpperLeft", "axisSeat",
			F.compose(F.inverse(framePoses.get("column/driveUpperLeft")), plate));
		result.addMemberConnector("zAxis/fixedMount", "frameSeat", F.identity());
		result.addMate("z-on-column", "fixed", "frame/column/driveUpperLeft", "axisSeat", "zAxis/fixedMount", "frameSeat");
		return result;
	}

	public static function concept(reference:BenchMill, frame:MillFrame):MachineAssembly {
		var result = structure(frame);
		var state:Map<String, AssemblyFrame> = reference.solvedPoses();
		var rise = frame.layout.railPlane - reference.base.height + stageRise(reference);
		var spindleParts = new MachineAssembly();
		var spindleIds:Array<String> = [];

		var spindle:AssemblyFrame = state.get("spindle");
		var spindleOrigin = F.translation(spindle.x, spindle.y, spindle.z + rise);
		for (member in reference.components()) {
			if (!spindlePart(member.id)) continue;
			var pose:AssemblyFrame = state.get(member.id);
			pose.z += rise;
			spindleParts.addComponent(member.id, member.component, pose);
			spindleParts.addMemberConnector(member.id, "studyOrigin", F.identity());
			spindleIds.push(member.id);
			if (member.id != "spindle") spindleParts.addMemberConnector("spindle", 'payload-${member.id}',
				F.compose(F.inverse(spindleOrigin), pose));
		}
		for (id in spindleIds) if (id != "spindle") spindleParts.addMate('payload-$id', "fixed", "spindle", 'payload-$id', id, "studyOrigin");
		result.include("spindle", spindleParts);
		if (frame.xyAxis == null) throw "Full mill concept needs XY steel drive seats";
		var xy = new MillXYStage(frame.xyAxis);
		result.include("xyStage", xy);
		var framePoses:Map<String, AssemblyFrame> = frame.solvedPoses();
		result.addMemberConnector("frame/base/driveYFixed", "stageSeat",
			F.compose(F.inverse(framePoses.get("base/driveYFixed")), xy.zeroPose("fixedPedestalY")));
		result.addMemberConnector("xyStage/fixedPedestalY", "frameSeat", F.identity());
		result.addMate("xy-on-base", "fixed", "frame/base/driveYFixed", "stageSeat", "xyStage/fixedPedestalY", "frameSeat");
		var axis = new MillZAxis(frame.zAxis);
		result.addMemberConnector("zAxis/head", "spindlePayload", F.compose(F.inverse(axis.zeroPose("head")), spindleOrigin));
		result.addMemberConnector("spindle/spindle", "headSeat", F.identity());
		result.addMate("spindle-on-head", "fixed", "zAxis/head", "spindlePayload", "spindle/spindle", "headSeat");
		return result;
	}

	static function spindlePart(id:String):Bool return ["spindle", "holder", "tool", "spindleMotor", "spindlePulley",
		"spindleMotorPulley", "spindleMotorShaft", "spindleBelt", "spindleMotorShelf", "spindleMotorRiser"].indexOf(id) >= 0;

	public static function main():Void {
		var reference = new BenchMill(false, false), l = layout(reference);
		var z = zLayout(reference, l), xy = new MillXYStageLayout(l, z.spindleY, BenchMill.RAIL), frame = new MillFrame(l, z, xy);
		MillZAxisIntegrity.check(new MillZAxis(z), frame).throwIfErrors();
		MillXYStageIntegrity.check(new MillXYStage(xy), frame).throwIfErrors();
		var root = Sys.getCwd(), directory = sys.FileSystem.exists(root + "/machinekit")
			? root + "/machinekit/examples/bench-mill/build" : root + "/build";
		if (!sys.FileSystem.exists(directory)) sys.FileSystem.createDirectory(directory);
		for (entry in [{name: "mill-frame", assembly: cast(frame, MachineAssembly)},
			{name: "mill-z-axis", assembly: structure(frame)}, {name: "mill-xy-stage", assembly: cast(new MillXYStage(xy), MachineAssembly)},
			{name: "mill-frame-concept", assembly: concept(reference, frame)}]) {
			var path = directory + "/" + entry.name + ".mtrg";
			sys.io.File.saveBytes(path, materia.project.SceneArtifact.encode(AssemblyPreview.scene(entry.assembly, entry.name)));
			Sys.println('Mill frame preview: $path');
		}
		var report = {status: "mechanical XYZ concept; not qualified for manufacture", units: "mm",
			travel: [{axis:"x",travel:2*xy.xHalfTravel},{axis:"y",travel:2*xy.yHalfTravel},{axis:"z",travel:-z.lower}],
			base: {width: l.baseWidth, depth: l.baseDepth, height: l.baseHeight, railPlane: l.railPlane},
			column: {width: l.columnWidth, depth: l.columnDepth, height: l.columnHeight, railFaceY: l.columnFront},
			xyFullStrokeStructuralClearance: "passed conservative Cartesian-product CAD swept-volume audit",
			zFullStrokeStructuralClearance: "passed conservative CAD swept-volume audit",
			xy: {tableTop: xy.tableTop, saddleHeight: MillXYStageLayout.SADDLE_HEIGHT, tableRibDepth: MillXYStageLayout.RISER_HEIGHT, xOvertravel: xy.xOvertravel, yOvertravel: xy.yOvertravel, yScrewZ: xy.yScrewZ, yMotorFace: xy.yMotorFace, yBayRoof: l.baseHeight - xy.yBayTop},
			z: {screwX: 0, screwY: z.screwY, guideSpacing: l.zRailSpacing, lower: z.lower, upper: 0},
			castingMassKg: {base: frame.base.castingByWeight().kg, column: frame.column.castingByWeight().kg},
			pending: ["ATC spindle/drawbar/actuator selection and head redesign", "guide/block drilling, vendor mounting patterns and installed fasteners", "insert anchorage and strength analysis",
				"electrical brake and motor controls", "chip drainage and enclosure", "lifting and mold design", "material qualification and static/modal analysis"]};
		MillDesignStudy.write(concept(reference, frame), frame, directory);
		sys.io.File.saveContent(directory + "/mill-frame-concept.json", haxe.Json.stringify(report, null, "  "));
		Sys.println(haxe.Json.stringify(report));
	}
}
