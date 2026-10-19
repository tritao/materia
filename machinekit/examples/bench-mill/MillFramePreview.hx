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
		return new MillFrameLayout({yRailSpacing: 220, yRailStart: y.y - 100, yRailLength: yRail.length + 88,
			columnFront: column.y - reference.column.depth / 2, columnHeight: reference.column.height + rise + 40 + spindleStackRise(reference),
			zRailSpacing: Math.max(180, reference.column.railSpacing),
			zRailStart: z.z - reference.base.height + 100, zRailLength: zRail.length + rise - 60 + spindleStackRise(reference)}, {baseWidth: 520, columnFootWidth: 440, frontExtension: 40, footPockets: true});
	}

	public static function spindleStackRise(reference:BenchMill):Float {
		var poses:Map<String,AssemblyFrame> = reference.solvedPoses();
		var head:AssemblyFrame = poses.get("head"), tool:AssemblyFrame = poses.get("tool");
		return machinekit.milling.Bt30Cartridge.SEAT+35+reference.tool.stickout-(head.z-tool.z);
	}

	public static function stageRise(reference:BenchMill):Float
		return MillXYStageLayout.stackHeight(BenchMill.RAIL) - (reference.tableTop - reference.base.height);

	public static function zLayout(reference:BenchMill, layout:MillFrameLayout, guideBlock:Null<machinekit.motion.LinearRailBlock> = null):MillZAxisLayout {
		var poses:Map<String, AssemblyFrame> = reference.solvedPoses();
		var head:AssemblyFrame = poses.get("head"), spindle:AssemblyFrame = poses.get("spindle");
		return new MillZAxisLayout(layout, head.z + layout.railPlane - reference.base.height + stageRise(reference) + spindleStackRise(reference),
			reference.head.height, spindle.y, reference.specs[2].lower, BenchMill.RAIL, guideBlock, layout.railPlane + machinekit.motion.LinearRailBlock.metric(BenchMill.RAIL).spec.blockHeight);
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

	public static function concept(reference:BenchMill, frame:MillFrame, ?motorAdjustment:Float):MachineAssembly {
		var result = structure(frame);
		var spindleLayout:MillZAxisLayout = frame.zAxis;
		var spindleParts = new machinekit.milling.MillSpindleHead(spindleLayout,motorAdjustment);
		var spindleOrigin = spindleParts.gauge;
		var tool = new machinekit.milling.FlatEndMill(new machinekit.component.ComponentValues().setNumber("diameter",reference.tool.diameter)
			.setNumber("fluteLength",reference.tool.fluteLength).setNumber("stickout",reference.tool.stickout));
		spindleParts.addComponent("tool",tool,F.translation(0,spindleLayout.spindleY,spindleOrigin.z-35-reference.tool.stickout));
		spindleParts.addMemberConnector("holder","toolSeat",F.translation(0,0,-35-reference.tool.stickout));
		spindleParts.addMemberConnector("tool","seat",F.identity());
		spindleParts.addMate("tool-in-holder","fixed","holder","toolSeat","tool","seat");
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

	static function writeServiceStudy(assembly:MachineAssembly, layout:MillZAxisLayout, directory:String):Void {

		machinekit.milling.MillSpindleIntegrity.checkPayload(assembly,layout).throwIfErrors();
		var poses:Map<String,AssemblyFrame> = assembly.solvedPoses(), cache = new machinekit.assembly.PosedParts();
		var parts:Array<materia.project.SceneArtifact.SceneArtifactPart> = [];
		try for (member in assembly.components()) if (member.id == "zAxis/head" || member.id == "zAxis/carriage" ||
			member.id == "frame/column/casting" || StringTools.startsWith(member.id, "spindle/"))
			parts.push(AssemblyPreview.part(member.id, member.component.description, cache.posed(member.component, poses.get(member.id)), member.component.materialId));
		catch(e:Dynamic) { cache.close(); throw e; }
		cache.close();
		// This static inspection scene carries no physics. CAD clearance is checked above and by the axis audits.
		var model = new cadkit.modeling.AssemblyModel("mm");
		for (part in parts) model.add(part.id);
		var definition = model.definition("spindle-service-review");
		for (component in definition.definitions) component.collisionHulls = [];
		sys.io.File.saveBytes(directory + "/mill-spindle-service-study.mtrg", materia.project.SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:parts,
			assemblyDefinition:definition,assemblyState:model.initialState("spindle-service-review").record()}));
	}


    /** Same concept report schema for previews and same-build FEM reference packets. */
    public static function writeReport(assembly:MachineAssembly,frame:MillFrame,directory:String,clearancesVerified:Bool,?extraPose:{x:Float,y:Float,z:Float}):Dynamic {
        var l=frame.layout,z:MillZAxisLayout=frame.zAxis,xy:MillXYStageLayout=frame.xyAxis;
		var report = {status: "mechanical XYZ concept; not qualified for manufacture", units: "mm",
			analysisReference:MillFemReference.capture(assembly,frame,extraPose),
			travel: [{axis:"x",travel:2*xy.xHalfTravel},{axis:"y",travel:2*xy.yHalfTravel},{axis:"z",travel:-z.lower}],
			base: {width: l.baseWidth, depth: l.baseDepth, height: l.baseHeight, railPlane: l.railPlane},
			column: {width: l.columnWidth, depth: l.columnDepth, height: l.columnHeight, railFaceY: l.columnFront,
				footShoulderMm: MillFrameLayout.COLUMN_FOOT_SHOULDER,footHeightMm: MillFrameLayout.COLUMN_FOOT_HEIGHT,clampAccess:"vertical side face"},
			xyFullStrokeStructuralClearance: clearancesVerified?"passed conservative Cartesian-product CAD swept-volume audit":"not rerun by the physical-reference writer",
			zFullStrokeStructuralClearance: clearancesVerified?"passed conservative CAD swept-volume audit":"not rerun by the physical-reference writer",
			xy: {tableTop: xy.tableTop, saddleHeight: MillXYStageLayout.SADDLE_HEIGHT, tableRibDepth: MillXYStageLayout.RISER_HEIGHT, xOvertravel: xy.xOvertravel, yOvertravel: xy.yOvertravel, yScrewZ: xy.yScrewZ, yMotorFace: xy.yMotorFace, yBayRoof: l.roof, yBayLocation: "lower column", yServiceRear: xy.yServiceRear},
			z: {screwX: 0, screwY: z.screwY, guideSpacing: l.zRailSpacing, lower: z.lower, upper: 0},
			castingMassKg: {base: frame.base.castingByWeight().kg, column: frame.column.castingByWeight().kg},
			pending: ["BT30 study axial interfaces, release force/stroke and supplier selection", "guide/block drilling, vendor mounting patterns and installed fasteners", "insert anchorage and strength analysis",
				"electrical brake and motor controls", "chip drainage and enclosure", "lifting and mold design", "material qualification and static/modal analysis"]};
        sys.io.File.saveContent(directory+"/mill-frame-concept.json",haxe.Json.stringify(report,null,"  "));
        return report;
    }

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
		writeServiceStudy(concept(reference, frame), z, directory);

		MillDesignStudy.write(concept(reference, frame), frame, directory);
		writeReport(concept(reference,frame),frame,directory,true);
		Sys.println('Mill frame concept report: $directory/mill-frame-concept.json');
	}
}
