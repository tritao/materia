import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import machinekit.milling.MillFrame;
import machinekit.milling.MillFrameLayout;
import machinekit.milling.MillZAxis;
import machinekit.milling.MillZAxisLayout;
import machinekit.milling.MillZAxisIntegrity;
import machinekit.motion.LinearRail;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

/** New mechanical Z assembly on the casting; X/Y remain a frozen reference.
 * The legacy BenchMill keeps its independent, working motion/control model.
 */
class MillFramePreview {
	public static function layout(reference:BenchMill):MillFrameLayout {
		var state:Map<String, AssemblyFrame> = reference.solvedPoses();
		var y:AssemblyFrame = state.get("railYLeft"), z:AssemblyFrame = state.get("railZLeft");
		var column:AssemblyFrame = state.get("column");
		var yRail:LinearRail = cast reference.components().filter(row -> row.id == "railYLeft")[0].component;
		var zRail:LinearRail = cast reference.components().filter(row -> row.id == "railZLeft")[0].component;
		return new MillFrameLayout({yRailSpacing: reference.base.railSpacing, yRailStart: y.y, yRailLength: yRail.length,
			columnFront: column.y - reference.column.depth / 2, columnHeight: reference.column.height,
			zRailSpacing: Math.max(180, reference.column.railSpacing),
			zRailStart: z.z - reference.base.height, zRailLength: zRail.length});
	}

	public static function zLayout(reference:BenchMill, layout:MillFrameLayout):MillZAxisLayout {
		var poses:Map<String, AssemblyFrame> = reference.solvedPoses();
		var head:AssemblyFrame = poses.get("head"), spindle:AssemblyFrame = poses.get("spindle");
		return new MillZAxisLayout(layout, head.z + layout.railPlane - reference.base.height,
			reference.head.height, spindle.y, reference.specs[2].lower, BenchMill.RAIL);
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
		var rise = frame.layout.railPlane - reference.base.height;
		var referenceParts = new MachineAssembly(), spindleParts = new MachineAssembly();
		var referenceIds:Array<String> = [], spindleIds:Array<String> = [];
		var inverseBase = F.inverse(frame.solvedPoses().get("base/casting"));
		var spindle:AssemblyFrame = state.get("spindle");
		var spindleOrigin = F.translation(spindle.x, spindle.y, spindle.z + rise);
		for (member in reference.components()) {
			var isSpindle = spindlePart(member.id);
			if (!isSpindle && !referencePart(member.id)) continue;
			var pose:AssemblyFrame = state.get(member.id);
			pose.z += rise;
			var target = isSpindle ? spindleParts : referenceParts;
			target.addComponent(member.id, member.component, pose);
			target.addMemberConnector(member.id, "studyOrigin", F.identity());
			if (isSpindle) {
				spindleIds.push(member.id);
				if (member.id != "spindle") spindleParts.addMemberConnector("spindle", 'payload-${member.id}',
					F.compose(F.inverse(spindleOrigin), pose));
			} else {
				result.addMemberConnector("frame/base/casting", 'study-${member.id}', F.compose(inverseBase, pose));
				referenceIds.push(member.id);
			}
		}
		for (id in spindleIds) if (id != "spindle") spindleParts.addMate('payload-$id', "fixed", "spindle", 'payload-$id', id, "studyOrigin");
		result.include("reference", referenceParts); result.include("spindle", spindleParts);
		for (id in referenceIds) result.addMate('study-$id', "fixed", "frame/base/casting", 'study-$id', 'reference/$id', "studyOrigin");
		var axis = new MillZAxis(frame.zAxis);
		result.addMemberConnector("zAxis/head", "spindlePayload", F.compose(F.inverse(axis.zeroPose("head")), spindleOrigin));
		result.addMemberConnector("spindle/spindle", "headSeat", F.identity());
		result.addMate("spindle-on-head", "fixed", "zAxis/head", "spindlePayload", "spindle/spindle", "headSeat");
		return result;
	}

	static function spindlePart(id:String):Bool return ["spindle", "holder", "tool", "spindleMotor", "spindlePulley",
		"spindleMotorPulley", "spindleMotorShaft", "spindleBelt", "spindleMotorShelf", "spindleMotorRiser"].indexOf(id) >= 0;

	static function referencePart(id:String):Bool {
		if (StringTools.startsWith(id, "railZ") || StringTools.startsWith(id, "blockZ")) return false;
		if (StringTools.startsWith(id, "rail") || StringTools.startsWith(id, "block")) return true;
		if (["saddle", "table"].indexOf(id) >= 0) return true;
		for (axis in ["X", "Y"]) if (['screw$axis', 'screw${axis}Nut', 'screw${axis}Fixed', 'screw${axis}Floating',
			'screw${axis}Coupling', 'motor$axis', 'motor${axis}Shaft'].indexOf(id) >= 0) return true;
		return false;
	}

	public static function main():Void {
		var reference = new BenchMill(false, false), l = layout(reference);
		var z = zLayout(reference, l), frame = new MillFrame(l, z);
		MillZAxisIntegrity.check(new MillZAxis(z), frame).throwIfErrors();
		var root = Sys.getCwd(), directory = sys.FileSystem.exists(root + "/machinekit")
			? root + "/machinekit/examples/bench-mill/build" : root + "/build";
		if (!sys.FileSystem.exists(directory)) sys.FileSystem.createDirectory(directory);
		for (entry in [{name: "mill-frame", assembly: cast(frame, MachineAssembly)},
			{name: "mill-z-axis", assembly: structure(frame)}, {name: "mill-frame-concept", assembly: concept(reference, frame)}]) {
			var path = directory + "/" + entry.name + ".mtrg";
			sys.io.File.saveBytes(path, materia.project.SceneArtifact.encode(AssemblyPreview.scene(entry.assembly, entry.name)));
			Sys.println('Mill frame preview: $path');
		}
		var report = {status: "mechanical Z concept; X/Y frozen reference; not qualified for manufacture", units: "mm",
			travel: [for (axis in reference.specs) {axis: axis.id, travel: axis.upper - axis.lower}],
			base: {width: l.baseWidth, depth: l.baseDepth, height: l.baseHeight, railPlane: l.railPlane},
			column: {width: l.columnWidth, depth: l.columnDepth, height: l.columnHeight, railFaceY: l.columnFront},
			zFullStrokeStructuralClearance: "passed conservative CAD swept-volume audit",
			z: {screwX: 0, screwY: z.screwY, guideSpacing: l.zRailSpacing, lower: z.lower, upper: 0},
			castingMassKg: {base: frame.base.castingByWeight().kg, column: frame.column.castingByWeight().kg},
			pending: ["X/Y mounting adapters", "vendor mounting patterns and installed fasteners", "insert anchorage and strength analysis",
				"electrical brake and motor controls", "chip drainage and enclosure", "lifting and mold design", "material qualification and static/modal analysis"]};
		sys.io.File.saveContent(directory + "/mill-frame-concept.json", haxe.Json.stringify(report, null, "  "));
		Sys.println(haxe.Json.stringify(report));
	}
}
