import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import machinekit.milling.MillFrame;
import machinekit.milling.MillFrameLayout;
import machinekit.motion.LinearRail;
import machinekit.motion.BallNut;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

/** Static structural study using the working mill's guide/carriage datums.
 * The wider column moves the Z drive sideways; this is not a motion model.
 */
class MillFramePreview {
	public static function layout(reference:BenchMill):MillFrameLayout {
		var state:Map<String, AssemblyFrame> = reference.solvedPoses();
		var y:AssemblyFrame = state.get("railYLeft");
		var z:AssemblyFrame = state.get("railZLeft");
		var column:AssemblyFrame = state.get("column");
		var yRail:LinearRail = cast reference.components().filter(row -> row.id == "railYLeft")[0].component;
		var zRail:LinearRail = cast reference.components().filter(row -> row.id == "railZLeft")[0].component;
		return new MillFrameLayout({yRailSpacing: reference.base.railSpacing, yRailStart: y.y, yRailLength: yRail.length,
			columnFront: column.y - reference.column.depth / 2,
			columnHeight: reference.column.height, zRailSpacing: reference.column.railSpacing,
			zRailStart: z.z - reference.base.height, zRailLength: zRail.length});
	}

	public static function concept(reference:BenchMill, frame:MillFrame):MachineAssembly {
		var result = new MachineAssembly();
		var state:Map<String, AssemblyFrame> = reference.solvedPoses();
		result.include("frame", frame);
		var rise = frame.layout.railPlane - reference.base.height;
		var zDriveShift = zDriveOffset(reference, frame);
		var referenceParts = new MachineAssembly();
		var referenceIds:Array<String> = [];
		var inverseBase = F.inverse(frame.solvedPoses().get("base/casting"));
		for (member in reference.components()) if (referencePart(member.id)) {
			var pose:AssemblyFrame = state.get(member.id);
			pose.z += rise;
			if (StringTools.startsWith(member.id, "screwZ") || StringTools.startsWith(member.id, "motorZ"))
				pose.x += zDriveShift;
			referenceParts.addComponent(member.id, member.component, pose);
			referenceParts.addMemberConnector(member.id, "studyOrigin", F.identity());
			result.addMemberConnector("frame/base/casting", 'study-${member.id}', F.compose(inverseBase, pose));
			referenceIds.push(member.id);
		}
		result.include("reference", referenceParts);
		// Lock the arrangement for inspection; these are study constraints, not
		// claims that the copied hardware has finished mounting adapters.
		for (id in referenceIds) result.addMate('study-$id', "fixed", "frame/base/casting", 'study-$id',
			'reference/$id', "studyOrigin");
		return result;
	}

	/** Keep the Z nut outside the wider column, using its actual flange size.
	 * The required head adapter is intentionally left for the next design step.
	 */
	public static function zDriveOffset(reference:BenchMill, frame:MillFrame):Float {
		var nut:BallNut = cast reference.components().filter(row -> row.id == "screwZNut")[0].component;
		var poses:Map<String, AssemblyFrame> = reference.solvedPoses();
		var screw:AssemblyFrame = poses.get("screwZ");
		return frame.layout.columnWidth / 2 + nut.flangeDiameter / 2 + 2 - screw.x;
	}

	static function referencePart(id:String):Bool {
		// The old base/column, electrical cabinet and their brackets belong to
		// the previous structure. Do not imply those adapters fit the new frame.
		if (StringTools.startsWith(id, "rail") || StringTools.startsWith(id, "block")) return true;
		if (["saddle", "table", "head", "spindle", "holder", "tool", "spindleMotor", "spindlePulley",
			"spindleMotorPulley", "spindleMotorShaft", "spindleBelt"].indexOf(id) >= 0) return true;
		for (axis in ["x", "y", "z"]) if (["screw" + axis.toUpperCase(), "screw" + axis.toUpperCase() + "Nut", "screw" + axis.toUpperCase() + "Fixed",
			"screw" + axis.toUpperCase() + "Floating", "screw" + axis.toUpperCase() + "Coupling", "motor" + axis.toUpperCase(),
			"motor" + axis.toUpperCase() + "Shaft"].indexOf(id) >= 0) return true;
		return false;
	}

	public static function main():Void {
		var reference = new BenchMill(false, false), frame = new MillFrame(layout(reference));
		var root = Sys.getCwd(), directory = sys.FileSystem.exists(root + "/machinekit")
			? root + "/machinekit/examples/bench-mill/build" : root + "/build";
		if (!sys.FileSystem.exists(directory)) sys.FileSystem.createDirectory(directory);
		for (entry in [{name: "mill-frame", assembly: cast(frame, MachineAssembly)},
			{name: "mill-frame-concept", assembly: concept(reference, frame)}]) {
			var path = directory + "/" + entry.name + ".mtrg";
			sys.io.File.saveBytes(path, materia.project.SceneArtifact.encode(AssemblyPreview.scene(entry.assembly, entry.name)));
			Sys.println('Mill frame preview: $path');
		}
		var l = frame.layout;
		var report = {status: "structural concept; not qualified for manufacture", units: "mm",
			travel: [for (axis in reference.specs) {axis: axis.id, travel: axis.upper - axis.lower}],
			base: {width: l.baseWidth, depth: l.baseDepth, height: l.baseHeight, railPlane: l.railPlane},
			column: {width: l.columnWidth, depth: l.columnDepth, height: l.columnHeight, railFaceY: l.columnFront},
			zDriveShiftX: zDriveOffset(reference, frame),
			castingMassKg: {base: frame.base.castingByWeight().kg, column: frame.column.castingByWeight().kg},
			pending: ["drive mounting adapters and insert patterns", "full-travel clearance audit", "fasteners and insert anchorage",
				"chip drainage and enclosure", "lifting and mold design", "material qualification and static/modal analysis"]};
		sys.io.File.saveContent(directory + "/mill-frame-concept.json", haxe.Json.stringify(report, null, "  "));
		Sys.println(haxe.Json.stringify(report));
	}
}
