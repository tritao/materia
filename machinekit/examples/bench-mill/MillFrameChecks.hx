import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.component.MachineKitComponents;
import machinekit.milling.MillFrame;
import machinekit.milling.MillFrameLayout;
import machinekit.structural.FoundationIntegrity;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Check real solid partitioning, solved mating datums and recipe persistence. */
class MillFrameChecks {
	static var assertions = 0;
	static function check(value:Bool, label:String):Void { assertions++; if (!value) throw label; }
	static function near(a:Float, b:Float, label:String):Void check(Math.abs(a - b) < 1e-6, '$label ($a vs $b)');
	static function rejects(action:Void->Void, label:String):Void {
		var rejected = false; try action() catch (_:Dynamic) rejected = true;
		check(rejected, label);
	}
	public static function main():Void {
		var reference = new BenchMill(false, false), layout = MillFramePreview.layout(reference), frame = new MillFrame(layout);
		frame.validate();
		var poses = frame.solvedPoses();
		for (foundation in [frame.base, frame.column]) {
			check(!FoundationIntegrity.check(foundation).hasErrors(), "nominal insert planes");
			var body = foundation.body.geometry();
			check(body.valid() && body.solidCount() == 1, "casting remains one connected valid solid");
			var locations = foundation.solvedPoses();
			for (row in foundation.insertLocations()) {
				var local = row.part.geometry(), steel = PosedParts.moved(local, locations.get(row.id));
				var common = body.intersect(steel);
				near(common.volume(), 0, 'no double-counted steel ${row.id}');
				common.close(); steel.close(); local.close();
			}
			check(foundation.castingByWeight().kg > 0, "casting mass available");
			body.close();
		}
		for (side in ["Left", "Right"]) {
			var railY = F.compose(poses.get('base/railY$side'), frame.base.components().filter(m -> m.id == 'railY$side')[0].component.connector("top").frame);
			near(railY.z, layout.railPlane, "Y steel mounting height");
			var railZ = F.compose(poses.get('column/railZ$side'), frame.column.components().filter(m -> m.id == 'railZ$side')[0].component.connector("top").frame);
			near(railZ.y, layout.columnFront, "Z steel mounting face");
			var normal = F.transformVector(railZ, 0, 1, 0);
			near(normal.y, -1, "Z mounting normal");
			for (end in ["Front", "Rear"]) {
				var id = side + end;
				var lower = F.transformPoint(poses.get('base/column$id'), 0, 0, layout.insertThickness);
				var upper = F.transformPoint(poses.get('column/foot$id'), 0, 0, layout.insertThickness);
				near(lower.x, upper.x, "column steel pair X"); near(lower.y, upper.y, "column steel pair Y");
				near(lower.z, upper.z, "column steel pair Z");
				var foot = F.transformPoint(poses.get('base/foot$id'), 0, 0, layout.insertThickness);
				near(foot.z, 0, "floor-side steel plane");
			}
		}
		var restored:MachineAssembly = MachineAssembly.decode(frame.encode(), MachineKitComponents.standard());
		restored.validate();
		check(restored.components().length == frame.components().length, "frame recipe roundtrip preserves parts");
		var restoredPoses:Map<String, AssemblyFrame> = restored.solvedPoses();
		for (member in frame.components()) {
			var original:AssemblyFrame = poses.get(member.id);
			var rebuilt:AssemblyFrame = restoredPoses.get(member.id);
			near(rebuilt.x, original.x, "roundtrip X"); near(rebuilt.y, original.y, "roundtrip Y"); near(rebuilt.z, original.z, "roundtrip Z");
		}
		var study = MillFramePreview.concept(reference, frame);
		study.validate();
		var studyPoses:Map<String, AssemblyFrame> = study.solvedPoses();
		var referencePoses:Map<String, AssemblyFrame> = reference.solvedPoses();
		check(study.components().filter(m -> m.id == "reference/base" || m.id == "reference/column").length == 0, "old structural bodies omitted");
		for (id in ["railYLeft", "railZLeft", "saddle", "table", "head", "tool", "screwX", "screwY", "screwZ"]) {
			var expected:AssemblyFrame = referencePoses.get(id);
			var actual:AssemblyFrame = studyPoses.get('reference/$id');
			check(actual != null, 'reference part $id included');
			near(actual.x - expected.x, id == "screwZ" ? MillFramePreview.zDriveOffset(reference, frame) : 0,
				"reference X or explicit Z drive clearance shift"); near(actual.y, expected.y, "reference Y retained");
			near(actual.z - expected.z, layout.railPlane - reference.base.height, "common datum shift");
		}
		var interfaces:MillFrameInterfaces = {yRailSpacing: layout.yRailSpacing, yRailStart: layout.yRailStart,
			yRailLength: layout.yRailLength, columnFront: layout.columnFront, columnHeight: layout.columnHeight,
			zRailSpacing: layout.zRailSpacing, zRailStart: layout.zRailStart, zRailLength: layout.zRailLength};
		var variant = new MillFrame(new MillFrameLayout(interfaces,
			{baseWidth: 620, baseHeight: 260, columnWidth: 360, columnDepth: 300}));
		variant.validate();
		var variantPoses:Map<String, AssemblyFrame> = variant.solvedPoses();
		var variantColumn:AssemblyFrame = variantPoses.get("column/casting");
		near(variantColumn.z, 264, "changed base height moves the column interface");
		near(variantColumn.y - 150 - 4, layout.columnFront, "deeper column preserves the guide datum");
		for (foundation in [variant.base, variant.column]) {
			var shape = foundation.body.geometry();
			check(shape.valid() && shape.solidCount() == 1, "resized frame stays connected");
			shape.close();
		}
		rejects(() -> new MillFrameLayout(interfaces, {baseWidth: 100}), "reject unsupported guide/column layout");
		rejects(() -> new MillFrameLayout(interfaces, {seatRise: 20}), "reject unembedded inserts");
		rejects(() -> new MillFrameLayout(interfaces, {wall: Math.NaN}), "reject nonfinite geometry");
		rejects(() -> new MillFrameLayout(interfaces, {roof: 220}), "reject absent underside cavities");
		rejects(() -> new MillFrameLayout(interfaces, {columnDepth: 120}), "reject overlapping column sole pads");
		Sys.println('Mill frame: $assertions assertions passed');
	}
}
