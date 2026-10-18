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
		var reference = new BenchMill(false, false), layout = MillFramePreview.layout(reference);
		var z = MillFramePreview.zLayout(reference, layout), xy = new machinekit.milling.MillXYStageLayout(layout, z.spindleY, BenchMill.RAIL);
		var frame = new MillFrame(layout, z, xy);
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
				near(foot.z, layout.footPockets ? 40 : 0, "foot steel plane");
			}
		}
		check(frame.components().filter(m -> StringTools.startsWith(m.id, "levelingFoot")).length == 4, "four leveling feet installed");
		for (side in ["Left", "Right"]) for (end in ["Front", "Rear"]) {
			var id = side + end;
			var steelLocal = frame.base.components().filter(m -> m.id == 'foot$id')[0].component.geometry();
			var footLocal = frame.components().filter(m -> m.id == 'levelingFoot$id')[0].component.geometry();
			var steel = PosedParts.moved(steelLocal, poses.get('base/foot$id')), foot = PosedParts.moved(footLocal, poses.get('levelingFoot$id'));
			near(steel.distance(foot), 0, "adjusting nut bears on steel seat");
			var common = steel.intersect(foot); near(common.volume(), 0, "stem clears bored steel seat"); common.close();
			var casting = frame.base.body.geometry(), body = PosedParts.moved(casting, poses.get("base/casting"));
			common = body.intersect(foot); near(common.volume(), 0, "foot clears access pocket and stem opening"); common.close();
			body.close(); casting.close(); steel.close(); foot.close(); steelLocal.close(); footLocal.close();
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
		for (id in ["railYLeft", "saddle", "table", "screwX", "screwY"]) check(studyPoses.exists('xyStage/$id'), 'mechanical XY part $id included');
		var table:AssemblyFrame = studyPoses.get("xyStage/table");
		near(table.z, xy.tableBottom, "table follows new stack datum");
		near(table.y, z.spindleY, "table centred beneath spindle");
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
