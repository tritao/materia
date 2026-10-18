import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.component.MachineKitComponents;
import machinekit.milling.*;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

class MillXYStageChecks {
	static var assertions = 0;

	static function check(ok:Bool, label:String):Void {
		assertions++;
		if (!ok) throw label;
	}

	static function near(a:Float, b:Float, label:String):Void check(Math.abs(a - b) < 1e-6, '$label ($a vs $b)');

	public static function main():Void {
		AxisBuilderChecks.run();
		var reference = new BenchMill(false, false), f = MillFramePreview.layout(reference), z = MillFramePreview.zLayout(
			reference,
			f
		);
		var l = new MillXYStageLayout(f, z.spindleY, BenchMill.RAIL), axis = new MillXYStage(l), frame = new MillFrame(f, z, l);
		axis.validate();
		frame.validate();
		near(l.tableTop, 448, "stack height");
		near(f.yRailSpacing, 220, "broad Y guide stance");
		near(l.xRailSpacing, 160, "X guides support separated table ribs");
		check(2*l.yBlockSpan > 80, "Y saddle has a longer guide footprint");
		near(l.xMotorZ, 292, "X return drive independent of lowered Y shaft");
		near(l.yScrewZ, 124, "Y direct shaft lowered beneath column");
		near(axis.axisOvertravel("x"), l.xOvertravel, "X bearing/guide overrun limit");
		near(axis.axisOvertravel("y"), l.yOvertravel, "Y bearing overrun limit");
		for (m in axis.components()) {
			var p = m.component.geometry();
			check(p.valid() && p.solidCount() == 1, 'connected valid solid ${m.id}');
			p.close();
		}
		var restored = MachineAssembly.decode(axis.encode(), MachineKitComponents.standard());
		restored.validate();
		check(restored.components().length == axis.components().length, "all stage recipes roundtrip");
		for (assembly in [cast(axis, MachineAssembly), restored]) {
			var model = new AssemblyModel("mm");
			assembly.addTo(model, "");
			var state = new AssemblyState(model.definition("xy-stage"));
			for (ix in 0...11) for (iy in 0...11) {
				var x = -l.xHalfTravel + ix * 2 * l.xHalfTravel / 10, y = -l.yHalfTravel + iy * 2 * l.yHalfTravel / 10;
				state.setJoint("x", x);
				state.setJoint("y", y);
				state.forwardKinematics();
				var table = state.worldPose("table"),
					saddle = state.worldPose("saddle"), motorX = state.worldPose("motorX"), motorY = state.worldPose("motorY");
				near(table.x, -x, "table follows X");
				near(table.y, l.centreY - y, "table follows Y");
				near(table.z, l.tableBottom, "table stays level");
				near(saddle.x, 0, "saddle independent of X");
				near(saddle.y, l.centreY - y, "saddle follows Y");
				near(motorX.x, -195, "X motor independent of X");
				near(motorX.y, l.centreY - y, "X drive rides saddle");
				near(motorY.y, l.yMotorFace, "Y motor remains fixed");
				near(state.joint("motorX-turn"), -state.joint("screwX-turn"), "opposed motor axis follows return belt");
				near(state.joint("screwX-turn"), l.nut.rotationFor(-x), "X screw signed lead");
				near(state.joint("screwY-turn"), l.nut.rotationFor(y), "Y screw signed lead");
			}
		}
		var poses:Map<String, AssemblyFrame> = axis.solvedPoses(), cache = new PosedParts();
		var pairs = [["fixedBearingY", "fixedPedestalY"], ["floatingBearingY", "floatingPedestalY"],
			["motorMountY", "fixedPedestalY"], ["motorY", "motorMountY"], ["fixedBearingX", "fixedPedestalX"],
			["floatingBearingX", "floatingPedestalX"], ["motorMountX", "saddle"], ["motorX", "motorMountX"],
			["nutY", "nutBracketY"], ["nutBracketY", "saddle"], ["nutX", "nutBracketX"], ["nutBracketX", "table"],
			["saddle", "blockY0"], ["table", "blockX0"], ["fixedPedestalX", "saddle"], ["floatingPedestalX", "saddle"],
			["motorCoverX", "motorMountX"]];
		for (i in 1...4) { pairs.push(["saddle", "blockY"+i]); pairs.push(["table", "blockX"+i]); }
		for (pair in pairs) {
			var a = cache.posed(axis.component(pair[0]), poses.get(pair[0])), b = cache.posed(
				axis.component(pair[1]),
				poses.get(pair[1])
			);
			near(a.distance(b), 0, 'supported contact $pair');
			var common = a.intersect(b);
			near(common.volume(), 0, 'no overlap at $pair');
			common.close();
			a.close();
			b.close();
		}
		for (id in ["motorX", "motorShaftX", "motorPulleyX", "screwPulleyX", "beltX"]) {
			near(cache.overlap(axis.component(id), poses.get(id), axis.saddle, poses.get("saddle")), 0, 'return drive clears saddle: $id');
		}
		for (id in ["motorShaftX", "motorPulleyX", "screwPulleyX", "beltX", "motorMountX", "fixedBearingX"]) {
			near(cache.overlap(axis.component(id), poses.get(id), axis.component("motorCoverX"), poses.get("motorCoverX")), 0, 'return hood clears $id');
		}
		for (id in ["motorX", "motorY", "motorCoverX", "motorCoverY"]) {
			var b = cache.box(axis.component(id), poses.get(id));
			check(b.minX >= -f.baseWidth / 2 && b.maxX <= f.baseWidth / 2, 'drive stays inside base width: $id');
			var travel = id.indexOf("X") >= 0 ? l.yHalfTravel : 0;
			check(b.minY - travel >= f.front && b.maxY + travel <= f.rear + (id == "motorCoverY" ? 4 : 0), 'drive stays inside base depth over full Y stroke: $id');
		}
		cache.close();
		var audit = MillXYStageIntegrity.check(axis, frame);
		audit.throwIfErrors();
		check(!audit.hasErrors(), "continuous full Cartesian stroke audit");
		var uncut = MillXYStageIntegrity.check(axis, new MillFrame(f, z));
		check(uncut.hasErrors(), "missing bearing steel seats rejected");
		check(uncut.items.filter(item -> item.code == "mill-xy.full-stroke-collision").length > 0, "unhollowed base collides with lowered Y drive");
		near(l.yHalfTravel * 2, 160, "usable Y travel increases with table coverage retained");
		near(l.tableDepth / 2 - l.yHalfTravel, 25, "tool axis retains table-edge margin at both stops");
		near(f.columnFront - l.block.spec.railHeight - (l.centreY + l.yHalfTravel + l.tableDepth / 2), 3, "rear table clears lower Z rails");
		check(l.yScrewZ + 33 + l.clearance <= l.yBayTop, "motor cage fits below base roof");
		check(f.baseHeight - l.yBayTop >= f.roof, "full casting roof retained below column");
		var blocked = new MillFrame(f, z, l);
		blocked.addComponent("obstruction", new MillPanel(20, 20, 20), F.translation(0, l.centreY, l.tableBottom + 5));
		var blockedAudit = MillXYStageIntegrity.check(axis, blocked);
		check(
			blockedAudit.items.filter(item -> item.code == "mill-xy.full-stroke-collision").length > 0,
			"audit detects an actual obstructed XY sweep"
		);
		var extractionBlocked = new MillFrame(f, z, l);
		extractionBlocked.addComponent("extractionObstruction", new MillPanel(20, 20, 20), F.translation(0, f.rear + 20, l.yScrewZ - 10));
		var extractionAudit = MillXYStageIntegrity.check(axis, extractionBlocked);
		check(extractionAudit.items.filter(item -> item.subject.indexOf("rear motor extraction/frame/extractionObstruction") >= 0).length > 0,
			"audit rejects blocked rear motor withdrawal");
		var concept = MillFramePreview.concept(reference, frame), model = new AssemblyModel("mm");
		concept.addTo(model, "");
		var state = new AssemblyState(model.definition("xyz-mill"));
		state.setJoint("xyStage/x", 125);
		state.setJoint("xyStage/y", 75);
		state.setJoint("zAxis/z", -250);
		state.forwardKinematics();
		near(state.worldPose("xyStage/table").x, -125, "integrated X");
		near(state.worldPose("xyStage/table").y, l.centreY - 75, "integrated Y");
		near(state.worldPose("zAxis/head").z, z.headBottom - 250, "integrated Z");
		Sys.println('Mill XY stage: $assertions assertions passed');
	}
}
