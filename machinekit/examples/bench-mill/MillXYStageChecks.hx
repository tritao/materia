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
				near(motorX.x, l.xScrewStart - l.motor.rating.shaftLength, "X motor independent of X");
				near(motorX.y, l.centreY - y, "X drive rides saddle");
				near(motorY.y, f.yRailStart - l.motor.rating.shaftLength, "Y motor remains fixed");
				near(state.joint("screwX-turn"), l.nut.rotationFor(-x), "X screw signed lead");
				near(state.joint("screwY-turn"), l.nut.rotationFor(-y), "Y screw signed lead");
			}
		}
		var poses:Map<String, AssemblyFrame> = axis.solvedPoses(), cache = new PosedParts();
		var pairs = [
			["fixedBearingY", "fixedPedestalY"],
			["floatingBearingY", "floatingPedestalY"],
			[
				"motorMountY",
				"fixedPedestalY"
			],
			[
				"motorY",
				"motorMountY"
			],
			[
				"fixedBearingX",
				"fixedPedestalX"
			],
			[
				"floatingBearingX",
				"floatingPedestalX"
			],
			[
				"motorMountX",
				"fixedPedestalX"
			],
			[
				"motorX",
				"motorMountX"
			],
			[
				"nutY",
				"nutBracketY"
			],
			[
				"nutBracketY",
				"saddle"
			],
			[
				"nutX",
				"nutBracketX"
			],
			[
				"nutBracketX",
				"table"
			],
			[
				"saddle",
				"blockY0"
			],
			[
				"table",
				"riser0"
			],
			[
				"riser0",
				"blockX0"
			],
			[
				"fixedPedestalX",
				"saddle"
			],
			[
				"floatingPedestalX",
				"saddle"
			]
		];
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
		cache.close();
		var audit = MillXYStageIntegrity.check(axis, frame);
		audit.throwIfErrors();
		check(!audit.hasErrors(), "continuous full Cartesian stroke audit");
		check(MillXYStageIntegrity.check(axis, new MillFrame(f, z)).hasErrors(), "missing bearing steel seats rejected");
		var blocked = new MillFrame(f, z, l);
		blocked.addComponent("obstruction", new MillPanel(20, 20, 20), F.translation(0, l.centreY, l.tableBottom + 5));
		var blockedAudit = MillXYStageIntegrity.check(axis, blocked);
		check(
			blockedAudit.items.filter(item -> item.code == "mill-xy.full-stroke-collision").length > 0,
			"audit detects an actual obstructed XY sweep"
		);
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
