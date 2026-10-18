package machinekit.milling;

import machinekit.assembly.AxisBuilder;
import machinekit.assembly.Transmission;
import machinekit.component.ComponentValues;
import machinekit.motion.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Supported nested Y saddle / X table. Each real prismatic joint drives its screw;
 * motors and bearing housings stay on their carrying structure, rotors follow the screws.
 */
class MillXYStage extends AxisBuilder {
	public final layout:MillXYStageLayout;
	public final saddle:MillCrossSaddle;
	public final table:MillCrossTable;

	public function new(layout:MillXYStageLayout) {
		super();
		if (layout == null) throw "XY stage needs a layout";
		this.layout = layout;
		var l = layout, f = l.frame;
		var y = orient(0, f.yRailStart, l.yScrewZ, [0, 0, 1], [0, 1, 0]);
		drive("Y", y, l.yScrew, l.yFixedY - f.yRailStart, l.yFloatingY - f.yRailStart, 100, l.yScrewZ - f.railPlane, null);
		var yp:Array<AssemblyFrame> = [];
		for (side in [-1, 1]) {
			var rail = orient(side * f.yRailSpacing / 2, f.yRailStart, f.railPlane + l.block.spec.railHeight, [0, 0, 1], [0, 1, 0]);
			attach('railY${side < 0 ? "Left" : "Right"}', LinearRail.metric(l.profile, f.yRailLength), rail, "fixedPedestalY");
			for (span in [-l.blockSpan, l.blockSpan]) yp.push(orient(
				side * f.yRailSpacing / 2,
				l.centreY + span,
				f.railPlane + l.block.spec.railHeight,
				[0, 0, 1],
				[0, 1, 0]
			));
		}
		slide(
			{id: "y", lower: -l.yHalfTravel, upper: l.yHalfTravel, initial: 0},
			"railYLeft",
			"blockY0",
			l.block,
			yp[0],
			{x: 0, y: -1, z: 0},
			yp.slice(1), l.yOvertravel);
		saddle = new MillCrossSaddle();
		attach("saddle", saddle, F.translation(0, l.centreY, l.saddleBottom), "blockY0");
		for (i in 1...4) attach('blockY$i', l.block, yp[i], "saddle");
		movingNut("Y", y, l.centreY + l.saddleDepth / 2 - f.yRailStart, "saddle", false);
		var x = orient(l.xScrewStart, l.centreY, l.xScrewZ, [0, 0, 1], [1, 0, 0]);
		drive("X", x, l.xScrew, l.xFixedX - l.xScrewStart, l.xFloatingX - l.xScrewStart, 84, 40, "saddle");
		var xp:Array<AssemblyFrame> = [];
		for (side in [-1, 1]) {
			attach(
				'railX${side < 0 ? "Front" : "Rear"}',
				LinearRail.metric(l.profile, l.xRailLength),
				orient(
					-l.xRailLength / 2,
					l.centreY + side * l.xRailSpacing / 2,
					l.xRailPlane + l.block.spec.railHeight,
					[0, 0, 1],
					[1, 0, 0]
				),
				"saddle"
			);
			for (span in [-l.blockSpan, l.blockSpan]) xp.push(orient(
				span,
				l.centreY + side * l.xRailSpacing / 2,
				l.xRailPlane + l.block.spec.railHeight,
				[0, 0, 1],
				[1, 0, 0]
			));
		}
		slide(
			{id: "x", lower: -l.xHalfTravel, upper: l.xHalfTravel, initial: 0},
			"railXFront",
			"blockX0",
			l.block,
			xp[0],
			{x: -1, y: 0, z: 0},
			xp.slice(1), l.xOvertravel);
		attach(
			"riser0",
			new MillPanel(l.block.spec.blockLength, l.block.spec.blockWidth, MillXYStageLayout.RISER_HEIGHT),
			F.translation(xp[0].x, xp[0].y, l.xRailPlane + l.block.spec.blockHeight),
			"blockX0"
		);
		table = new MillCrossTable();
		attach("table", table, F.translation(0, l.centreY, l.tableBottom), "riser0");
		for (i in 1...4) {
			attach('blockX$i', l.block, xp[i], "table");
			attach(
				'riser$i',
				new MillPanel(l.block.spec.blockLength, l.block.spec.blockWidth, MillXYStageLayout.RISER_HEIGHT),
				F.translation(xp[i].x, xp[i].y, l.xRailPlane + l.block.spec.blockHeight),
				'blockX$i'
			);
		}
		movingNut("X", x, -l.xScrewStart, "table", true);
		for (a in ["X", "Y"]) {
			var id = a.toLowerCase(), along = a == "X" ? [1.0, 0.0, 0.0] : [0.0, 1.0, 0.0];
			addTransmission('$id-lead', id, 'screw$a-turn', Transmission.LeadScrew('screw$a', 'nut$a'), screwSense(id, along));
			supportScrew('$id-lead', l.fixed.support, l.floating.support);
		}
		exposeConnector("mount", "fixedPedestalY", "mount");
		exposeConnector("table", "table", "top");
	}

	function movingNut(a:String, axis:AssemblyFrame, face:Float, parent:String, foot:Bool):Void {
		attach(
			'nutBracket$a',
			new MillBallNutBracket(new ComponentValues().setBoolean("tableFoot", foot)),
			F.compose(
				axis,
				F.translation(0, 0, face)
			),
			parent
		);
		attach(
			'nut$a',
			layout.nut,
			F.compose(
				axis,
				F.translation(0, 0, face - layout.nut.bodyLength - layout.nut.flangeThickness)
			),
			'nutBracket$a'
		);
	}

	function drive(
		a:String,
		axis:AssemblyFrame,
		screw:BallScrew,
		fixedAt:Float,
		floatingAt:Float,
		width:Float,
		shaftHeight:Float,
		parent:Null<String>
	):Void {
		var l = layout, h = shaftHeight - 25;
		var pedestalRotation = F.fromRotationMatrix(0, 0, 0, [1, 0, 0, 0, 0, 1, 0, -1, 0]);
		for (end in ["fixed", "floating"]) {
			var fixed = end == "fixed", station = fixed ? fixedAt : floatingAt, bearing = fixed ? l.fixed : l.floating;
			var part = new MillScrewPedestal(new ComponentValues().setNumber("width", width).setNumber(
				"depth",
				bearing.length
			).setNumber("height", h).setBoolean("motorAnchors", fixed));
			var pose = F.compose(axis, F.compose(F.translation(0, -shaftHeight, station), pedestalRotation));
			if (parent == null && fixed) place('fixedPedestal$a', part, pose);
			else attach('${end}Pedestal$a', part, pose, parent == null ? 'fixedPedestal$a' : parent);
			attach('${end}Bearing$a', bearing, F.compose(axis, F.translation(0, 0, station)), '${end}Pedestal$a');
		}
		var mountRotation = F.fromRotationMatrix(0, 0, 0, [-1, 0, 0, 0, 1, 0, 0, 0, -1]);
		attach(
			'motorMount$a',
			new MillServoMount(new ComponentValues().setNumber("width", width).setNumber("shaftHeight", shaftHeight).setNumber("pedestalHeight", h)),
			F.compose(
				axis,
				F.compose(
					F.translation(0, 0, fixedAt - l.fixed.length / 2),
					mountRotation
				)
			),
			'fixedPedestal$a'
		);
		attach('motor$a', l.motor, F.compose(axis, F.translation(0, 0, -l.motor.rating.shaftLength)), 'motorMount$a');
		hang('screw$a', screw, axis, 'fixedBearing$a');
		var along = F.transformVector(axis, 0, 0, 1);
		addMateOnAxis('screw$a-turn', "continuous", 'fixedBearing$a', 'to-screw$a', 'screw$a', 'attach-screw$a', along);
		attach('motorShaft$a', l.motor.shaft(), F.compose(axis, F.translation(0, 0, -l.motor.rating.shaftLength)), 'screw$a');
		attach('coupling$a', l.coupling, F.compose(axis, F.translation(0, 0, -l.coupling.length / 2)), 'screw$a');
	}
}
