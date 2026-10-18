package machinekit.milling;

import machinekit.assembly.AxisBuilder;
import machinekit.assembly.Transmission;
import machinekit.component.ComponentValues;
import machinekit.motion.*;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.transmission.TimingPulley;
import machinekit.transmission.TimingBelt;
import machinekit.assembly.Sense;
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
		var y = orient(0, l.yScrewStart, l.yScrewZ, [0, 0, 1], [0, -1, 0]);
		drive("Y", y, l.yScrew, l.yScrewStart - l.yFixedY, l.yScrewStart - l.yFloatingY, 100, 40, null);
		var yp:Array<AssemblyFrame> = [];
		for (side in [-1, 1]) {
			var rail = orient(side * f.yRailSpacing / 2, f.yRailStart, f.railPlane + l.block.spec.railHeight, [0, 0, 1], [0, 1, 0]);
			attach('railY${side < 0 ? "Left" : "Right"}', LinearRail.metric(l.profile, f.yRailLength), rail, "fixedPedestalY");
			for (span in [-l.yBlockSpan, l.yBlockSpan]) yp.push(orient(
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
		saddle = new MillCrossSaddle(new ComponentValues().setNumber("bore", 58).setNumber("boreZ", 0).setNumber("depth", l.saddleDepth).setBoolean("sidePockets", true));
		attach("saddle", saddle, F.translation(0, l.centreY, l.saddleBottom), "blockY0");
		for (i in 1...4) attach('blockY$i', l.block, yp[i], "saddle");
		var yFace = l.centreY - l.saddleDepth / 2;
		attach("nutBracketY", new MillYNutMount(),
			F.compose(y, F.translation(0, 0, l.yScrewStart - yFace)), "saddle");
		attach("nutY", l.nut, F.compose(y, F.translation(0, 0, l.yScrewStart - yFace - l.nut.bodyLength - l.nut.flangeThickness)), "nutBracketY");
		var x = orient(l.xScrewStart, l.centreY, l.xScrewZ, [0, 0, 1], [1, 0, 0]);
		foldedXDrive(x);
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
			for (span in [-l.xBlockSpan, l.xBlockSpan]) xp.push(orient(
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
		table = new MillCrossTable(l.tableDepth, l.xRailSpacing);
		attach("table", table, F.translation(0, l.centreY, l.tableBottom), "blockX0");
		for (i in 1...4) attach('blockX$i', l.block, xp[i], "table");
		movingNut("X", x, -l.xScrewStart, "table", true);
		for (a in ["X", "Y"]) {
			var id = a.toLowerCase(), along = a == "X" ? [1.0, 0.0, 0.0] : [0.0, -1.0, 0.0];
			addTransmission('$id-lead', id, 'screw$a-turn', Transmission.LeadScrew('screw$a', 'nut$a'), screwSense(id, along));
			supportScrew('$id-lead', l.fixed.support, l.floating.support);
		}
		// Nominal bearing bolts: 43 mm housing + 12 mm pedestal engagement.
		for (a in ["X", "Y"]) for (end in ["fixed", "floating"]) for (side in [-1, 1]) {
			var bearing = zeroPose('${end}Bearing$a');
			var centre = F.transformPoint(bearing, side * 23, 18, 0);
			attach('bearingBolt$a$end$side', SocketHeadCapScrew.metric("M5", 55), F.translation(centre.x, centre.y, centre.z), '${end}Bearing$a');
		}
		for (side in [-1, 1]) for (height in [10, 70]) attach('motorPlateBoltX$side$height', SocketHeadCapScrew.metric("M6", 25),
			orient(-207, l.centreY + side * 32, l.saddleBottom + height, [0,0,1], [-1,0,0]), "motorMountX");
		attach("motorCoverY", new MillDriveHatch(new ComponentValues().setNumber("lowerZ", l.yHatchLowerZ).setNumber("upperZ", l.yHatchUpperZ)), F.translation(0, l.yServiceRear + 2, 0), "fixedPedestalY");
		for (side in [-1, 1]) for (row in [{name:"Lower",z:l.yHatchLowerZ},{name:"Upper",z:l.yHatchUpperZ}]) attach('rearCoverBoltY$side${row.name}' , SocketHeadCapScrew.metric("M4", 10),
			orient(side * 75, l.yServiceRear + 4, row.z, [0,0,1], [0,1,0]), "motorCoverY");
		attach("motorCoverX", new MillMotorCover(new ComponentValues().setNumber("width", 84).setNumber("depth", 38).setNumber("height", 142)),
			F.fromRotationMatrix(-226, l.centreY, l.saddleBottom, [0,1,0, -1,0,0, 0,0,1]), "motorMountX");
		exposeConnector("mount", "fixedPedestalY", "mount");
		exposeConnector("table", "table", "top");
	}

	/** Return the X motor into the saddle instead of widening the entire base for a direct-drive motor. */
	function foldedXDrive(shaft:AssemblyFrame):Void {
		var l = layout;
		for (end in ["fixed", "floating"]) {
			var fixed = end == "fixed", bearing = fixed ? l.fixed : l.floating, x = fixed ? l.xFixedX : l.xFloatingX;
			attach('${end}PedestalX', new MillScrewPedestal(new ComponentValues().setNumber("width", 84)
				.setNumber("depth", bearing.length).setNumber("height", 15).setBoolean("motorAnchors", false)),
				F.fromRotationMatrix(x, l.centreY, l.xRailPlane, [0,-1,0, 1,0,0, 0,0,1]), "saddle");
			attach('${end}BearingX', bearing, orient(x, l.centreY, l.xScrewZ, [0,0,1], [1,0,0]), '${end}PedestalX');
		}
		attach("motorMountX", new MillBeltMotorPlate(), F.fromRotationMatrix(-201, l.centreY, l.saddleBottom,
			[0,-1,0, 1,0,0, 0,0,1]), "saddle");
		var motor = orient(-195, l.centreY, l.xMotorZ, [0,0,1], [-1,0,0]);
		attach("motorX", l.motor, motor, "motorMountX");
		hang("screwX", l.xScrew, shaft, "fixedBearingX");
		attach("screwRetainerX", l.xScrew.retainer(), F.compose(shaft, F.translation(0, 0, machinekit.motion.BallScrewRetainer.STATION)), "screwX");
		addMateOnAxis("screwX-turn", "continuous", "fixedBearingX", "to-screwX", "screwX", "attach-screwX", {x:1,y:0,z:0});
		var screwPulley = new TimingPulley(HTD5M, 20, 12, 15), motorPulley = new TimingPulley(HTD5M, 20, 14, 15);
		attach("screwPulleyX", screwPulley, orient(-225, l.centreY, l.xScrewZ, [0,0,1], [1,0,0]), "screwX");
		hang("motorPulleyX", motorPulley, orient(-225, l.centreY, l.xMotorZ, [0,0,1], [1,0,0]), "motorX");
		addMateOnAxis("motorX-turn", "continuous", "motorX", "to-motorPulleyX", "motorPulleyX", "attach-motorPulleyX", {x:-1,y:0,z:0});
		attach("motorShaftX", l.motor.shaft(), motor, "motorPulleyX");
		var belt = TimingBelt.twoPulley(HTD5M, 20, 20, l.xScrewZ-l.xMotorZ, 15, 5000);
		attach("beltX", belt, F.fromRotationMatrix(-225, l.centreY, l.xMotorZ, [0,0,1, 0,-1,0, 1,0,0]), "saddle");
		addBeltPath({belt:"beltX", wraps:[{instanceId:"motorPulleyX", connectorName:"front"}, {instanceId:"screwPulleyX", connectorName:"front"}]});
		addTransmission("x-return-belt", "screwX-turn", "motorX-turn", Transmission.BeltReduction("beltX", "screwPulleyX", "motorPulleyX"), Same);
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
		attach('screwRetainer$a', screw.retainer(), F.compose(axis, F.translation(0, 0, machinekit.motion.BallScrewRetainer.STATION)), 'screw$a');
		attach('coupling$a', l.coupling, F.compose(axis, F.translation(0, 0, -l.coupling.length / 2)), 'screw$a');
	}
}
