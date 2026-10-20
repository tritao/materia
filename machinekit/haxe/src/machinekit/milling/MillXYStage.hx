package machinekit.milling;

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
class MillXYStage extends MillGuidedXYStage {
	public final layout:MillXYStageLayout;

	public function new(layout:MillXYStageLayout) {
		super(requireGuides(layout));
		this.layout = layout;
		var l = layout;
		var y = orient(0, l.yScrewStart, l.yScrewZ, [0, 0, 1], [0, -1, 0]);
		if(l.supplierY)supplierYDrive(y);else drive("Y", y, l.yScrew, l.yScrewStart - l.yFixedY, l.yScrewStart - l.yFloatingY, 100, 40, "railYLeft");
		var yFace = l.centreY - l.saddleDepth / 2;
		var nutMountPose=F.compose(y,F.translation(0,0,l.yScrewStart-yFace));
		attach("nutBracketY",new MillYNutMount(new ComponentValues().setBoolean("reinforced",true).setBoolean("serviceAccess",true)),nutMountPose,"saddle");
		for(p in MillYNutMount.anchors()) attach('nutBracketYBolt${p.x}-${p.y}',SocketHeadCapScrew.metric("M6",MillYNutMount.ANCHOR_LENGTH),
			F.compose(nutMountPose,F.translation(p.x,p.y,MillYNutMount.ANCHOR_SEAT)),"nutBracketY");
		var nutBolt=0;
		for(p in l.nut.boltPattern()) attach('nutYBolt${nutBolt++}',SocketHeadCapScrew.metric(l.nut.mountScrew,MillYNutMount.FLANGE_BOLT_LENGTH),
			F.compose(nutMountPose,F.fromRotationMatrix(p.x,p.y,-l.nut.flangeThickness,[1,0,0,0,-1,0,0,0,-1])),"nutBracketY");
		attach("nutY", l.nut, F.compose(y, F.translation(0, 0, l.yScrewStart - yFace - l.nut.bodyLength - l.nut.flangeThickness)), "nutBracketY");
		var x = orient(l.xScrewStart, l.centreY, l.xScrewZ, [0, 0, 1], [1, 0, 0]);
		foldedXDrive(x);
		movingNut("X", x, -l.xScrewStart, "table", true);
		for (a in ["X", "Y"]) {
			var id = a.toLowerCase(), along = a == "X" ? [1.0, 0.0, 0.0] : [0.0, -1.0, 0.0];
   if(a=="Y"&&l.supplierY) {
    // HZMotion publishes load capacity but no axial rigidity. Keep CAD kinematics
    // explicit rather than borrowing the generic BK12 stiffness for this unit.
    addCoupling("y-lead","y","screwY-turn",2*Math.PI/l.yDriveScrew.thread.signedLead());
   }else {
    addTransmission('$id-lead', id, 'screw$a-turn', Transmission.LeadScrew('screw$a', 'nut$a'), screwSense(id, along));
    supportScrew('$id-lead', l.fixed.support, l.floating.support);
   }
		}
		// Nominal bearing bolts: 43 mm housing + 12 mm pedestal engagement.
		for (a in ["X", "Y"]) if(a!="Y"||!l.supplierY) for (end in ["fixed", "floating"]) for (side in [-1, 1]) {
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
		MillYNutDesignChecks.declare(this);
	}

 static function requireGuides(layout:MillXYStageLayout):MillGuidedXYStage.MillGuidedXYLayout {
  if(layout==null)throw "XY stage needs a layout";
  return layout.guided();
 }

 /** Selected Leadshine/HZMotion Y drive; the X return drive retains its own layout. */
 function supplierYDrive(shaft:AssemblyFrame):Void {
  var l=layout;
  attach("fixedPedestalY",new MillYDriveSeat(false),F.translation(0,l.yFixedY,l.yMountPlane),"railYLeft");
  attach("floatingPedestalY",new MillYDriveSeat(true),F.translation(0,l.yFloatingY,l.yMountPlane),"fixedPedestalY");
  var mountPose=orient(0,l.yMotorFace,l.yScrewZ-25,[0,0,1],[0,-1,0]);
  var mount=new machinekit.assembly.HzMotionComponent("M60D-BK12-C",null,"unspecified","supplier-step");
  attach("motorMountY",mount,mountPose,"fixedPedestalY");
  var motorPose=orient(0,l.yMotorFace,l.yScrewZ,[0,0,1],[0,-1,0]);
  attach("motorY",l.yMotor,motorPose,"motorMountY");
  hang("screwY",l.yDriveScrew,shaft,"motorMountY");
  addMateOnAxis("screwY-turn","continuous","motorMountY","to-screwY","screwY","attach-screwY",{x:0,y:-1,z:0});
  attach("motorShaftY",l.yMotor.shaft(),motorPose,"screwY");
  attach("couplingY",new ShaftCoupling(14,10,25,32,[{z:8,angle:Math.PI/2},{z:24,angle:Math.PI/2}]),F.compose(motorPose,F.translation(0,0,16)),"screwY");
  var support=new machinekit.assembly.HzMotionComponent("BF12",null,"unspecified","supplier-step","installed-envelope");
  attach("floatingBearingY",support,orient(0,l.yFloatingY+10,l.yScrewZ-25,[0,0,1],[0,-1,0]),"floatingPedestalY");
  for(floating in [false,true])for(x in [-42.0,42])attach('seatYAnchor${floating}$x',SocketHeadCapScrew.metric("M6",25),
   F.translation(x,(floating?l.yFloatingY:l.yFixedY)+(floating?0:-10),l.yMountPlane+15),floating?"floatingPedestalY":"fixedPedestalY");
  var i=0;
  for(hole in mount.spec.baseHoles)attach('mountYBolt${++i}',SocketHeadCapScrew.metric("M6",35),
   F.translation(hole.x,l.yMotorFace-hole.z,l.yScrewZ-1),"motorMountY");
  for(x in [-23.0,23])attach('bearingBoltYfloating$x',SocketHeadCapScrew.metric("M6",40),
   F.translation(x,l.yFloatingY,l.yScrewZ+6),"floatingBearingY");
  i=0;
  var mounting:machinekit.motion.ServoMotor.ServoMounting=l.yMotor.mounting();
  for(hole in mounting.holes)attach('motorYBolt${++i}',SocketHeadCapScrew.metric("M5",12),
   F.compose(motorPose,F.fromRotationMatrix(hole.x,hole.y,-mounting.flangeThickness,[1,0,0,0,-1,0,0,0,-1])),"motorMountY");
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
