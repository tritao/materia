package machinekit.milling;

import machinekit.assembly.MachineAssembly;
import machinekit.component.*;
import machinekit.assembly.Transmission;
import machinekit.assembly.Sense;
import machinekit.transmission.TimingPulley;
import machinekit.transmission.TimingBelt;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** ATC-ready spindle packaging, independent of the legacy ER20 control model.
 * Both release joints are inspection motions, not a pneumatic control system.
 * Supplier release force/stroke, spring sizing and orientation feedback are pending. */
class MillSpindleHead extends MachineAssembly {
	public final gauge:AssemblyFrame;
	public function new(l:MillZAxisLayout,?motorAdjustment:Float) {
		super();
		// A 540 mm / 108-tooth belt with equal 40T pulleys has 170 mm centres.
		var adjustment:Float = motorAdjustment ?? (Math.sqrt(170*170-78*78)-150);
		if (!Math.isFinite(adjustment) || Math.abs(adjustment)>8) throw "Spindle belt adjustment exceeds its slots";
		gauge = F.translation(0,l.spindleY,l.headBottom-Bt30Cartridge.SEAT);
		addComponent("spindle",new Bt30Cartridge(),gauge);
		rotating("rotor",new MillPowerDrawbarPart(new ComponentValues().setToken("role","rotor")),gauge,"spindle","rotor-turn");
		for (role in ["springs","pullStud","holder","reactionFrame","cylinder"])
			fixed(role,new MillPowerDrawbarPart(new ComponentValues().setToken("role",role)),gauge,
				(role=="reactionFrame" || role=="cylinder")?"spindle":"rotor");
		moving("drawbar",new MillPowerDrawbarPart(new ComponentValues().setToken("role","drawbar")),gauge,"rotor",4.5);
		fixed("gripper",new MillPowerDrawbarPart(new ComponentValues().setToken("role","gripper")),gauge,"drawbar");
		moving("pistonRod",new MillPowerDrawbarPart(new ComponentValues().setToken("role","pistonRod")),gauge,"cylinder",6.5);
		fixed("piston",new MillPowerDrawbarPart(new ComponentValues().setToken("role","piston")),gauge,"pistonRod");
		for (i in 0...6) { var a=i*Math.PI/3;
			fixed('collarBolt$i',SocketHeadCapScrew.metric("M6",30),F.fromRotationMatrix(53.5*Math.cos(a),l.spindleY+53.5*Math.sin(a),gauge.z+40,[1,0,0, 0,-1,0, 0,0,-1]),"spindle");
		}
		for (p in [[-37.5,0.0],[37.5,0.0],[0.0,-37.5],[0.0,37.5]]) fixed('reactionBolt${p[0]}-${p[1]}',SocketHeadCapScrew.metric("M5",12),F.translation(p[0],l.spindleY+p[1],gauge.z+244.6),"reactionFrame");
		for (x in [-50.0,50.0]) for (y in [-50.0,50.0]) fixed('cylinderBolt$x-$y',SocketHeadCapScrew.metric("M8",16),F.fromRotationMatrix(x,l.spindleY+y,gauge.z+322,[1,0,0, 0,-1,0, 0,0,-1]),"reactionFrame");
		var motor = new Bt30SpindleMotor();
		var face = gauge.z+282, mountBottom = l.headBottom+l.headHeight;
		fixed("motorCarrier",new MillSpindleMotorMount(new ComponentValues().setNumber("height",face-12-mountBottom)),F.translation(0,l.spindleY,mountBottom),"spindle");
		var motorPose = F.fromRotationMatrix(150+adjustment,l.spindleY+78,face,[-1,0,0, 0,1,0, 0,0,-1]);
		fixed("motor",motor,motorPose,"motorCarrier");
		rotating("motorShaft",motor.shaft(),motorPose,"motor","motor-turn");
		var pulley = new TimingPulley(HTD5M,40,20,18);
		fixed("spindlePulley",pulley,F.translation(0,l.spindleY,gauge.z+251),"rotor");
		fixed("motorPulley",pulley,F.translation(150+adjustment,l.spindleY+78,gauge.z+251),"motorShaft");
		fixed("tensionScrew",SocketHeadCapScrew.metric("M6",27-adjustment),F.fromRotationMatrix(232,l.spindleY+78,gauge.z+288,[0,0,1, 1,0,0, 0,1,0]),"motorCarrier");
		fixed("beltGuard",new MillSpindleBeltGuard(),gauge,"motorCarrier");
		for (p in [[94.0,108.0],[165.0,121.0]]) fixed('guardBolt${p[0]}',SocketHeadCapScrew.metric("M4",10),
			F.fromRotationMatrix(p[0],l.spindleY+p[1],gauge.z+268,[1,0,0, 0,-1,0, 0,0,-1]),"beltGuard");
		var dx=150.0+adjustment,dy=78.0,centres=Math.sqrt(dx*dx+dy*dy),angle=Math.atan2(dy,dx);
		fixed("belt",TimingBelt.twoPulley(HTD5M,40,40,centres,15,5000),
			F.fromRotationMatrix(0,l.spindleY,gauge.z+252.5,[Math.cos(angle),-Math.sin(angle),0, Math.sin(angle),Math.cos(angle),0, 0,0,1]),"spindle");
		for (y in [58.0,88.0]) fixed('carrierBolt$y',SocketHeadCapScrew.metric("M6",20),F.translation(55,l.spindleY+y,mountBottom+10),"motorCarrier");
		for (x in [120.0,180.0]) for (y in [48.0,108.0]) fixed('motorBolt$x-$y',SocketHeadCapScrew.metric("M5",20),F.fromRotationMatrix(x+adjustment,l.spindleY+y,face-12,[1,0,0, 0,-1,0, 0,0,-1]),"motorCarrier");
		addMemberConnector("spindlePulley","beltPlane",Solids.axial(0,0,1.5));
		addMemberConnector("motorPulley","beltPlane",Solids.axial(0,0,1.5));
		addBeltPath({belt:"belt",wraps:[{instanceId:"spindlePulley",connectorName:"beltPlane"},{instanceId:"motorPulley",connectorName:"beltPlane"}]});
		addTransmission("spindle-belt","rotor-turn","motor-turn",Transmission.BeltReduction("belt","spindlePulley","motorPulley"),Same);
		exposeConnector("mount","spindle","mount");
	}
	function rotating(id:String,part:MachineComponent,pose:AssemblyFrame,parent:String,joint:String):Void {
		addComponent(id,part,pose);
		var positions=solvedPoses(),frame=F.translation(pose.x,pose.y,pose.z);
		addMemberConnector(parent,'to-$id',F.compose(F.inverse(positions.get(parent)),frame));
		addMemberConnector(id,"seat",F.compose(F.inverse(pose),frame));
		addMateOnAxis(joint,"continuous",parent,'to-$id',id,"seat",{x:0,y:0,z:1},0,
			{lower:null,upper:null,velocity:8000*2*Math.PI/60,effort:null});
	}

	function fixed(id:String,part:MachineComponent,pose:AssemblyFrame,parent:String):Void {
		addComponent(id,part,pose);
		var positions = solvedPoses();
		addMemberConnector(parent,'to-$id',F.compose(F.inverse(positions.get(parent)),pose));
		addMemberConnector(id,"seat",F.identity());
		addMate('attach-$id',"fixed",parent,'to-$id',id,"seat");
	}
	function moving(id:String,part:MachineComponent,pose:AssemblyFrame,parent:String,stroke:Float):Void {
		addComponent(id,part,pose);
		var positions = solvedPoses();
		addMemberConnector(parent,'to-$id',F.compose(F.inverse(positions.get(parent)),pose));
		addMemberConnector(id,"seat",F.identity());
		addMateOnAxis('$id-release',"prismatic",parent,'to-$id',id,"seat",{x:0,y:0,z:-1},0,
			{lower:0,upper:stroke,velocity:null,effort:null});
	}
}
