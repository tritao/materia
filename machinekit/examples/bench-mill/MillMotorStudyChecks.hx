import machinekit.milling.*;
import machinekit.assembly.*;
import machinekit.component.*;
import cadkit.modeling.Part;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames as F;

class MillMotorStudyChecks {
	static function near(a:Float,b:Float,label:String):Void {if(!Math.isFinite(a)||Math.abs(a-b)>1e-6)throw '$label: $a != $b';}
	public static function main():Void {
		var c=MillMotorStudy.context();
		// Mount consumers use the shared contract without knowing the vendor type.
		for(name in machinekit.motion.LeadshineServo.catalog().designations()) {
			var source=machinekit.motion.LeadshineServo.model(name);
			var motor:machinekit.motion.ServoMotor=source,mount=motor.mounting();
			if(mount==null)throw 'Missing mounting interface: $name';
			near(mount.face,source.spec.face,"catalog mounting face");
			near(mount.pilotDiameter,source.spec.pilotDiameter,"catalog pilot");
			near(mount.holes.length,4,"catalog mounting holes");
		}
		var generic=machinekit.motion.ServoMotor.model("GENERIC-SERVO-400W");
		if(generic.mounting()!=null)throw "Unspecified mounting must remain unknown";
		var sweep=generic.axialEnvelope(-50,10,true),bounds=PosedParts.boxOf(sweep);
		near(bounds.minZ,-generic.rating.bodyLength-50,"generic sweep lower limit");
		near(bounds.maxZ,generic.rating.shaftLength+10,"generic sweep shaft limit");
		sweep.close();

		for(p in [{name:"ELM2L-1500LA100FT-H MS1AE",motor:80,spindle:30},
			{name:"ELM2M-2000LB130F-H",motor:80,spindle:25},
			{name:"ELM2M-2900LD180FT-H",motor:96,spindle:18}]) {
			var drive=MillMotorStudy.selection(c.frame,p.name,p.motor,p.spindle);
			var head=new MillSpindleHead(c.frame.zAxis,0,drive),parts=head.components(),poses=head.solvedPoses();
			head.validate();
			var plate=drive.geometry(),guard=new MillSpindleDrivePart(drive.values().setToken("role","guard")).geometry();
			if(plate.solidCount()!=1 || guard.solidCount()!=1) throw "Carrier and guard must each be a connected solid";
			var belt=cast(parts.filter(m->m.id=="belt")[0].component,machinekit.transmission.TimingBelt);
			near(belt.slack(),0,"unequal pulley path fits integer belt teeth");
			near(drive.motorPulley().boreDiameter,drive.motor().rating.shaftDiameter,"selected shaft defines pulley bore");
			if(Math.abs(drive.centres()-drive.parameterNumber("centres"))>5)throw "Whole-tooth fit exceeds slots";
			for(hole in drive.mountingHoles()) {
				var probe=Part.cylinderSpan(drive.holeDiameter()*0.4,drive.face()-11,drive.face()-1,-hole.x,drive.centres()+hole.y);
				var intersection=plate.intersect(probe);near(intersection.volume(),0,"plate bores match actual bolt centres");
				intersection.close();probe.close();
			}
			var cache=new PosedParts();
			var housing=parts.filter(m->m.id=="motor")[0].component;
			near(cache.overlap(housing,poses.get("motor"),drive,poses.get("motorCarrier")),0,"motor pilot seats in adapter recess");
			var shaft=parts.filter(m->m.id=="motorShaft")[0].component;
			near(cache.overlap(shaft,poses.get("motorShaft"),drive,poses.get("motorCarrier")),0,"shaft clears adapter");
			cache.close();plate.close();guard.close();
			var rebuilt=MachineAssembly.decode(head.encode(),MachineKitComponents.standard());
			var model=new AssemblyModel("mm");rebuilt.addTo(model,"");
			var state=new AssemblyState(model.definition("selected-drive"));
			state.setJoint("rotor-turn",1.25);state.forwardKinematics();
			near(state.joint("motor-turn"),1.25/drive.speedRatio(),"saved assembly retains pulley ratio");
			var recreated=cast(drive.componentType().create(drive.values()),MillSpindleDrivePart);
			near(recreated.centres(),drive.centres(),"carrier recipe retains drive settings");
			Sys.println('Selected drive ${p.name}: geometry and reconstruction passed');
		}
		var rejected=false;
		try MillMotorStudy.selection(c.frame,"ELM2M-2900LD180FT-H",8,18) catch(_:Dynamic) rejected=true;
		if(!rejected)throw "Pulley with insufficient material around bore must be rejected";
		// Both old and configured assemblies must still use a 20 mm spindle pulley bore.
		var baseline=new MillSpindleHead(c.frame.zAxis),basePulley=cast(baseline.components().filter(m->m.id=="spindlePulley")[0].component,machinekit.transmission.TimingPulley);
		near(basePulley.teeth,40,"legacy head retains 40T drive");
		Sys.println("Selectable mill drive checks passed");
	}
}
