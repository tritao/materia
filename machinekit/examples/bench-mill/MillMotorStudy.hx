import haxe.io.Bytes;
import haxe.Json;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.TypedProperty;
import machinekit.assembly.*;
import machinekit.assembly.Diagnostics.Diagnostic;
import machinekit.component.*;
import machinekit.document.MachineKitDocuments;
import machinekit.document.MachineKitRecipes;
import machinekit.milling.*;
import machinekit.motion.ServoMotor;
import machinekit.transmission.TimingBelt;
import materia.project.SceneArtifact;

typedef MillMotorReport = {
	var status:String;
	var motor:String;
	var motorTeeth:Int;
	var spindleTeeth:Int;
	var spindleToMotorSpeedRatio:Float;
	var motorRpmAt8000:Float;
	var motorMaximumRpm:Float;
	var ratedPointOutputTorqueNmAtRatedMotorSpeed:Float;
	var continuousCurveQualified:Bool;
	var transmissionEfficiencyAssumed:Float;
	var motorShaftDiameterMm:Float;
	var motorPulleyBoreMm:Float;
	var shaftCoversPulleyWidth:Bool;
	var carrierFeetAtHeadTop:Bool;
	var motorMountFaceGaugeZmm:Float;
	var carrierWidthMm:Float;
	var carrierRearMm:Float;
	var motorCentreDistanceMm:Float;
	var beltPitchLengthMm:Float;
	var beltTeeth:Int;
	var beltSlackMm:Float;
	var beltSelection:String;
	var movingZMassKg:Float;
	var baselineMovingZMassKg:Float;
	var addedMovingMassKg:Float;
	var additionalGravityForceN:Float;
	var additionalStaticScrewTorqueNm:Float;
	var loadCalculationLimits:String;
	var payloadClearanceFindings:Array<Diagnostic>;
	var motorRemovalFindings:Array<Diagnostic>;
	var tensioning:String;
	var shaftTorqueTransfer:String;
};

/** Visible, recipe-editable motor substitution in the actual mechanical mill. */
class MillMotorStudy {
	public static function context():{reference:BenchMill,frame:MillFrame} {
		var reference=new BenchMill(false,false),f=MillFramePreview.layout(reference),z=MillFramePreview.zLayout(reference,f);
		return {reference:reference,frame:new MillFrame(f,z,new MillXYStageLayout(f,z.spindleY,BenchMill.RAIL))};
	}
	public static function selection(frame:MillFrame, name:String="ELM2M-2900LD180FT-H", motorTeeth:Int=96, spindleTeeth:Int=18):MillSpindleDrivePart {
		var z:MillZAxisLayout=frame.zAxis;
		return new MillSpindleDrivePart(new ComponentValues().setToken("motor",name)
			.setInteger("motorTeeth",motorTeeth).setInteger("spindleTeeth",spindleTeeth)
			.setNumber("faceZ",251+machinekit.motion.ServoMotor.model(name).rating.shaftLength)
			.setNumber("mountBottom",Bt30Cartridge.SEAT+z.headHeight));
	}
	static function translating(id:String,parents:Map<String,String>):Bool {
		var seen:Map<String,Bool> =[];
		while(id!=null && !seen.exists(id)) {
			if(id=="zAxis/blockLeftLower") return true;
			seen.set(id,true);id=parents.get(id);
		}
		return false;
	}
	public static function movingMass(assembly:MachineAssembly):Float {
		var parents=assembly.mateParents(),total=0.0;
		for(m in assembly.components()) if(translating(m.id,parents)) total+=m.component.massProperties().mass;
		return total;
	}
	/** These are CAD diagnostics, not a claim of sufficient stiffness or motor duty. */
	public static function report(assembly:MachineAssembly, frame:MillFrame, drive:MillSpindleDrivePart, baselineMass:Float):MillMotorReport {
		var z:MillZAxisLayout=frame.zAxis,motor=drive.motor(),mass=movingMass(assembly),delta=mass-baselineMass;
		var belt=cast(assembly.components().filter(m->m.id=="spindle/belt")[0].component,TimingBelt);
		var payload=MillSpindleIntegrity.checkPayload(assembly,z),removal=MillSpindleIntegrity.checkMotorRemoval(assembly);
		var relation=machinekit.motion.LeadScrew.relation(z.screw,z.nut,1);
		return {status:"packaging study; hardware and manufacturing qualification pending",motor:motor.designation,
			motorTeeth:drive.motorTeeth(),spindleTeeth:drive.spindleTeeth(),spindleToMotorSpeedRatio:drive.speedRatio(),
			motorRpmAt8000:8000/drive.speedRatio(),motorMaximumRpm:motor.rating.maxSpeed*60/(2*Math.PI),
			ratedPointOutputTorqueNmAtRatedMotorSpeed:motor.rating.ratedTorque*0.95/drive.speedRatio(),
			continuousCurveQualified:false,transmissionEfficiencyAssumed:0.95,
			motorShaftDiameterMm:motor.rating.shaftDiameter,motorPulleyBoreMm:drive.motorPulley().boreDiameter,
			shaftCoversPulleyWidth:drive.face()-motor.rating.shaftLength<=251 && drive.face()>=269,
			carrierFeetAtHeadTop:Math.abs(drive.parameterNumber("mountBottom")-(Bt30Cartridge.SEAT+z.headHeight))<1e-7,
			motorMountFaceGaugeZmm:drive.face(),carrierWidthMm:drive.plateWidth(),carrierRearMm:drive.plateRear(),motorCentreDistanceMm:drive.centres(),
			beltPitchLengthMm:belt.length,beltTeeth:belt.teeth,beltSlackMm:belt.slack(),
			beltSelection:"nearest whole-tooth pitch length; supplier availability, width, tension and power capacity unqualified",
			movingZMassKg:mass,baselineMovingZMassKg:baselineMass,addedMovingMassKg:delta,
			additionalGravityForceN:delta*9.80665,
			additionalStaticScrewTorqueNm:delta*9.80665/(1000*Math.abs(relation.ratio)*relation.efficiency),
			loadCalculationLimits:"incremental gravity only; excludes acceleration, cutting, brake holding qualification and guide moments",
			payloadClearanceFindings:payload.items,motorRemovalFindings:removal.items,
			tensioning:"±5 mm motor slots; tension screw, belt pretension and retention need design",
			shaftTorqueTransfer:"motor keyway modeled; pulley keyway/clamping, installed key and axial retention remain undesigned"};
	}
	public static function preview(?savedDocument:String):Bytes {
		MachineKitRecipes.register();
		var c=context(),document=new Document();
		try {
			var definition=MachineKitDocuments.define(document,MillSpindleDrivePart.recipeType(),selection(c.frame).values());
			var instance=document.createInstance("spindle/motorCarrier",definition);
			instance.restoreProperty("machinekit.occurrence",TypedProperty.text("machinekit.occurrence","spindle/motorCarrier"));
			var diagnostics:Array<String> =[];
			if(savedDocument!=null) MachineKitRecipes.reconcileDocument(document,savedDocument,diagnostics);
			var drive=cast(MachineKitRecipes.component(instance),MillSpindleDrivePart);
			var assembly=MillFramePreview.concept(c.reference,c.frame,0,drive);
			var baseline=MillFramePreview.concept(c.reference,c.frame);
			var study=report(assembly,c.frame,drive,movingMass(baseline));
			for(item in study.payloadClearanceFindings) diagnostics.push(item.message);
			for(item in study.motorRemovalFindings) diagnostics.push(item.message);
			if(study.shaftCoversPulleyWidth!=true)diagnostics.push("Motor shaft does not span the pulley: adjust motor face height before using this layout.");
			if(study.carrierFeetAtHeadTop!=true)diagnostics.push("Carrier foot datum differs from the head top.");
			diagnostics.push('Moving Z mass ${study.movingZMassKg} kg; added ${study.addedMovingMassKg} kg. Drive remains unqualified.');
			var scene=AssemblyPreview.scene(assembly,"mill-selectable-spindle-drive");
			scene.recipeDocument=DocumentCodec.encode(document);scene.recipeDiagnostics=diagnostics;
			var bytes=SceneArtifact.encode(scene);MachineKitRecipes.forget(document);document.close();return bytes;
		} catch(e:Dynamic) {MachineKitRecipes.forget(document);document.close();throw e;}
	}
	public static function main():Void {
		var c=context(),baseline=MillFramePreview.concept(c.reference,c.frame),baselineMass=movingMass(baseline);
		var directory=Sys.getEnv("MILL_MOTOR_STUDY_OUTPUT");if(directory==null) directory=Sys.getCwd()+"/build/motor-study";
		if(!sys.FileSystem.exists(directory)) sys.FileSystem.createDirectory(directory);
		var reports:Array<MillMotorReport> =[];
		for(p in [{id:"1500w",name:"ELM2L-1500LA100FT-H MS1AE",motor:80,spindle:30},
			{id:"2000w",name:"ELM2M-2000LB130F-H",motor:80,spindle:25},
			{id:"2900w",name:"ELM2M-2900LD180FT-H",motor:96,spindle:18}]) {
			var drive=selection(c.frame,p.name,p.motor,p.spindle),assembly=MillFramePreview.concept(c.reference,c.frame,0,drive);
			assembly.validate();
			var result=report(assembly,c.frame,drive,baselineMass);reports.push(result);
			sys.io.File.saveContent(directory+"/motor-"+p.id+".json",Json.stringify(result,null,"  "));
			sys.io.File.saveBytes(directory+"/mill-motor-"+p.id+".mtrg",SceneArtifact.encode(AssemblyPreview.scene(assembly,"mill-motor-"+p.id)));
			Sys.println('Motor study ${p.id}: added Z mass ${result.addedMovingMassKg} kg');
		}
		sys.io.File.saveContent(directory+"/motor-study.json",Json.stringify({candidates:reports},null,"  "));
		Sys.println('Motor study written to $directory');
	}
}
