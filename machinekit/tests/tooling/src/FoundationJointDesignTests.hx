import materia.casting.FoundationJoint;
import materia.casting.FoundationJoint.FoundationJoints;
import materia.casting.InterfaceRequirement;
import materia.assembly.AssemblyFrames as F;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import machinekit.structural.InsertHoles;
import machinekit.structural.MineralFoundation;
import machinekit.design.AssemblyDesignChecks;
import machinekit.design.DesignRequirement;

/** Synthetic nominal joint, not a production gap or hardware selection. */
class FoundationJointDesignTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {assertions++;if (!ok) throw message;}
	static function joint():FoundationJoint return {
		id:"column-base",lowerBody:"lower/casting",upperBody:"upper/casting",route:FoundationJointRoute.Grouted,gapMm:2,gapToleranceMm:0.1,
		seats:[{id:"seat",lower:{member:"lower/seat",feature:"seat"},upper:{member:"upper/seat",feature:"seat"}}],
		jacks:[{id:"jack",screw:"jack",receiver:{member:"block",feature:"fixing"},bearing:{member:"upper/seat",feature:"seat"}}],
		anchors:[{id:"anchor",screw:"anchor",target:{member:"lower/seat",feature:"fixing"},
			through:[{member:"upper/seat",feature:"fixing"}]}],cureBeforeTorque:true
	};
	static function seat(width:Float,holes:String):CastInsert {
		var requirement:InterfaceRequirement={id:"seat",geometry:Plane(F.translation(0,0,20)),route:InterfaceRoute.AsSupplied,allowanceMm:0,finishingProcessErrorMm:null,
			permittedFinishes:[InterfaceFinish.AsSupplied],positionToleranceMm:0.01,datumReference:"primary",relationshipIds:[]};
		return new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(width,40,20),holes)
			.setToken("role",materia.casting.InsertSpec.InsertRole.ColumnMount)
			.setToken("finish",materia.casting.InsertSpec.InsertFinish.AsSupplied)
			.setToken("interfaces",haxe.Json.stringify(materia.casting.InterfaceRequirement.InterfaceRequirements.records([requirement]))));
	}
	static function fixture(definition:FoundationJoint,upperZ:Float=42):MachineAssembly return model(definition,upperZ).assembly;
	static function model(definition:FoundationJoint,upperZ:Float=42,jackX:Float=70):{assembly:MachineAssembly,lower:MineralFoundation} {
		var thread=InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(4).mating());
		var lower=new MineralFoundation(new ComponentValues().setNumber("width",100).setNumber("depth",80).setNumber("height",40),
			[{id:"seat",part:seat(40,InsertHoles.tappedBlind(thread,20)),pose:F.translation(0,0,20),
				openings:[materia.casting.CastingPatterns.face("body.z-max")]}]);
		var upper=new MineralFoundation(new ComponentValues().setNumber("width",180).setNumber("depth",80).setNumber("height",40),
			[{id:"seat",part:seat(180,InsertHoles.through(thread.clearanceDiameter,20)),pose:F.compose(F.translation(0,0,20),F.turnY(Math.PI)),
				openings:[materia.casting.CastingPatterns.face("body.z-min"),materia.casting.CastingPatterns.face("body.x-min"),
					materia.casting.CastingPatterns.face("body.x-max")]}]);
		var blockThread=InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(6).mating());
		var block=new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(40,40,10),InsertHoles.tappedThrough(blockThread,10))
			.setToken("role",materia.casting.InsertSpec.InsertRole.ColumnMount));
		var result=new MachineAssembly();result.include("lower",lower);result.include("upper",upper,F.translation(0,0,upperZ));
		result.addComponent("block",block,F.translation(jackX,0,20));
		result.addComponent("jack",machinekit.standard.SocketHeadCapScrew.metric("M6",30,machinekit.standard.PropertyClass.Steel88),
			F.compose(F.translation(jackX,0,12),F.turnY(Math.PI)));
		result.addComponent("anchor",machinekit.standard.SocketHeadCapScrew.metric("M4",30,machinekit.standard.PropertyClass.Steel88),F.translation(0,0,62));
		result.addDesignRequirement(FoundationConnection("joint-instance",definition));return {assembly:result,lower:lower};
	}
	static function hasCode(result:machinekit.assembly.Diagnostics,code:String):Bool
		return result.items.filter(item -> item.code==code).length>0;
	static function invalid(result:machinekit.assembly.Diagnostics,message:String):Bool
		return result.items.filter(item -> item.code=="design.invalid-requirement" && item.message.indexOf(message)>=0).length==1;
	static function rejects(fn:Void->Void,expected:String):Void {
		var message="";try fn() catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf(expected)>=0,'Expected "$expected", got "$message"');
	}
	static function drawingIntegrity(directory:String,fabrication:materia.tooling.export.ToolingFabrication.FabricationPlan,
			checks:Array<materia.tooling.ToolingValidation>):Void {
		var manifestPath=directory+"/manifest.json",manifestText=sys.io.File.getContent(manifestPath);
		var manifest:Dynamic=haxe.Json.parse(manifestText);
		check(manifest.version==10,"the section-drawing manifest has one new strict version");
		try {
			manifest.version=9;sys.io.File.saveContent(manifestPath,haxe.Json.stringify(manifest));
			rejects(() -> machinekit.tooling.ToolingPackageManifest.verify(directory),"Unsupported tooling manifest");
		} catch (error:Dynamic) {sys.io.File.saveContent(manifestPath,manifestText);throw error;}
		sys.io.File.saveContent(manifestPath,manifestText);
		var reportPath=directory+"/tooling-section-drawings.json",reportText=sys.io.File.getContent(reportPath);
		var report:Dynamic=haxe.Json.parse(reportText),records:Array<Dynamic> =cast report.drawings;
		var rendered=records.filter(record -> record.validation.status=="pass");
		check(rendered.length>0,"the package includes finished tooling section evidence");
		var name=machinekit.tooling.ToolingSectionReview.filename(rendered[0].id)+".svg";
		var svgPath=directory+"/"+name,svg=sys.io.File.getContent(svgPath);
		try {
			sys.io.File.saveContent(svgPath,svg+"<!-- changed -->\n");
			rejects(() -> machinekit.tooling.ToolingPackageManifest.write(directory,fabrication,checks),
				"Tooling drawing artifact differs from source");
		} catch (error:Dynamic) {sys.io.File.saveContent(svgPath,svg);throw error;}
		sys.io.File.saveContent(svgPath,svg);
		try {
			Reflect.setField(records[0],"sections",new Array<Dynamic>());
			sys.io.File.saveContent(reportPath,materia.tooling.ToolingCodec.canonical(report));
			rejects(() -> machinekit.tooling.ToolingPackageManifest.write(directory,fabrication,checks),
				"Tooling section drawings differ from finished source geometry");
		} catch (error:Dynamic) {sys.io.File.saveContent(reportPath,reportText);throw error;}
		sys.io.File.saveContent(reportPath,reportText);
		try {
			manifest=haxe.Json.parse(manifestText);
			var files:Array<Dynamic> =cast manifest.files;
			Reflect.setField(manifest,"files",files.filter(file -> file.path!=name));
			sys.io.File.saveContent(manifestPath,materia.tooling.ToolingCodec.canonical(manifest));
			rejects(() -> machinekit.tooling.ToolingPackageManifest.verify(directory),"Missing hashed tooling drawing");
		} catch (error:Dynamic) {sys.io.File.saveContent(manifestPath,manifestText);throw error;}
		sys.io.File.saveContent(manifestPath,manifestText);
		machinekit.tooling.ToolingPackageManifest.verify(directory);
	}

	static function exportJoint(plan:materia.tooling.casting.CastingMoldPlan):Void {
		var fabrication=materia.tooling.export.ToolingFabrication.plan(plan,FoundationContextTests.fabricationSpec());
		var directory=Sys.getCwd()+"/build/tooling/synthetic-foundation-joint-"+fabrication.fingerprint.substr(0,12);
		machinekit.tooling.ToolingFiles.directory(directory);
		sys.io.File.saveContent(directory+"/mold-plan.json",materia.tooling.casting.CastingMoldCodec.encode(plan));
		var checks=machinekit.tooling.ToolingFabricationExport.write(plan,fabrication,directory),full=fabrication.validation;
		fabrication.validation=full.filter(c -> !StringTools.startsWith(c.rule,"tooling.foundation-joint."));
		machinekit.tooling.ToolingPackageManifest.write(directory,fabrication,checks);
		fabrication.validation=full;machinekit.tooling.ToolingPackageManifest.verify(directory);
		drawingIntegrity(directory,fabrication,checks);
		var manifest:Dynamic=haxe.Json.parse(sys.io.File.getContent(directory+"/manifest.json"));
		var rows:Array<Dynamic> = cast manifest.validation;
		check(rows.filter(r -> StringTools.startsWith(Std.string(r.rule),"tooling.foundation-joint.")).length==18,
			"manifest recomputes and retains every source joint check even if the caller drops them");
		var path=directory+"/foundation-joints.json",original=sys.io.File.getContent(path);
		try {
			var omitted:Dynamic=haxe.Json.parse(original);Reflect.setField(omitted,"validation",new Array<Dynamic>());
			sys.io.File.saveContent(path,materia.tooling.ToolingCodec.canonical(omitted));
			rejects(() -> machinekit.tooling.ToolingPackageManifest.write(directory,fabrication,checks),
				"Foundation joint source or process validation omitted or changed");
		} catch (error:Dynamic) {sys.io.File.saveContent(path,original);throw error;}
		sys.io.File.saveContent(path,original);machinekit.tooling.ToolingPackageManifest.verify(directory);
		var inventory=sys.io.File.getContent(directory+"/foundation-joint-hardware.csv");
		check(inventory.indexOf("anchor")>=0 && inventory.indexOf("jack")>=0 && inventory.indexOf("iso4762-8.8")>=0,
			"fabrication inventory retains the source joint's real standard fasteners");
		var sequence=sys.io.File.getContent(directory+"/foundation-joint-sequence.csv");
		check(sequence.indexOf("cure evidence")<sequence.indexOf("Apply and inspect the approved final torque")
			&& sequence.indexOf("pending")>=0,"joint process sequence retains cure-before-final-torque without claiming physical evidence");
	}
	static function threadLimits(source:materia.casting.FoundationJointSources,definition:FoundationJoint):Void {
		var original=materia.casting.FoundationJointSources.FoundationJointSourceCodec.snapshot(source);
		var facts=materia.casting.FoundationJointSources.FoundationJointSourceCodec.snapshot(original),anchor=definition.anchors[0],jack=definition.jacks[0];
		function measure(receiver:Bool=false):Array<materia.casting.FoundationJointAnalysis.FoundationJointFinding> {
			var subject=receiver ? "jack-"+jack.id : "anchor-"+anchor.id;
			return materia.casting.FoundationJointAnalysis.check(facts,definition).filter(f -> f.subject==subject);
		}
		function passes(code:String,receiver:Bool=false):Bool
			return measure(receiver).filter(f -> f.code==code && f.passed).length==1;
		function reset():Void facts=materia.casting.FoundationJointSources.FoundationJointSourceCodec.snapshot(original);
		function screw():materia.casting.FoundationJointSources.FoundationJointScrew
			return materia.casting.FoundationJointAnalysis.screw(facts,anchor.screw);
		function hole():materia.casting.InsertHole return materia.casting.FoundationJointAnalysis.hole(facts,anchor.target);
		check(measure().filter(f -> !f.passed).length==0,"the source anchor meets its own resolved thread limits");
		check(measure(true).filter(f -> !f.passed).length==0,"the through-tapped receiver permits reverse screw entry");
		screw().shankLength=27.2;
		check(passes("thread-engagement"),"engagement at the source minimum is accepted");
		screw().shankLength=27.19;
		check(!passes("thread-engagement"),"shorter full-thread overlap fails the actual source minimum");
		reset();screw().shankLength=50;
		check(passes("thread-engagement") && !passes("thread-shank"),"adequate threaded overlap cannot hide a smooth shank inside tapped steel");
		reset();screw().shankLength=38.9;
		check(passes("thread-tip-gap"),"tip clearance at the source policy minimum is accepted");
		screw().shankLength=38.91;
		check(!passes("thread-tip-gap"),"a screw short of the drill floor can still violate usable-thread tip clearance");
		reset();screw().bearingFrame.x+=0.1;
		check(!passes("thread-axis"),"an offset screw cannot qualify by interval overlap alone");
		reset();screw().bearingFrame=F.compose(screw().bearingFrame,F.turnY(0.001));
		check(!passes("thread-axis"),"a tilted screw does not share the source hole axis");
		reset();hole().counterbore={diameter:8,depth:5};
		check(!passes("thread-engagement"),"counterbore depth does not contribute full thread engagement");
		hole().counterbore={diameter:8,depth:18};
		check(!passes("thread-engagement"),"a counterbore consuming the usable thread cannot create a reversed phantom interval");
		reset();hole().relief={diameter:6,start:(cast hole().threadDepth:Float)};screw().shankLength=40;
		check(passes("thread-tip-gap"),"a declared enlarged blind relief admits the tip beyond the full-thread endpoint");
		screw().shankLength=40.31;
		check(!passes("thread-tip-gap"),"a relief does not remove the source tip-gap requirement at the drill floor");
		reset();screw().bearingFrame=F.compose(screw().bearingFrame,F.turnY(Math.PI));
		check(!passes("thread-entry"),"a blind source hole cannot be approached through its intact back wall");
		reset();materia.casting.FoundationJointAnalysis.hole(facts,jack.receiver).threadDepth=8;
		check(!passes("thread-entry",true),"reverse entry cannot pass an untapped tail of a partially tapped through bore");
		reset();var common=F.compose(F.translation(120,-80,200),F.turnY(0.7));
		for (insert in facts.inserts) insert.pose=F.compose(common,insert.pose);
		for (fastener in facts.screws) fastener.bearingFrame=F.compose(common,fastener.bearingFrame);
		check(measure().filter(f -> !f.passed).length==0 && measure(true).filter(f -> !f.passed).length==0,
			"screw engagement and reverse receiver entry are invariant under common rigid placement");
	}
	static function finiteContact():Void {
		for (x in [70.0,87.0,89.0,93.0,0.0]) {
			var fixture=model(joint(),42,x),expected=x==70 || x==87;
			var diagnostics=AssemblyDesignChecks.check(fixture.assembly);
			check(hasCode(diagnostics,"design.foundation-joint.jack-finite-contact")!=expected,
				"finite actual bearing coverage at jack x="+x);
			check(!hasCode(diagnostics,"design.foundation-joint.jack-contact"),
				"infinite bearing-plane coincidence alone cannot detect an edge or the bearing hole");
			var context=machinekit.tooling.FoundationAssemblyContext.snapshot(fixture.assembly,"lower/casting",[{id:"seat",member:"lower/seat"}]);
			var input=machinekit.tooling.FoundationToolingAdapter.snapshot(fixture.lower,"joint-bearing-test",1,context);
			var reports=machinekit.tooling.FoundationJointCad.review(input);
			check(reports.length==1 && (reports[0].status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass)==expected,
				"fingerprinted source reconstruction agrees with actual assembly bearing contact");
		}
		var fixture=model(joint());
		var context=machinekit.tooling.FoundationAssemblyContext.snapshot(fixture.assembly,"lower/casting",[{id:"seat",member:"lower/seat"}]);
		var input=machinekit.tooling.FoundationToolingAdapter.snapshot(fixture.lower,"joint-hardware-check",1,context);
		var facts=input.interfaceContext;
		if (facts==null) throw "Expected joint source context";
		facts.jointSources.screws[0].threadedLength-=1;
		rejects(() -> machinekit.tooling.FoundationJointCad.review(input),"Joint hardware differs from source standard component");
	}

	static function toolingSnapshot():Void {
		var fixture=model(joint()),context=machinekit.tooling.FoundationAssemblyContext.snapshot(fixture.assembly,"lower/casting",[{id:"seat",member:"lower/seat"}]);
		var input=machinekit.tooling.FoundationToolingAdapter.snapshot(fixture.lower,"joint-base",1,context);
		check(context.foundationJoints.length==1 && context.jointSources.bodies.length==2 && context.jointSources.inserts.length==3
			&& context.jointSources.screws.length==2,"tooling copies the scoped joint and de-duplicates actual source components");
		var encoded=materia.tooling.ToolingCodec.encodeInput(input),copy=materia.tooling.ToolingCodec.decodeInput(encoded);
		check(materia.tooling.ToolingCodec.encodeInput(copy)==encoded,"source joint facts round trip with the foundation fingerprint");
		check(materia.tooling.casting.FoundationJointQualification.missingSource(copy)==null,"declared source joint resolves the mounting-interface definition request");
		var contextCopy=copy.interfaceContext;
		if (contextCopy==null) throw "Expected source context";
		var facts=contextCopy.jointSources;
		threadLimits(facts,contextCopy.foundationJoints[0].joint);
		var checks=materia.tooling.casting.FoundationJointQualification.analyze(copy,[]);
		check(checks.filter(c -> c.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail).length==0,
			"portable checks recompute the same nominal joint geometry as native design checks");
		check(checks.filter(c -> c.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending).length==1
			&& checks.filter(c -> c.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported).length==1,
			"source declarations never qualify cure evidence, finite contacts or restraint");
		var unknown:Array<materia.tooling.casting.CastingMoldPlan.FoundationJointProcess> = [{jointId:"missing",materialReference:null,cureEvidence:[]}];
		rejects(() -> materia.tooling.casting.FoundationJointQualification.analyze(copy,unknown),"Unknown source foundation joint process");
		facts.inserts.filter(i -> i.member=="lower/seat")[0].spec.holes[0].threadDepth=16;
		rejects(() -> materia.tooling.ToolingCodec.inputSnapshot(copy),"Joint context differs from source insert");
		copy=materia.tooling.ToolingCodec.decodeInput(encoded);contextCopy=copy.interfaceContext;
		if (contextCopy==null) throw "Expected source context";
		contextCopy.jointSources.screws.filter(s -> s.member=="jack")[0].shankLength+=1;
		check(materia.tooling.ToolingCodec.inputFingerprint(copy)!=materia.tooling.ToolingCodec.inputFingerprint(input),"source fastener dimensions enter the foundation fingerprint");
		check(materia.tooling.casting.FoundationJointQualification.analyze(copy,[]).filter(c -> c.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail).length==1,
			"changed screw geometry is rechecked rather than trusting an encoded native verdict");
		copy=materia.tooling.ToolingCodec.decodeInput(encoded);contextCopy=copy.interfaceContext;
		if (contextCopy==null) throw "Expected source context";
		contextCopy.foundationJoints=[];contextCopy.jointSources=materia.casting.FoundationJointSources.FoundationJointSourceCodec.empty();
		check(materia.tooling.casting.FoundationJointQualification.missingSource(materia.tooling.ToolingCodec.inputSnapshot(copy))!=null,
			"removing the source declaration restores the mounting-interface request");
		var document=SinglePocketSetup.document();document.input=input;document.setup.features=[];
		document.setup.fillHeightMm=40;document.setup.sourceToMold=F.fromRotationMatrix(0,0,40,[1,0,0,0,-1,0,0,0,-1]);
		document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(input);
		var options:materia.tooling.casting.CastingMoldPlan.CastingMoldOptions={replicationJigs:[],foundationJointProcesses:[],

			precision:[{insertId:"seat",masterReference:null,jigReference:null,masterFlatnessMm:null,releaseSpec:null,holeProtectionReference:null,evidence:[]}],
			compaction:{method:materia.tooling.casting.CastingMoldPlan.CompactionMethod.Undeclared,bedInterfaceReference:null,
				frequencyHz:null,amplitudeMm:null,durationSeconds:null,evidence:[]},

			serviceOrientation:F.identity(),overlapRule:materia.tooling.casting.CastingMoldPlan.OverlapRule.ReservedThenPriority,liftingEvidence:[]};
		var plan=materia.tooling.casting.CastingMoldPlanner.plan(document,options);
		check(plan.changeRequests.filter(c -> c.kind==materia.tooling.casting.CastingMoldPlan.DesignChangeKind.FoundationJointDefinition).length==0,
			"a declared source joint clears its design-change request in the mold planner");
		check(!plan.fabrication.ready,"mold planning keeps source joint physical qualification open");
		exportJoint(plan);
		plan.validation=plan.validation.filter(c -> c.rule.indexOf("tooling.foundation-joint.")!=0);
		rejects(() -> materia.tooling.casting.CastingMoldPlanner.validate(plan),"Foundation joint validation differs");
		document.input=copy;document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(copy);
		plan=materia.tooling.casting.CastingMoldPlanner.plan(document,options);
		check(plan.changeRequests.filter(c -> c.kind==materia.tooling.casting.CastingMoldPlan.DesignChangeKind.FoundationJointDefinition).length==1,
			"removing the declaration restores the real source request in the mold planner");
		var old:Dynamic=haxe.Json.parse(materia.tooling.ToolingCodec.canonical(options));Reflect.setField(old,"groutedJoints",new Array<Dynamic>());
		rejects(() -> materia.tooling.casting.CastingMoldOptionsCodec.read(old),"Invalid mold process record");
	}
	public static function main():Void run();

	public static function run():Void {
		var source=fixture(joint()),saved=source.encode(),copy=MachineAssembly.decode(saved);
		check(copy.encode()==saved,"foundation joint source declaration survives typed assembly encoding");
		var declarations=copy.designRequirements();
		switch declarations[0] {case FoundationConnection(_,j):j.gapMm=900;default:throw "Expected joint";}
		check(copy.encode()==saved,"returned source joint declarations are defensive copies");
		var result=AssemblyDesignChecks.check(copy);
		var errors=[for (item in result.items) if (item.severity==machinekit.assembly.Diagnostics.DiagnosticSeverity.Error) item.message];
		check(!result.hasErrors(),"valid synthetic source references and nominal gap: "+errors.join("; "));
		check(hasCode(result,"design.foundation-joint.qualification"),"nominal joint checks cannot qualify physical grout, restraint or cure");
		result=AssemblyDesignChecks.check(fixture(joint(),43));
		check(hasCode(result,"design.foundation-joint.gap"),"a declaration cannot hide the wrong source assembly gap");
		check(hasCode(result,"design.foundation-joint.jack-contact"),"a moved foundation leaves the fixed jack tip off its bearing plane");
		var bad=joint();bad.seats[0].lower.feature="missing";
		check(invalid(AssemblyDesignChecks.check(fixture(bad)),"Unknown foundation joint precision interface"),
			"source joint seats must resolve to actual precision interfaces");
		bad=joint();bad.anchors[0].target.feature="seat";
		check(invalid(AssemblyDesignChecks.check(fixture(bad)),"Unknown hole"),
			"a named precision plane cannot substitute for a source threaded hole");
		bad=joint();bad.jacks[0].screw="block";
		check(invalid(AssemblyDesignChecks.check(fixture(bad)),"screw is not an actual fastener"),
			"jack hardware must be an actual fastener component");
		bad=joint();bad.anchors[0].through=[{member:"block",feature:"fixing"}];
		check(invalid(AssemblyDesignChecks.check(fixture(bad)),"source through-clearance hole"),
			"an unrelated tapped bore cannot substitute for the opposite foundation's clearance hole");
		var outer=new MachineAssembly();outer.include("station",source,F.compose(F.translation(100,-200,300),F.turnY(0.4)));
		check(!AssemblyDesignChecks.check(MachineAssembly.decode(outer.encode())).hasErrors(),"joint checks preserve ownership and geometry under nested common rigid transforms");
		for (r in outer.designRequirements()) switch r {
			case FoundationConnection(id,j):
				check(id=="station/joint-instance" && j.id=="column-base","occurrence and design IDs retain separate scopes");
				check(j.lowerBody=="station/lower/casting" && j.jacks[0].receiver.member=="station/block"
					&& j.anchors[0].through[0].member=="station/upper/seat","nesting remaps all source joint references");
			default:throw "Expected joint";
		}
		finiteContact();
		toolingSnapshot();
		Sys.println('FoundationJointDesignTests: $assertions assertions passed');
	}
}
