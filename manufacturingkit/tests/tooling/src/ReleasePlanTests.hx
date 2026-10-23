import materia.tooling.ReleasePlan;
import materia.tooling.ReleaseSystem;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;

class ReleasePlanTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {assertions++;if (!value) throw message;}
	static function face(id:String,part:String,kind:ReleaseSubstrate,wet:Float,seal:Bool):ReleaseFace
		return {id:id,partId:part,sourceIds:["casting"],faceIndex:0,areaMm2:100,wettedAreaMm2:wet,
			bounds:{x:0,y:0,z:0,width:10,depth:10,height:0},substrate:kind,needsSealing:seal,adjacentCoatedFaceIds:[]};
	static function fixture():ReleasePlan {
		var source=ReleaseSystems.unresolved(2,5,1,EPSCoveringKind.Tape);
		source.epsSafety=EPSReleaseSafety.DeclaredSafe;
		var bond=face("insert-bond","insert",BondingSteel,100,false);
		bond.adjacentCoatedFaceIds=["board-wet"];
		return ReleasePlans.build("casting",source,0.1,[
			face("board-wet","board",Board,100,true),face("board-dry","board",Board,0,true),
			face("fixture-wet","fixture",ToolingMetal,100,false),face("eps-wet","eps",EPSCovering,100,false),
			bond,face("anchor-bond","anchor",BondingSteel,100,false)
		]);
	}
	static function status(plan:ReleasePlan,suffix:String):ToolingCheckStatus
		return ReleasePlans.validate(plan).filter(row -> row.rule=="fabrication.release."+suffix)[0].status;
	static function first(plan:ReleasePlan,part:String,action:ReleaseAction):Int {
		for (index in 0...plan.steps.length) if (plan.steps[index].partId==part && plan.steps[index].action==action) return index;
		return -1;
	}
	public static function run():Void {
		var plan=fixture();
		check(plan.coatedFaceIds.length==3 && plan.coatedFaceIds.indexOf("board-dry")<0,"Only wetted tooling receives release");
		check(plan.mustNotCoatFaceIds.length==2,"Both insert and anchor bonding faces are protected");
		for (rule in ["coverage","bonding","assembly","eps-safety"]) check(status(plan,rule)==Pass,"Physical release rule passes: "+rule);
		check(first(plan,"board",CureSealer)<first(plan,"board",ApplyAgent),"Sealing and cure precede wax");
		check(first(plan,"board",CureAgent)<first(plan,"board",ApplyPVA),"PVA follows release-agent cure");
		check(first(plan,"eps",CoverEPS)<first(plan,"eps",ApplyAgent),"EPS is covered before release-agent exposure");
		var dryBond=face("eps-dry-bond","eps",EPSCovering,0,false);
		var supported=ReleasePlans.build("casting",plan.system,0.1,plan.faces.concat([dryBond]));
		check(supported.steps.filter(step -> step.action==CoverEPS && step.faceIds.indexOf(dryBond.id)>=0).length==0,
			"Dry support/foot bond faces stay free of covering and release");
		check(first(plan,"insert",MaskBondingFaces)<first(plan,"board",ApplyAgent),"Adjacent steel is masked before coating");
		check(plan.steps.filter(step -> step.action==Seal && step.faceIds.indexOf("board-dry")>=0).length==2,
			"Dry exposed board faces in the sealing record are sealed too");
		check(plan.prepareBeforePartIds.join("|")=="anchor|insert","All preparation precedes fitting steel or anchored assemblies");
		plan=fixture();plan.system.epsSafety=EPSReleaseSafety.Unsafe;
		var unsafe=ReleasePlans.validate(plan).filter(row -> row.rule=="fabrication.release.eps-safety")[0];
		check(unsafe.status==Fail && unsafe.affectedIds.join("|")=="eps-wet","Unsafe agent fails against the exact EPS-contact face IDs");
		plan.faces.filter(surface -> surface.id=="eps-wet")[0].wettedAreaMm2=1e-12;
		check(status(plan,"eps-safety")==Fail,"A positive EPS-contact area is never hidden by a size threshold");
		plan=fixture();plan.coveringThicknessMm=0;
		check(status(plan,"eps-safety")==Fail,"Declared covering requires actual dimensional thickness");
		plan=fixture();plan.system.epsSafety=EPSReleaseSafety.Unresolved("Supply EPS compatibility declaration");
		check(status(plan,"eps-safety")==Pending,"Unknown EPS safety remains an explicit pending input");
		plan=fixture();plan.coatedFaceIds.pop();
		check(status(plan,"coverage")==Fail,"An omitted wet face fails coverage");
		plan=fixture();plan.coatedFaceIds.push(plan.coatedFaceIds[0]);
		check(status(plan,"coverage")==Fail,"A duplicate assignment fails exact-once coverage");
		plan=fixture();plan.steps.filter(step -> step.action==ApplyAgent)[0].faceIds=[];
		check(status(plan,"coverage")==Fail,"An omitted physical coat fails even if the assignment summary is unchanged");
		plan=fixture();plan.coatedFaceIds.push("anchor-bond");
		check(status(plan,"bonding")==Fail,"Bonding steel cannot be assigned a release system");
		plan=fixture();plan.steps.filter(step -> step.action==ApplyAgent)[0].faceIds.push("insert-bond");
		check(status(plan,"bonding")==Fail,"An actual coating action on protected steel fails even if the summary list is unchanged");
		plan=fixture();plan.steps=plan.steps.filter(step -> step.action!=MaskBondingFaces);
		check(status(plan,"assembly")==Fail,"Adjacent bonding masks are required before coating");
		plan=fixture();plan.prepareBeforePartIds=[];
		check(status(plan,"assembly")==Fail,"The insert-fitting prerequisite cannot be omitted");
		var dry=ReleasePlans.build("casting",plan.system,0,[face("steel","insert",BondingSteel,0,false)]);
		check(ReleasePlans.validate(dry).filter(row -> row.status==Pending).length==0,
			"Unused coating products do not produce repeated missing-input findings");
		Sys.println('ReleasePlanTests: $assertions assertions passed');
	}
	public static function main():Void run();
}
