package materia.tooling;

import materia.casting.CastingInput;
import materia.casting.CastingInput.CastingInputs;
import materia.tooling.ReleaseSystem;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.export.ToolingLoads;

enum abstract ReleaseSubstrate(String) to String {
	var Board = "board";
	var EPSCovering = "eps-covering";
	var ToolingMetal = "tooling-metal";
	var SealedCompound = "sealed-compound";
	var BondingSteel = "bonding-steel";
	var BondingCompound = "bonding-compound";
}

/** A native geometry query supplies the contact area, never a hand-entered list.
 * Sealing treats the complete exposed board face; release treats its wetted
 * portion only. Dry faces may need sealing without receiving mold release.
 */
typedef ReleaseFace = {
	var id:String;
	var partId:String;
	var sourceIds:Array<String>;
	var faceIndex:Int;
	var areaMm2:Float;
	var wettedAreaMm2:Float;
	var bounds:materia.casting.Box3;
	var substrate:ReleaseSubstrate;
	var needsSealing:Bool;
	var adjacentCoatedFaceIds:Array<String>;
}

enum abstract ReleaseAction(String) to String {
	var MaskBondingFaces = "mask-bonding-faces";
	var Seal = "seal";
	var DrySealer = "dry-sealer";
	var CureSealer = "cure-sealer";
	var CoverEPS = "cover-eps";
	var ApplyAgent = "apply-agent";
	var DryAgent = "dry-agent";
	var BuffAgent = "buff-agent";
	var WaitToRecoat = "wait-to-recoat";
	var CureAgent = "cure-agent";
	var ApplyPVA = "apply-pva";
	var DryPVA = "dry-pva";
	var ReadyForAssembly = "ready-for-assembly";
}

typedef ReleaseStep = {
	var id:String;
	var partId:String;
	var action:ReleaseAction;
	/** Sealing treats whole exposed faces; agent/PVA/covering treat their wetted portions. */
	var faceIds:Array<String>;
	var coat:Int;
	/** Null means an untimed action, never an unknown duration. */
	var seconds:Null<CastingInput>;
}

typedef ReleasePlan = {
	var sourceId:String;
	var system:ReleaseSystem;
	var coveringThicknessMm:Float;
	var faces:Array<ReleaseFace>;
	var coatedFaceIds:Array<String>;
	var mustNotCoatFaceIds:Array<String>;
	var steps:Array<ReleaseStep>;
	/** All preparation precedes fitting any of these source steel parts. */
	var prepareBeforePartIds:Array<String>;
}

class ReleasePlans {
	public static function isBonding(substrate:ReleaseSubstrate):Bool
		return substrate==BondingSteel || substrate==BondingCompound;

	public static function build(sourceId:String,system:ReleaseSystem,coveringThicknessMm:Float,
			faces:Array<ReleaseFace>):ReleasePlan {
		var sorted=faces.copy();sorted.sort((a,b) -> Reflect.compare(a.id,b.id));
		var coated=[for (face in sorted) if (face.wettedAreaMm2>0 && !isBonding(face.substrate)) face.id];
		var protected=[for (face in sorted) if (isBonding(face.substrate)) face.id];
		var parts:Array<String>=[],steel:Array<String>=[],steps:Array<ReleaseStep>=[];
		for (face in sorted) {
			ToolingIds.validate(face.id);ToolingIds.validate(face.partId);
			if (parts.indexOf(face.partId)<0) parts.push(face.partId);
			if (isBonding(face.substrate) && steel.indexOf(face.partId)<0) steel.push(face.partId);
		}
		parts.sort(Reflect.compare);steel.sort(Reflect.compare);
		function add(partId:String,kind:ReleaseAction,ids:Array<String>,coat:Int,seconds:Null<CastingInput>):Void {
			steps.push({id:ToolingIds.derive("release-",partId,"-step-"+steps.length),partId:partId,
				action:kind,faceIds:ids.copy(),coat:coat,seconds:seconds});
		}
		// Masking is prepared before any coating. Steel stays out of the mold
		// throughout preparation, including anchored insert assemblies.
		for (partId in steel) {
			var mask=[for (face in sorted) if (face.partId==partId && face.adjacentCoatedFaceIds.length>0) face.id];
			if (mask.length>0) add(partId,MaskBondingFaces,mask,0,null);
		}
		for (partId in parts) {
			var owned=sorted.filter(face -> face.partId==partId);
			if (isBonding(owned[0].substrate)) continue;
			var seal=[for (face in owned) if (face.needsSealing) face.id];
			var wet=[for (face in owned) if (coated.indexOf(face.id)>=0) face.id];
			var eps=[for (face in owned) if (face.substrate==EPSCovering && face.wettedAreaMm2>0) face.id];
			if (seal.length>0) {
				for (coat in 1...system.sealer.coats+1) {
					add(partId,Seal,seal,coat,null);
					add(partId,DrySealer,seal,coat,system.sealer.drySeconds);
				}
				add(partId,CureSealer,seal,0,system.sealer.cureSeconds);
			}
			if (eps.length>0) add(partId,CoverEPS,eps,0,null);
			if (wet.length>0) {
				for (coat in 1...system.agent.coats+1) {
					add(partId,ApplyAgent,wet,coat,null);
					add(partId,DryAgent,wet,coat,system.agent.drySeconds);
					add(partId,BuffAgent,wet,coat,system.agent.buffSeconds);
					if (coat<system.agent.coats) add(partId,WaitToRecoat,wet,coat,system.agent.recoatSeconds);
				}
				add(partId,CureAgent,wet,0,system.agent.cureSeconds);
				if (system.pva!=null) {
					var pva:ReleaseFilm=cast system.pva;
					for (coat in 1...pva.coats+1) {
						add(partId,ApplyPVA,wet,coat,null);
						add(partId,DryPVA,wet,coat,pva.drySeconds);
					}
				}
			}
			add(partId,ReadyForAssembly,wet,0,null);
		}
		return {sourceId:sourceId,system:ReleaseSystems.copy(system),coveringThicknessMm:coveringThicknessMm,
			faces:sorted,coatedFaceIds:coated,mustNotCoatFaceIds:protected,steps:steps,prepareBeforePartIds:steel};
	}

	public static function validate(plan:ReleasePlan):Array<ToolingValidation> {
		var ids=[plan.sourceId],expectedCoated:Array<String>=[],expectedProtected:Array<String>=[],eps:Array<String>=[];
		var all:Array<String>=[],bad:Array<String>=[];
		for (face in plan.faces) {
			if (all.indexOf(face.id)>=0 || !Math.isFinite(face.areaMm2) || face.areaMm2<=0
				|| !Math.isFinite(face.wettedAreaMm2) || face.wettedAreaMm2<0 || face.wettedAreaMm2>face.areaMm2+1e-6)
				bad.push(face.id);
			all.push(face.id);
			if (isBonding(face.substrate)) expectedProtected.push(face.id);
			else if (face.wettedAreaMm2>0) expectedCoated.push(face.id);
			if (face.substrate==EPSCovering && face.wettedAreaMm2>0) eps.push(face.id);
		}
		function agrees(actual:Array<String>,expected:Array<String>):Bool {
			if (actual.length!=expected.length) return false;
			for (id in expected) if (actual.filter(value -> value==id).length!=1) return false;
			return true;
		}
		function coatsMatch(action:ReleaseAction,count:Int):Bool {
			var counts=new Map<String,Int>();
			for (step in plan.steps) if (step.action==action) {
				if (step.coat<1 || step.coat>count) return false;
				for (id in step.faceIds) {
					var owned=plan.faces.filter(face -> face.id==id);
					if (expectedCoated.indexOf(id)<0 || owned.length!=1 || owned[0].partId!=step.partId) return false;
					var key=id+"/"+step.coat,value=counts.get(key);
					if (value==null) value=0;
					counts.set(key,value+1);
				}
			}
			for (id in expectedCoated) for (coat in 1...count+1)
				if (counts.get(id+"/"+coat)!=1) return false;
			return true;
		}
		var pvaCoats=plan.system.pva==null ? 0 : plan.system.pva.coats;
		var coverage=bad.length==0 && agrees(plan.coatedFaceIds,expectedCoated)
			&& coatsMatch(ApplyAgent,plan.system.agent.coats) && coatsMatch(ApplyPVA,pvaCoats);
		var badCoating=[for (step in plan.steps)
			if (step.action==Seal || step.action==ApplyAgent || step.action==ApplyPVA)
				for (id in step.faceIds) if (expectedProtected.indexOf(id)>=0) id];
		var bonding=agrees(plan.mustNotCoatFaceIds,expectedProtected) && badCoating.length==0
			&& plan.coatedFaceIds.filter(id -> expectedProtected.indexOf(id)>=0).length==0;
		var steel:Array<String>=[];
		for (face in plan.faces) if (isBonding(face.substrate) && steel.indexOf(face.partId)<0) steel.push(face.partId);
		var firstCoat=plan.steps.length;
		for (index in 0...plan.steps.length) if (plan.steps[index].action==ApplyAgent) {firstCoat=index;break;}
		var assembly=agrees(plan.prepareBeforePartIds,steel);
		for (face in plan.faces) if (isBonding(face.substrate) && face.adjacentCoatedFaceIds.length>0) {
			var masked=false;
			for (index in 0...firstCoat) if (plan.steps[index].action==MaskBondingFaces
				&& plan.steps[index].faceIds.indexOf(face.id)>=0) masked=true;
			if (!masked) assembly=false;
		}
		var result=[
			ToolingLoads.check("fabrication.release.coverage",bad.length>0 ? bad : ids,coverage ? Pass : Fail,
				"Every geometrically wetted tooling face must receive exactly one release-system assignment.",[]),
			ToolingLoads.check("fabrication.release.bonding",ids,bonding ? Pass : Fail,
				"Insert and anchor steel is never coated; adjacent bonding faces are masked before tooling preparation.",[]),
			ToolingLoads.check("fabrication.release.assembly",ids,assembly ? Pass : Fail,
				"Complete all release preparation before fitting any source insert or anchor assembly; mask adjacent bonding faces before coating.",[])
		];
		var safety:ToolingCheckStatus=Pass,message="No release-agent exposure to EPS.";
		if (eps.length>0) {
			if (!Math.isFinite(plan.coveringThicknessMm) || plan.coveringThicknessMm<=0) {
				safety=Fail;message="EPS covering needs a positive declared thickness already included in the core geometry.";
			} else switch plan.system.epsSafety {
				case DeclaredSafe:message="Declared EPS-safe release system; the physical chemical coupon remains independently required.";
				case Unsafe:safety=Fail;message="EPS-unsafe release agent would be applied to the listed EPS-contact faces.";
				case Unresolved(reason):safety=Pending;message=reason+"; blocks release on the listed EPS-contact faces.";
			}
		}
		result.push(ToolingLoads.check("fabrication.release.eps-safety",eps.length>0 ? eps : ids,safety,message,[]));
		for (input in ReleaseSystems.missing(plan.system,plan.faces.filter(face -> face.needsSealing).length>0,
			eps.length>0,expectedCoated.length>0)) {
			result.push(ToolingLoads.check("fabrication.release.input."+haxe.crypto.Sha256.encode(input.reason).substr(0,16),
				ids,Pending,input.reason+"; blocks "+input.blocks.join(", ")+".",[]));
		}
		return result;
	}

	public static function record(plan:ReleasePlan):Dynamic return {
		sourceId:plan.sourceId,system:ReleaseSystems.record(plan.system),coveringThicknessMm:plan.coveringThicknessMm,
		faces:plan.faces,coatedFaceIds:plan.coatedFaceIds,mustNotCoatFaceIds:plan.mustNotCoatFaceIds,
		prepareBeforePartIds:plan.prepareBeforePartIds,
		steps:[for (step in plan.steps) {id:step.id,partId:step.partId,action:(step.action:String),
			faceIds:step.faceIds,coat:step.coat,seconds:step.seconds==null ? null : CastingInputs.record(cast step.seconds)}]
	};
}
