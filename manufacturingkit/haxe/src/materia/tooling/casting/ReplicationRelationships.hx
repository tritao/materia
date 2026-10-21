package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.casting.InterfaceRelationship;
import materia.casting.InterfaceRelationship.InterfaceRelationships as R;
import materia.casting.CastingInterfaceContext;
import materia.casting.InterfaceRequirement;
import materia.tooling.casting.ReplicationJig;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;

private enum ReferenceKind {PlaneReference;LineReference;}
private typedef Point3 = {var x:Float;var y:Float;var z:Float;}
private typedef MasterReference = {var point:Point3;var direction:Point3;var kind:ReferenceKind;var radius:Float;}

/** Nominal jig relationships. Angular limits retain the source reference span;
 * coplanarity/distance use conservative bounds over the recorded finite stock.
 * These are design prerequisites, never flatness traces or measured qualification.
 */
class ReplicationRelationships {
	static inline var DIRECTION_ROUNDOFF = 1e-10;
	public static function ruleId(instanceId:String,jigId:String):String
		return "tooling.replication.relationship-geometry."+haxe.crypto.Sha256.encode(instanceId+"/"+jigId);
	public static function check(jig:ReplicationJig,instanceId:String,authored:InterfaceRelationship,
		context:CastingInterfaceContext):ToolingValidation {
		var relationship=R.snapshot([authored])[0];
		var rule=ruleId(instanceId,jig.id);
		var ids=[jig.id],references:Array<MasterReference> = [];
		for (member in relationship.members) {
			var key=R.memberKey(member),masters=jig.masters.filter(master -> master.members.filter(m -> R.memberKey(m)==key).length==1);
			var sources=context.sources.filter(source -> CastingInterfaceContexts.sourceKey(source)==key);
			if (masters.length!=1 || sources.length!=1)
				return result(rule,ids,ToolingCheckStatus.Fail,"Relationship requires one master and source reference for "+key);
			var master=masters[0],frame=F.compose(F.compose(jig.pose,master.pose),master.reference);
			if (ids.indexOf(master.id)<0) ids.push(master.id);
			var kind=switch sources[0] {
				case Axis(_,_,_):LineReference;
				case Precision(owner,requirement,_):
					for (binding in context.inserts) if (binding.member==owner && ids.indexOf(binding.id)<0) ids.push(binding.id);
					switch requirement.geometry {case Plane(_):PlaneReference;case Line(_):LineReference;}
			};
			var direction=switch sources[0] {
				case Precision(_,requirement,_):switch requirement.geometry {
					case Line(_):F.transformVector(frame,1,0,0);
					case Plane(_):F.transformVector(frame,0,0,1);
				};
				case Axis(_,_,_):F.transformVector(frame,0,0,1);
			};
			references.push({point:{x:frame.x,y:frame.y,z:frame.z},direction:unit(direction),kind:kind,
				radius:ReplicationPlacement.stockRadius(master.body,master.reference)});
		}
		var limit=R.maxDeviation(relationship.tolerance),worst=0.0;
		switch relationship.kind {
			case Straightness:
				return result(rule,ids,ToolingCheckStatus.Unsupported,"Straightness needs a finite profile or measured trace; an ideal reference line is not evidence.");
			case InPlaneParallelism,InPlaneSquareness:
				return result(rule,ids,ToolingCheckStatus.Unsupported,"In-plane geometry requires an explicit common reference-plane contract; it is not inferred from arbitrary line/axis frames.");
			case Coplanar:
				for (reference in references) if (reference.kind!=PlaneReference)
					return result(rule,ids,ToolingCheckStatus.Unsupported,"Coplanarity requires plane interfaces, not a substituted axis or edge.");
				for (i in 0...references.length) for (j in i+1...references.length) {
					var a=references[i],b=references[j],delta=subtract(b.point,a.point),sine=length(cross(a.direction,b.direction));
					var ab=Math.abs(dot(delta,a.direction))+b.radius*sine;
					var ba=Math.abs(dot(delta,b.direction))+a.radius*sine;
					worst=Math.max(worst,Math.max(ab,ba));
				}
			case Parallel,Perpendicular:
				var span=switch relationship.tolerance {case PerLength(_,value):value;case Linear(_):throw "Angular relationship requires a declared span";};
				for (i in 0...references.length) for (j in i+1...references.length) {
					var sine=length(cross(references[i].direction,references[j].direction));
					var cosine=Math.abs(dot(references[i].direction,references[j].direction));
					var mixed=references[i].kind!=references[j].kind;
					// Plane-line parallelism means a perpendicular normal and line direction.
					var parallelDirections=relationship.kind==Parallel ? !mixed : mixed;
					var rise=parallelDirections ? (cosine==0 ? Math.POSITIVE_INFINITY : span*sine/cosine)
						: (sine==0 ? Math.POSITIVE_INFINITY : span*cosine/sine);
					worst=Math.max(worst,rise);
				}
			case Distance(separation):
				var a=references[0],b=references[1],delta=subtract(b.point,a.point);
				if (a.kind==PlaneReference && b.kind==PlaneReference) {
					var sine=length(cross(a.direction,b.direction));
					worst=Math.max(Math.abs(Math.abs(dot(delta,a.direction))-separation)+b.radius*sine,
						Math.abs(Math.abs(dot(delta,b.direction))-separation)+a.radius*sine);
				} else if (a.kind==LineReference && b.kind==LineReference) {
					if (length(cross(a.direction,b.direction))>DIRECTION_ROUNDOFF)
						return result(rule,ids,ToolingCheckStatus.Unsupported,"Nonparallel axis/edge distance needs an explicit projection or gage-plane definition.");
					worst=Math.abs(length(cross(delta,a.direction))-separation);
				} else return result(rule,ids,ToolingCheckStatus.Unsupported,"Mixed plane/axis distance needs an explicit projection or contact definition.");
		}
		if (Math.isNaN(worst)) return result(rule,ids,ToolingCheckStatus.Unsupported,"Nominal relationship bound is not numerically representable.");
		return result(rule,ids,worst<=limit ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail,
			instanceId+": nominal pairwise relationship bound "+worst+" mm; source limit "+limit
			+" mm. Finite contact coverage, uncertainty and measured alignment remain separate.");
	}

	static function dot(a:Point3,b:Point3):Float return a.x*b.x+a.y*b.y+a.z*b.z;
	static function cross(a:Point3,b:Point3):Point3 return {x:a.y*b.z-a.z*b.y,y:a.z*b.x-a.x*b.z,z:a.x*b.y-a.y*b.x};
	static function subtract(a:Point3,b:Point3):Point3 return {x:a.x-b.x,y:a.y-b.y,z:a.z-b.z};
	static function length(p:Point3):Float return Math.sqrt(dot(p,p));
	static function unit(p:Point3):Point3 {var n=length(p);return {x:p.x/n,y:p.y/n,z:p.z/n};}
	static function result(rule:String,ids:Array<String>,status:ToolingCheckStatus,message:String):ToolingValidation return {
		rule:rule,affectedIds:ids,status:status,severity:status==ToolingCheckStatus.Pass ? ToolingSeverity.Info : ToolingSeverity.Error,
		message:message,evidence:[]};
}
