package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingSolid;
import materia.casting.CastingInterfaceContext;
import materia.casting.InterfaceRelationship;
import materia.casting.InterfaceRelationship.InterfaceRelationships as R;
import materia.casting.InterfaceRequirement;
import materia.casting.InterfaceRequirement.InterfaceRequirements;
import materia.tooling.casting.ReplicationJig;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;

private typedef V3 = {var x:Float;var y:Float;var z:Float;}

/** A conservative analytic bound on nominal master placement, not a contact,
 * flatness, metrology or physical relationship qualification. Angular error is
 * evaluated over the recorded plane/edge stock or the source's declared axis spans.
 * Every acceptance limit comes from the source; no fixture accuracy is assumed.
 */
class ReplicationPlacement {
	/** Rigid-transform roundoff, not a fixture manufacturing allowance. Source
	 * axes have exact nominal lines; their physical placement budget is unqualified.
	 */
	static inline var FRAME_ROUNDOFF_MM = 1e-8;
	public static function check(jig:ReplicationJig,master:ReplicationMaster,
		source:InterfaceContextSource,context:CastingInterfaceContext):ToolingValidation {
		var key=CastingInterfaceContexts.sourceKey(source);
		var rule="tooling.replication.jig."+jig.id+".master."+master.id+".placement."+haxe.crypto.Sha256.encode(key);
		var ids=[jig.id,master.id];
		switch source {
			case Precision(owner,requirement,_):
				for (binding in context.inserts) if (binding.member==owner && ids.indexOf(binding.id)<0) ids.push(binding.id);
				if (ids.indexOf(requirement.id)<0) ids.push(requirement.id);
			case Axis(_,_,_):
		}
		var frame=F.compose(F.compose(jig.pose,master.pose),master.reference);
		var radius=stockRadius(master.body,master.reference),worst=0.0;
		var failures:Array<String> = [];
		switch source {
			case Precision(_,requirement,pose):
				var target=F.compose(pose,InterfaceRequirements.geometryFrame(requirement.geometry));
				var axis=switch requirement.geometry {case Plane(_):2;case Line(_):0;};
				var normal=direction(target,axis),actual=direction(frame,axis);
				var delta={x:frame.x-target.x,y:frame.y-target.y,z:frame.z-target.z};
				var offset=switch requirement.geometry {
					case Plane(_):Math.abs(dot(delta,normal));
					case Line(_):length(cross(delta,normal));
				};
				var bound=offset+radius*length(cross(normal,actual));
				worst=bound;
				if (!Math.isFinite(bound)) return result(rule,ids,ToolingCheckStatus.Unsupported,"Master placement bound is not numerically representable.");
				if (bound>requirement.positionToleranceMm)
					failures.push("Interface "+requirement.id+" placement bound "+bound+" mm exceeds source position tolerance "+requirement.positionToleranceMm+" mm");
			case Axis(_,targetFrame,localDirection):
				var target=unit(F.transformVector(targetFrame,localDirection.x,localDirection.y,localDirection.z));
				var actual=direction(frame,2),delta={x:frame.x-targetFrame.x,y:frame.y-targetFrame.y,z:frame.z-targetFrame.z};
				var offset=length(cross(delta,target)),sine=length(cross(target,actual)),count=0;
				if (!Math.isFinite(offset)) return result(rule,ids,ToolingCheckStatus.Unsupported,"Nominal axis offset is not numerically representable.");
				if (offset>FRAME_ROUNDOFF_MM) failures.push("Master axis is not nominally coaxial with the source axis; lateral offset "+offset+" mm");
				for (instance in context.relationships) if (instance.relationship.members.filter(member -> R.memberKey(member)==key).length>0)
					switch instance.relationship.tolerance {
						case Linear(_): // A distance tolerance is not an angular allowance.
						case PerLength(tolerance,span):
							var cosine=Math.abs(dot(target,actual));
							// The gage span is on the reference axis, so lateral rise is L*tan(theta), not L*sin(theta).
							var bound=cosine==0 ? Math.POSITIVE_INFINITY : span*sine/cosine;
							worst=Math.max(worst,bound);count++;
							if (Math.isNaN(bound)) return result(rule,ids,ToolingCheckStatus.Unsupported,"Axis-master angular bound is not numerically representable.");
							if (bound>tolerance) failures.push("Axis angular deviation "+bound+" mm at "+span+" mm exceeds "+instance.id+" tolerance "+tolerance+" mm");
					}
				if (count==0 && failures.length==0) return result(rule,ids,ToolingCheckStatus.Unsupported,
					"Axis master has no source angular tolerance; no positional or angular manufacturing budget is invented.");
		}
		return failures.length>0 ? result(rule,ids,ToolingCheckStatus.Fail,key+": "+failures.join("; "))
			: result(rule,ids,ToolingCheckStatus.Pass,"Nominal master-to-source placement bound is "+worst
				+" mm within the source limits; contact coverage, measured geometry and relationship qualification are separate.");
	}

	/** Farthest distance from the contact-reference origin to the stock. Boxes
	 * use the exact corner extrema of a convex norm; cylinders use radial support
	 * and the farthest cap analytically. This is a bound, not sampled CAD coverage.
	 */
	public static function stockRadius(body:CastingSolid,reference:AssemblyFrame):Float return switch body {
		case Box(w,d,h):
			var x=w/2+Math.abs(reference.x),y=d/2+Math.abs(reference.y),z=Math.max(Math.abs(reference.z),Math.abs(h-reference.z));
			Math.sqrt(x*x+y*y+z*z);
		case Cylinder(d,h):
			var radial=d/2+Math.sqrt(reference.x*reference.x+reference.y*reference.y);
			var z=Math.max(Math.abs(reference.z),Math.abs(h-reference.z));Math.sqrt(radial*radial+z*z);
	};

	static function direction(frame:AssemblyFrame,axis:Int):V3
		return unit(F.transformVector(frame,axis==0 ? 1 : 0,0,axis==2 ? 1 : 0));
	static function unit(v:V3):V3 {
		var magnitude=length(v);return {x:v.x/magnitude,y:v.y/magnitude,z:v.z/magnitude};
	}
	static function dot(a:V3,b:V3):Float return a.x*b.x+a.y*b.y+a.z*b.z;
	static function cross(a:V3,b:V3):V3 return {x:a.y*b.z-a.z*b.y,y:a.z*b.x-a.x*b.z,z:a.x*b.y-a.y*b.x};
	static function length(v:V3):Float return Math.sqrt(dot(v,v));
	static function result(rule:String,ids:Array<String>,status:ToolingCheckStatus,message:String):ToolingValidation return {
		rule:rule,affectedIds:ids,status:status,severity:status==ToolingCheckStatus.Pass ? ToolingSeverity.Info : ToolingSeverity.Error,
		message:message,evidence:[]};
}
