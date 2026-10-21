package materia.casting;

import materia.casting.FoundationJoint;
import materia.casting.FoundationJointSources;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

typedef FoundationJointFinding = ThreadedFastenerFit.NominalThreadFinding;

/** Shared source-reference resolver and nominal geometry checks. Results are
 * recomputed from facts, never accepted as evidence encoded in a source file.
 */
class FoundationJointAnalysis {
	public static function validate(source:FoundationJointSources,joints:Array<FoundationJointOccurrence>):Void {
		var used:Map<String,Bool> = [];
		for (instance in joints) {
			var j=instance.joint;
			body(source,j.lowerBody);body(source,j.upperBody);used.set(j.lowerBody,true);used.set(j.upperBody,true);
			for (seat in j.seats) {
				owned(source,j.lowerBody,seat.lower);owned(source,j.upperBody,seat.upper);
				plane(source,seat.lower);plane(source,seat.upper);used.set(seat.lower.member,true);used.set(seat.upper.member,true);
			}
			for (jack in j.jacks) {
				screw(source,jack.screw);hole(source,jack.receiver);plane(source,jack.bearing);
				if (!belongs(source,j.lowerBody,jack.bearing) && !belongs(source,j.upperBody,jack.bearing))
					throw "Foundation joint jack bearing does not belong to either casting: "+jack.id;
				used.set(jack.screw,true);used.set(jack.receiver.member,true);used.set(jack.bearing.member,true);
			}
			for (anchor in j.anchors) {
				screw(source,anchor.screw);hole(source,anchor.target);used.set(anchor.screw,true);used.set(anchor.target.member,true);
				var lower=belongs(source,j.lowerBody,anchor.target),upper=belongs(source,j.upperBody,anchor.target);
				if (!lower && !upper) throw "Foundation joint anchor target does not belong to either casting: "+anchor.id;
				var linked=false,other=lower ? j.upperBody : j.lowerBody;
				for (reference in anchor.through) {
					var clearance=hole(source,reference);
					if (clearance.extent!=InsertHole.HoleExtent.Through || clearance.thread!=null)
						throw "Foundation joint anchor needs a source through-clearance hole: "+anchor.id;
					if (belongs(source,other,reference)) linked=true;
					used.set(reference.member,true);
				}
				if (!linked) throw "Foundation joint anchor does not cross the opposite casting's source insert: "+anchor.id;
			}
		}
		for (rows in [[for (b in source.bodies) b.member],[for (i in source.inserts) i.member],[for (s in source.screws) s.member]])
			for (member in rows) if (!used.exists(member)) throw "Unused foundation joint source member: "+member;
	}

	public static function check(source:FoundationJointSources,joint:FoundationJoint):Array<FoundationJointFinding> {
		var result:Array<FoundationJointFinding> = [];
		function add(code:String,subject:String,passed:Bool,message:String):Void
			result.push({code:code,subject:subject,passed:passed,message:message});
		for (seat in joint.seats) {
			var lower=plane(source,seat.lower),upper=plane(source,seat.upper);
			var normal=F.transformVector(lower,0,0,1),opposite=F.transformVector(upper,0,0,1);
			var dot=normal.x*opposite.x+normal.y*opposite.y+normal.z*opposite.z;
			add("normals",seat.id,dot<=-1+1e-10,"Joint seat planes must have opposing nominal normals.");
			var gap=(upper.x-lower.x)*normal.x+(upper.y-lower.y)*normal.y+(upper.z-lower.z)*normal.z;
			add("gap",seat.id,Math.abs(gap-joint.gapMm)<=joint.gapToleranceMm+1e-8,
				"Source seat separation "+gap+" mm; declared gap "+joint.gapMm+" +/- "+joint.gapToleranceMm+" mm.");
		}
		for (jack in joint.jacks) {
			var fastener=screw(source,jack.screw),receiver=hole(source,jack.receiver),thread=receiver.thread;
			add("jack-thread",jack.id,thread!=null && matches(fastener.thread,thread),"Jack requires a matching source threaded receiver.");
			result=result.concat(threadFit(source,jack.receiver,fastener,"jack-"+jack.id));
			var contact=plane(source,jack.bearing),head=fastener.bearingFrame;
			var tip=F.transformPoint(head,0,0,fastener.shankLength),normal=F.transformVector(contact,0,0,1);
			var shaft=F.transformVector(head,0,0,1),dot=shaft.x*normal.x+shaft.y*normal.y+shaft.z*normal.z;
			var distance=(tip.x-contact.x)*normal.x+(tip.y-contact.y)*normal.y+(tip.z-contact.z)*normal.z;
			add("jack-contact",jack.id,Math.abs(distance)<=requirement(source,jack.bearing).positionToleranceMm+1e-8 && dot<=-1+1e-10,
				"Jack tip must meet the source bearing plane along its inward normal; plane distance "+distance+" mm.");
		}
		for (anchor in joint.anchors) {
			var fastener=screw(source,anchor.screw),target=hole(source,anchor.target),thread=target.thread;
			add("anchor-thread",anchor.id,thread!=null && matches(fastener.thread,thread),"Anchor requires a matching source tapped hole.");
			result=result.concat(threadFit(source,anchor.target,fastener,"anchor-"+anchor.id));
			for (reference in anchor.through) {
				var clearance=hole(source,reference);
				add("anchor-clearance",anchor.id+"-"+haxe.crypto.Sha256.encode(reference.member+"/"+reference.feature),
					clearance.diameter>fastener.thread.majorDiameter && clearance.diameter+1e-8>=clearance.clearanceDiameter,
					"Anchor bore must provide the source's declared screw clearance: "+reference.member+"/"+reference.feature);
			}
		}
		return result;
	}
	static function threadFit(source:FoundationJointSources,reference:FoundationJointReference,
		fastener:FoundationJointScrew,subject:String):Array<FoundationJointFinding> {
		var target=hole(source,reference);
		if (target.thread==null) return []; // The parent reports a missing source tap as a design failure.
		var frame=F.compose(insert(source,reference.member).pose,target.frame);
		return ThreadedFastenerFit.checkIntervals(target,frame,
			{shankLength:fastener.shankLength,threadedLength:fastener.threadedLength,bearingFrame:fastener.bearingFrame},subject);
	}
	static function matches(a:InsertHole.InsertThread,b:InsertHole.InsertThread):Bool
		return !a.internal && b.internal && a.profile==b.profile && a.hand==b.hand && a.starts==b.starts
			&& Math.abs(a.majorDiameter-b.majorDiameter)<1e-6 && Math.abs(a.pitch-b.pitch)<1e-6;
	public static function body(source:FoundationJointSources,id:String):FoundationJointBody {
		var matches=source.bodies.filter(b -> b.member==id);
		if (matches.length!=1) throw "Unknown foundation joint source casting: "+id;return matches[0];
	}
	public static function insert(source:FoundationJointSources,id:String):FoundationJointInsert {
		var matches=source.inserts.filter(i -> i.member==id);
		if (matches.length!=1) throw "Unknown foundation joint source insert: "+id;return matches[0];
	}
	public static function screw(source:FoundationJointSources,id:String):FoundationJointScrew {
		var matches=source.screws.filter(s -> s.member==id);
		if (matches.length!=1) throw "Foundation joint screw is not an actual fastener: "+id;return matches[0];
	}
	public static function hole(source:FoundationJointSources,reference:FoundationJointReference):InsertHole {
		var matches=insert(source,reference.member).spec.holes.filter(h -> h.id==reference.feature);
		if (matches.length!=1) throw "Unknown hole "+reference.member+"/"+reference.feature;return matches[0];
	}
	public static function requirement(source:FoundationJointSources,reference:FoundationJointReference):InterfaceRequirement {
		var bodies=source.bodies.filter(b -> b.member==reference.member);
		var interfaces=bodies.length==1 ? bodies[0].interfaces : insert(source,reference.member).spec.interfaces;
		var matches=interfaces.filter(r -> r.id==reference.feature);
		if (matches.length!=1) throw "Unknown foundation joint precision interface: "+reference.member+"/"+reference.feature;return matches[0];
	}
	public static function plane(source:FoundationJointSources,reference:FoundationJointReference):AssemblyFrame {
		var bodies=source.bodies.filter(b -> b.member==reference.member);
		var pose=bodies.length==1 ? bodies[0].pose : insert(source,reference.member).pose;
		return switch requirement(source,reference).geometry {
			case Plane(frame):F.compose(pose,frame);
			case Line(_):throw "Foundation joint requires plane interfaces: "+reference.member+"/"+reference.feature;
		};
	}
	public static function belongs(source:FoundationJointSources,bodyId:String,reference:FoundationJointReference):Bool
		return reference.member==bodyId || body(source,bodyId).insertMembers.indexOf(reference.member)>=0;
	static function owned(source:FoundationJointSources,bodyId:String,reference:FoundationJointReference):Void
		if (!belongs(source,bodyId,reference)) throw "Foundation joint seat is not owned by casting "+bodyId+": "+reference.member;
}
