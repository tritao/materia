package materia.tooling.casting;

import materia.tooling.casting.ReplicationJig;
import materia.tooling.casting.ReplicationJig.ReplicationJigCodec;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;
import materia.casting.InterfaceRelationship;
import materia.casting.InterfaceRelationship.InterfaceRelationships as R;
import materia.casting.CastingInterfaceContext.InterfaceContextSource;
import materia.casting.CastingInterfaceContext.CastingInterfaceContexts;
import materia.casting.InterfaceRequirement;

/** Explicit jig inventory and prerequisites. Numeric support and reference checks
 * never stand in for rigidity, contact coverage or measured relationship evidence.
 */
class ReplicationJigs {
	public static function analyze(input:CastingToolingInput,authored:Array<ReplicationJig>,
		process:Array<CastingMoldPlan.PrecisionInterface>):Array<ToolingValidation> {
		var jigs=ReplicationJigCodec.snapshot(authored),result:Array<ToolingValidation> = [],context=input.interfaceContext;
		var coverage:Map<String,Array<String>> = [],sources:Map<String,InterfaceContextSource> = [];
		if (context!=null) for (source in context.sources) sources.set(CastingInterfaceContexts.sourceKey(source),source);
		for (jig in jigs) {
			var prefix="tooling.replication.jig."+jig.id,affected=[input.id,jig.id],scheme=input.datumScheme;
			result.push(check(prefix+".datum",affected,scheme==null ? ToolingCheckStatus.Pending
				: scheme.id==jig.datumSchemeId ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail,
				scheme==null ? "Declare the source foundation datum scheme before locating this jig."
				: scheme.id==jig.datumSchemeId ? "Jig references the source datum scheme; actual contacts remain unqualified."
				: "Jig names a different foundation datum scheme: "+jig.datumSchemeId));
			result.push(ToolingReadiness.pending(prefix+".rigidity",affected,
				"Qualify the common frame, support hardware, locks, stability and measured master relationships: "+jig.frameReference));
			result.push(check(prefix+".contact-geometry",affected,ToolingCheckStatus.Unsupported,
				"Master stock is recorded; jack contact geometry and full removal/collision qualification are not implemented. Native planar coverage is reported separately."));
			var keys:Array<String> = [];
			for (master in jig.masters) {
				var id=prefix+".master."+master.id,ids=affected.concat([master.id]);
				result.push(supports(id,ids,master));
				result.push(supportContacts(id,ids,master));
				var missing:Array<String> = [],invalid:Array<String> = [];
				for (member in master.members) {
					var key=R.memberKey(member);keys.push(key);
					var source=sources.get(key);
					if (source==null) {
						if (context==null) missing.push(key);else invalid.push("Unknown source member "+key);
						continue;
					}
					result.push(ReplicationPlacement.check(jig,master,source,context));
					switch source {
						case Precision(_,requirement,_):
							if (requirement.route==InterfaceRoute.Replicated && requirement.permittedFinishes.indexOf(master.finish)<0)
								invalid.push("Source interface "+key+" does not permit "+master.finish);
						case Axis(_,_,_):
							if (master.kind!=ReplicationMasterKind.AxisMaster) invalid.push("Machine axis requires an axis master: "+key);
					}
				}
				result.push(check(id+".source-members",ids,invalid.length>0 ? ToolingCheckStatus.Fail
					: missing.length>0 ? ToolingCheckStatus.Pending : ToolingCheckStatus.Pass,
					invalid.length>0 ? invalid.join("; ") : missing.length>0 ? "Assembly context is needed to resolve "+missing.join(", ")
					: "Master source members resolve and respect the source's permitted finishes."));
				if (master.releaseSpec==null || master.flatnessMm==null || master.inspectionReference==null)
					result.push(ToolingReadiness.pending(id+".inspection",ids,
						"Master release, measured flatness and inspection evidence must be declared; no flatness is assumed."));
				else result.push(ToolingReadiness.pending(id+".inspection",ids,
					"Bind master inspection and release evidence to its realized geometry before qualification: "+master.inspectionReference));
			}
			coverage.set(jig.id,keys);
		}
		for (choice in process) if (choice.jigReference!=null) {
			var found=jigs.filter(jig -> jig.id==choice.jigReference);
			if (found.length==0) result.push(ToolingReadiness.pending("tooling.replication.jig-reference."+choice.insertId,[choice.insertId],
				"Process references a jig whose authored inventory is missing: "+choice.jigReference));
			else if (context!=null) {
				var bindings=context.inserts.filter(binding -> binding.id==choice.insertId);
				var keys=coverage.get(found[0].id),insert=input.inserts.filter(insert -> insert.id==choice.insertId);
				if (bindings.length!=1 || insert.length!=1) throw "Replication jig requires a unique source insert binding";
				for (requirement in insert[0].interfaces) if (requirement.route==InterfaceRoute.Replicated
					&& keys.indexOf(R.memberKey(ComponentInterface(bindings[0].member,requirement.id)))<0)
					result.push(check("tooling.replication.jig-reference."+choice.insertId+"."+requirement.id,[choice.insertId,requirement.id],
						ToolingCheckStatus.Fail,"Selected jig does not carry the source replication interface: "+requirement.id));
			}
		}
		if (context!=null) for (instance in context.relationships) {
			var needsReplication=false;
			for (member in instance.relationship.members) {
				var source=sources.get(R.memberKey(member));
				if (source!=null) switch source {
					case Precision(_,requirement,_):if (requirement.route==InterfaceRoute.Replicated) needsReplication=true;
					case Axis(_,_,_):
				}
			}
			var common:Array<String> = [];
			for (jig in jigs) {
				var keys=coverage.get(jig.id),complete=true;
				for (member in instance.relationship.members) {
					if (keys.indexOf(R.memberKey(member))<0) complete=false;
					var source=sources.get(R.memberKey(member));
					if (source!=null) switch source {
						case Precision(owner,requirement,_):
							if (requirement.route==InterfaceRoute.Replicated) for (binding in context.inserts)
								if (binding.member==owner) {
									var choices=process.filter(choice -> choice.insertId==binding.id);
									if (choices.length!=1 || choices[0].jigReference!=jig.id) complete=false;
								}
						case Axis(_,_,_):
					}
				}
				if (complete) common.push(jig.id);
			}
			if (!needsReplication && common.length==0) continue;
			for (id in common) {
				var jig=jigs.filter(jig -> jig.id==id)[0];
				result.push(ReplicationRelationships.check(jig,instance.id,instance.relationship,context));
			}
			var rule="tooling.replication.relationship-frame."+haxe.crypto.Sha256.encode(instance.id);
			result.push(common.length==0 ? ToolingReadiness.pending(rule,[input.id],
				instance.id+": declare one common jig frame carrying every interface and axis master in the relationship.")
				: check(rule,[input.id],ToolingCheckStatus.Pass,
					instance.id+": source members share jig frame "+common.join(", ")+"; relationship geometry and measured alignment are checked separately."));
		}
		result.sort((a,b) -> Reflect.compare(a.rule,b.rule));return result;
	}

	public static function validate(input:CastingToolingInput,jigs:Array<ReplicationJig>,
		process:Array<CastingMoldPlan.PrecisionInterface>,actual:Array<ToolingValidation>):Void {
		var expected=analyze(input,jigs,process);
		var stored=actual.filter(check -> StringTools.startsWith(check.rule,"tooling.replication."));
		stored.sort((a,b) -> Reflect.compare(a.rule,b.rule));
		if (materia.tooling.ToolingCodec.canonical(expected)!=materia.tooling.ToolingCodec.canonical(stored))
			throw "Replication jig validation differs from source and authored jig inventory";
	}

	/** Numerical coincidence only; this is not a fabrication tolerance or a load check.
	 * Positions are already in master-blank coordinates, so this prerequisite is
	 * invariant under placement of the jig and master. Contact normals and the
	 * actual screw/pad geometry must be qualified separately.
	 */
	static function supportContacts(rule:String,ids:Array<String>,master:ReplicationMaster):ToolingValidation {
		var invalid:Array<String> = [];
		for (jack in master.jacks) if (!onBoundary(master.body,jack.position)) invalid.push(jack.id);
		if (master.jacks.length==0) return ToolingReadiness.pending(rule+".support-contact",ids,
			"Declare jack contacts before checking their location on the master stock.");
		return check(rule+".support-contact",ids.concat(invalid),invalid.length==0 ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail,
			invalid.length==0 ? "Declared jack points lie on the primitive master boundary; contact direction, hardware, stability and loads remain unqualified."
			: "Jack contact points must lie on the master boundary, not inside or outside its stock: "+invalid.join(", "));
	}

	static function onBoundary(body:materia.casting.CastingSolid,p:{x:Float,y:Float,z:Float}):Bool {
		var epsilon=1e-8;
		return switch body {
			case Box(width,depth,height):
				var dx=width/2-Math.abs(p.x),dy=depth/2-Math.abs(p.y);
				var bottom=p.z,top=height-p.z;
				dx>=-epsilon && dy>=-epsilon && bottom>=-epsilon && top>=-epsilon
					&& Math.min(Math.min(dx,dy),Math.min(bottom,top))<=epsilon;
			case Cylinder(diameter,height):
				var radial=diameter/2-Math.sqrt(p.x*p.x+p.y*p.y),bottom=p.z,top=height-p.z;
				radial>=-epsilon && bottom>=-epsilon && top>=-epsilon
					&& Math.min(radial,Math.min(bottom,top))<=epsilon;
		};
	}

	static function supports(rule:String,ids:Array<String>,master:ReplicationMaster):ToolingValidation {
		if (master.jacks.length!=3) return check(rule+".supports",ids,ToolingCheckStatus.Fail,
			"Each master requires exactly three declared jack contacts.");
		var p=master.jacks[0].position,q=master.jacks[1].position,r=master.jacks[2].position;
		var ax=q.x-p.x,ay=q.y-p.y,az=q.z-p.z,bx=r.x-p.x,by=r.y-p.y,bz=r.z-p.z;
		var cross={x:ay*bz-az*by,y:az*bx-ax*bz,z:ax*by-ay*bx};
		var aa=ax*ax+ay*ay+az*az,bb=bx*bx+by*by+bz*bz;
		if (aa==0 || bb==0 || cross.x*cross.x+cross.y*cross.y+cross.z*cross.z<=1e-16*aa*bb)
			return check(rule+".supports",ids,ToolingCheckStatus.Fail,"Master jack contacts must be distinct and non-collinear.");
		for (jack in master.jacks) if (jack.lockReference==null)
			return ToolingReadiness.pending(rule+".supports",ids,"Every jack requires an identified lock; actual support hardware remains unqualified.");
		return check(rule+".supports",ids,ToolingCheckStatus.Pass,
			"Three non-collinear jack contacts and lock references are declared; actual contacts and hardware remain unqualified.");
	}

	static function check(rule:String,ids:Array<String>,status:ToolingCheckStatus,message:String):ToolingValidation return {
		rule:rule,affectedIds:ids,status:status,severity:status==ToolingCheckStatus.Pass ? ToolingSeverity.Info : status==ToolingCheckStatus.Pending ? ToolingSeverity.Warning : ToolingSeverity.Error,
		message:message,evidence:[]};
}
