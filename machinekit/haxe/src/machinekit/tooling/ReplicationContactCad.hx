package machinekit.tooling;

import cadkit.modeling.Part;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;
import materia.casting.InterfaceRequirement;
import materia.casting.InterfaceRequirement.InterfaceRequirements;
import materia.casting.InterfaceRelationship.InterfaceRelationships as R;
import materia.casting.CastingInterfaceContext.InterfaceContextSource;
import materia.tooling.casting.CastingToolingInput;
import materia.tooling.casting.ReplicationJig;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;

private enum ContactTarget {
	RealizedContact(part:Part);
	ContactUnsupported(reason:String);
}

/** Native replication qualification uses the shared finite boundary checker. */
class ReplicationContactCad {
	public static function review(input:CastingToolingInput,jigs:Array<ReplicationJig>):Array<ToolingValidation> {
		var checks:Array<ToolingValidation> = [],context=input.interfaceContext;
		for (jig in jigs) for (master in jig.masters) {
			var prefix="tooling.replication.cad."+jig.id+"."+master.id;
			var ids=[input.id,jig.id,master.id];
			if (master.kind==ReplicationMasterKind.AxisMaster) {
				checks.push(result(prefix+".axis-contact",ids,ToolingCheckStatus.Unsupported,
					"Axis reference placement is checked separately; source shaft/connector contact geometry and arbor fixing are not recorded."));
				continue;
			}
			if (context==null) {
				checks.push(ToolingReadiness.pending(prefix+".context",ids,"Source assembly context is required for finite contact coverage."));
				continue;
			}
			for (member in master.members) {
				var key=R.memberKey(member),rule=prefix+".coverage."+haxe.crypto.Sha256.encode(key);
				var sources=context.sources.filter(source -> materia.casting.CastingInterfaceContext.CastingInterfaceContexts.sourceKey(source)==key);
				if (sources.length!=1) {
					checks.push(result(rule,ids,ToolingCheckStatus.Fail,"Unknown or ambiguous source contact member: "+key));continue;
				}
				var parts:Array<Part> = [];
				function own(part:Part):Part {parts.push(part);return part;}
				try {
					switch sources[0] {
						case Axis(_,_,_):checks.push(result(rule,ids,ToolingCheckStatus.Unsupported,"A plane master has no finite source-axis contact boundary."));
						case Precision(owner,requirement,pose):
							switch requirement.geometry {
								case Line(_):checks.push(result(rule,ids,ToolingCheckStatus.Unsupported,"Native reference-edge contact geometry is not implemented: "+key));
								case Plane(_):
									switch target(input,owner,requirement,parts) {
										case ContactUnsupported(reason):checks.push(result(rule,ids,ToolingCheckStatus.Unsupported,key+": "+reason));
										case RealizedContact(contact):
											var masterPose=F.compose(jig.pose,master.pose);
											var body=own(PosedParts.moved(own(machinekit.structural.CastingSolids.part(master.body)),masterPose));
											var report=PlanarContactCad.coverage(body,F.compose(masterPose,master.reference),contact,
												F.compose(pose,InterfaceRequirements.geometryFrame(requirement.geometry)));
											checks.push(result(rule,ids,report.status,key+": "+report.message));
									}
							}
					}
				} catch (error:Dynamic) {
					for (part in parts) part.close();
					throw error;
				}
				for (part in parts) part.close();
			}
		}
		checks.sort((a,b) -> Reflect.compare(a.rule,b.rule));return checks;
	}

	static function target(input:CastingToolingInput,owner:String,requirement:InterfaceRequirement,parts:Array<Part>):ContactTarget {
		function own(part:Part):Part {parts.push(part);return part;}
		var context=input.interfaceContext;
		if (context==null) return ContactUnsupported("Source assembly context is absent.");
		if (owner==context.bodyMember) {
			if (requirement.route==InterfaceRoute.Replicated) return ContactUnsupported("Foundation replication-layer geometry is not yet realized.");
			return RealizedContact(own(ToolingSourceCasting.fromInput(input).geometry()));
		}
		var bindings=context.inserts.filter(binding -> binding.member==owner);
		if (bindings.length!=1) return ContactUnsupported("External member's finite source geometry is absent from this casting snapshot.");
		var matches=input.inserts.filter(insert -> insert.id==bindings[0].id);
		if (matches.length!=1) throw "Source binding has no unique insert geometry: "+bindings[0].id;
		var insert=matches[0];
		var steel=switch materia.casting.InsertFinishing.assess(insert.body,insert.interfaces) {
			case Invalid(_),Unsupported(_):return ContactUnsupported("Resolve source finishing geometry before native contact review.");
			case Resolved(result):result;
		};
		if (requirement.route!=InterfaceRoute.Replicated)
			return RealizedContact(own(PosedParts.moved(own(ToolingInsertCad.serviceComponent(insert).geometry()),insert.pose)));
		var layers=steel.replicatedLayers.filter(layer -> layer.interfaceId==requirement.id);
		if (layers.length!=1) return ContactUnsupported("Source interface has no unique realized replication layer.");
		var b=layers[0].region,local=own(Part.box(b.width,b.depth,b.height));
		var layer=own(PosedParts.moved(local,F.translation(b.x+b.width/2,b.y+b.depth/2,b.z)));
		var protection=layerProtection(insert,b,layer,parts);
		return protection==null ? RealizedContact(own(PosedParts.moved(layer,insert.pose))) : ContactUnsupported(protection);
	}

	/** Detect only holes present before finishing whose opening can enter this
	 * layer. A blind back-face anchor is not a front-face replication penetration.
	 * Entry protection extends outward; through drilling is checked in both
	 * directions. These are protection envelopes, never substituted design cuts.
	 */
	static function layerProtection(insert:materia.tooling.casting.CastingToolingInput.CastingToolingInsert,
		box:materia.casting.Box3,layer:Part,parts:Array<Part>):Null<String> {
		function own(part:Part):Part {parts.push(part);return part;}
		for (hole in insert.holes) {
			var present=true;
			switch insert.origin {
				case Catalogue(_):
				case Fabricated(manufacturing):
					var choices=manufacturing.holeRoutes.filter(choice -> choice.holeId==hole.id);
					if (choices.length!=1) return "Hole manufacturing route is undecided: "+hole.id;
					present=choices[0].route==materia.casting.InsertManufacturing.InsertHoleRoute.BeforeCasting;
			}
			if (!present) continue;
			var radius=(hole.thread==null ? hole.diameter : hole.thread.majorDiameter)/2;
			if (hole.counterbore!=null) radius=Math.max(radius,hole.counterbore.diameter/2);
			if (hole.relief!=null) radius=Math.max(radius,hole.relief.diameter/2);
			var farthest=0.0;
			for (x in [box.x,box.x+box.width]) for (y in [box.y,box.y+box.depth]) for (z in [box.z,box.z+box.height]) {
				var dx=x-hole.frame.x,dy=y-hole.frame.y,dz=z-hole.frame.z;
				farthest=Math.max(farthest,Math.sqrt(dx*dx+dy*dy+dz*dz));
			}
			var travel=farthest+Math.sqrt(box.width*box.width+box.depth*box.depth+box.height*box.height);
			var ray=own(Part.cylinderSpan(radius,-travel,hole.extent==materia.casting.InsertHole.HoleExtent.Through ? travel : 0));
			var placed=own(PosedParts.moved(ray,hole.frame)),common=placed.shape.common(layer.shape);
			var intersects:Bool;
			try intersects=common.subshapeCount(CadKit.ShapeKind.Solid)>0
			catch (error:Dynamic) {common.close();throw error;}
			common.close();
			if (intersects) return "Replicated-layer bore protection is not realized for source hole: "+hole.id;
		}
		return null;
	}

	static function result(rule:String,ids:Array<String>,status:ToolingCheckStatus,message:String):ToolingValidation return {
		rule:rule,affectedIds:ids,status:status,severity:status==ToolingCheckStatus.Pass ? ToolingSeverity.Info : ToolingSeverity.Error,
		message:message,evidence:[]};
}
