package machinekit.tooling;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineComponent;
import machinekit.design.DesignInterfaces;
import machinekit.design.InterfaceSourceSupport;
import materia.casting.CastingInterfaceContext;
import materia.casting.CastingInterfaceContext.CastingInterfaceContexts;
import materia.casting.CastingInterfaceContext.InterfaceContextSource;
import materia.casting.InterfaceRelationship.InterfaceRelationships;
import materia.assembly.AssemblyFrames as F;

/** Explicit instance bindings preserve source ownership across assembly nesting.
 * This copies declared interface/axis facts, not casting strategies or tolerances.
 */
class FoundationAssemblyContext {
	public static function snapshot(assembly:MachineAssembly,bodyMember:String,
			inserts:Array<{id:String,member:String}>):CastingInterfaceContext {
		materia.casting.CastingRecordCodec.referencePath(bodyMember);
		if (inserts==null) throw "Interface context needs explicit insert bindings";
		var parts:Map<String,MachineComponent> = [];
		for (part in assembly.components()) parts.set(part.id,part.component);
		var body=parts.get(bodyMember);
		if (body==null || !Std.isOfType(body,machinekit.structural.MineralCasting))
			throw "Interface context body binding needs a source casting: "+bodyMember;
		var local:Map<String,Bool> = [bodyMember=>true];
		for (binding in inserts) {
			var part=parts.get(binding.member);
			if (part==null || !Std.isOfType(part,machinekit.structural.CastInsert))
				throw "Interface context insert binding needs a source insert: "+binding.member;
			local.set(binding.member,true);
		}
		var model=new AssemblyModel("mm");assembly.addTo(model,"");
		var definition=model.definition("casting-interface-context"),state=new AssemblyState(definition);
		state.forwardKinematics();
		var worldToFoundation=F.inverse(state.worldPose(bodyMember));
		var sources:Map<String,InterfaceContextSource> = [];
		for (partId in local.keys()) {
			var interfaces=DesignInterfaces.of(parts.get(partId));
			if (interfaces!=null) for (requirement in interfaces.precisionRequirements()) {
				var source:InterfaceContextSource=Precision(partId,requirement,F.compose(worldToFoundation,state.worldPose(partId)));
				sources.set(CastingInterfaceContexts.sourceKey(source),source);
			}
		}
		var relationships:Array<{id:String,relationship:materia.casting.InterfaceRelationship}> = [];
		for (requirement in assembly.designRequirements()) switch requirement {
			case InterfaceAlignment(id,relationship):
				var relevant=false;
				for (member in relationship.members) switch member {
					case ComponentInterface(part,_): if (local.exists(part)) relevant=true;
					case MachineAxis(_):
				}
				if (!relevant) continue;
				var copy=InterfaceRelationships.snapshot([relationship])[0];
				relationships.push({id:id,relationship:copy});
				for (member in copy.members) {
					var source:InterfaceContextSource=switch member {
						case ComponentInterface(part,face):
							Precision(part,InterfaceSourceSupport.precision(parts,part,face,copy.id),
								F.compose(worldToFoundation,state.worldPose(part)));
						case MachineAxis(axis):
							var joint=InterfaceSourceSupport.axis(definition,axis);
							Axis(axis,F.compose(worldToFoundation,state.worldConnector(joint.parent,joint.parentConnector)),
								{x:joint.axis.x,y:joint.axis.y,z:joint.axis.z});
					};
					sources.set(CastingInterfaceContexts.sourceKey(source),source);
				}
			default:
		}
		return CastingInterfaceContexts.snapshot({bodyMember:bodyMember,inserts:inserts,relationships:relationships,
			sources:[for (source in sources) source]});
	}
}
