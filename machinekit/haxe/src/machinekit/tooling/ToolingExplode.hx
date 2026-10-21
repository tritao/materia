package machinekit.tooling;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames as F;
import materia.presentation.AssemblyPresentation;
import materia.presentation.Presentations;

/** Converts producer review displacements to the presentation's parent-frame coordinates. */
class ToolingExplode {
	public static function presentation(definition:AssemblyDefinition, producer:String,
			displacements:Map<String,materia.assembly.AssemblyDefinition.AssemblyVector>):AssemblyPresentation {
		var result=Presentations.assembled(definition,-1.1,0.35,producer);
		var flat=AssemblyDefinitionFlattener.flattenView(definition);
		var poses=new Map<String,materia.assembly.AssemblyRecord.AssemblyFrame>();
		var library=new Map<String,materia.assembly.AssemblyDefinition.AssemblySubdefinition>();
		if(definition.assemblies!=null)for(entry in definition.assemblies)library.set(entry.id,entry);
		function collect(members:Array<materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence>,
				prefix:String,base:materia.assembly.AssemblyRecord.AssemblyFrame):Void {
			for(member in members) {
				var id=prefix+member.id,pose=F.compose(base,member.initialPose);poses.set(id,pose);
				if(member.assembly!=null) {
					var nested=library.get(member.assembly);if(nested==null)throw "Missing tooling assembly scope";
					collect(nested.occurrences,id+"/",pose);
				}
			}
		}
		collect(definition.occurrences,"",F.identity());
		for(part in flat.occurrences) {
			var delta=displacements.get(part.id);if(delta==null)continue;
			var parent:Null<String> = null;
			for(joint in flat.joints)if(joint.role=="tree"&&joint.child==part.id)parent=joint.parent;
			if(parent==null&&part.id.lastIndexOf("/")>=0)parent=part.id.substr(0,part.id.lastIndexOf("/"));
			var pose=parent==null?null:poses.get(parent);
			var local=pose==null?delta:F.transformVector(F.inverse(pose),delta.x,delta.y,delta.z);
			var length=Math.sqrt(local.x*local.x+local.y*local.y+local.z*local.z);
			if(length>0)result.explode.push({select:{occurrence:part.id,subtree:false},
				direction:{x:local.x/length,y:local.y/length,z:local.z/length},distance:length,stage:1,source:Generated(producer)});
		}
		for(id in displacements.keys())if(!poses.exists(id))throw 'Unknown tooling explode occurrence "$id"';
		materia.presentation.PresentationCodec.validate(definition,result);
		return result;
	}
}
