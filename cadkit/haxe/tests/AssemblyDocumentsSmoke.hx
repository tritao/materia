import cadkit.modeling.AssemblyState;
import cadkit.parametric.AssemblyDocuments;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.QuantityKind;
import cadkit.parametric.TypedProperty;
import haxeon.Equality;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;

class AssemblyDocumentsSmoke {
	public static function run():Void {
		var frame = AssemblyFrames.identity();
		var limits:materia.assembly.AssemblyDefinition.AssemblyJointLimits =
			{lower: 0.0, upper: 100.0, velocity: null, effort: null};
		var original:AssemblyDefinition = {schemaVersion: AssemblyDefinitionCodec.VERSION, id: "document-assembly",
			definitions: [{id: "root-part", connectors: [{name: "mount", frame: frame}]},
				{id: "follower-part", connectors: [{name: "base", frame: frame}]}],
			occurrences: [{id: "root", definition: "root-part", initialPose: frame},
				{id: "module", definition: "module-design", assembly: "module-design", initialPose: frame},
				{id: "follower", definition: "follower-part", initialPose: frame}],
			joints: [{id: "slide", type: AssemblyJointType.Prismatic, role: AssemblyJointRole.Tree,
				parent: "root", parentConnector: "mount", child: "module", childConnector: "base",
				axis: {x: 0, y: 1, z: 0}, limits: limits, defaultValue: 0},
				{id: "follow", type: AssemblyJointType.Prismatic, role: AssemblyJointRole.Tree,
				parent: "module", parentConnector: "tip", child: "follower", childConnector: "base",
				axis: {x: 0, y: 1, z: 0}, limits: limits, defaultValue: 0}],
			couplings: [{id: "linked", source: "slide", target: "follow", ratio: 1, offset: 0}],
			assemblies: [{id: "module-design", definitions: [{id: "body-part", connectors: [
				{name: "base", frame: frame}, {name: "tip", frame: frame}]}],
				occurrences: [{id: "body", definition: "body-part", initialPose: frame}], joints: [],
				exposedConnectors: [{name: "base", occurrence: "body", connector: "base"},
					{name: "tip", occurrence: "body", connector: "tip"}]}]};
		var document = new Document();
		var root = AssemblyDocuments.fromDefinition(document, original);
		if (!Equality.equals(AssemblyDocuments.toDefinition(root), original))
			throw "Assembly document changed the mechanical definition";
		var savedText = DocumentCodec.encode(document);
		var cloned = DocumentCodec.decode(savedText, true, false);
		if (!Equality.equals(AssemblyDocuments.toDefinition(cloned.element(root.id)), original))
			throw "Cloning lost assembly ownership references";
		cloned.close();
		var reopened = DocumentCodec.decode(savedText, false, false);
		var loadedRoot = reopened.element(root.id);
		var rebuilt = AssemblyDocuments.toDefinition(loadedRoot);
		if (!Equality.equals(rebuilt, original)) throw "Saved assembly document changed the mechanical definition";
		var joints = 0, couplings = 0;
		for (relationship in reopened.allRelationships()) {
			if (relationship.typeName == AssemblyDocuments.JOINT) {
				joints++;
				var idProperty = relationship.property("cadkit.assembly.id");
				if (idProperty != null && idProperty.value == "slide")
					relationship.setProperty(TypedProperty.quantity("cadkit.assembly.defaultValue", QuantityKind.Scalar, 25, "1"));
			} else if (relationship.typeName == AssemblyDocuments.COUPLING) couplings++;
		}
		if (joints != 2 || couplings != 1) throw "Assembly relationships were not saved";
		rebuilt = AssemblyDocuments.toDefinition(loadedRoot);
		if (Math.abs(new AssemblyState(rebuilt).worldPose("module/body").y - 25) > 1e-9)
			throw "Editing a joint property did not change the solved pose";
		var edited = DocumentCodec.decode(DocumentCodec.encode(reopened), false, false);
		var savedEdit = AssemblyDocuments.toDefinition(edited.element(root.id));
		if (Math.abs(new AssemblyState(savedEdit).worldPose("module/body").y - 25) > 1e-9)
			throw "The edited joint property was not saved";
		edited.close();
		reopened.close();
		document.close();
	}
}
