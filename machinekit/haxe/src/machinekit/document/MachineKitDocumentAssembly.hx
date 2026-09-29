package machinekit.document;

import cadkit.modeling.AssemblyModel;
import cadkit.parametric.DefinitionEvaluatorRegistry;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.PlacementFrames;

/** Adds a document instance and its named connector outputs to a kinematics assembly. */
class MachineKitDocumentAssembly {
	public static function add(model:AssemblyModel, id:String, instance:InstanceElement):Void {
		var definition = instance.document.definition(instance.definitionId);
		model.add(id, PlacementFrames.toAssemblyFrame(instance.document.worldPlacement(instance)));
		for (name in instance.connectorNames())
			model.connector(id, name, PlacementFrames.toAssemblyFrame(DefinitionEvaluatorRegistry.connector(definition, instance, name)));
	}
}
