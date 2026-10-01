import haxe.io.Bytes;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import materia.project.MaterialLibrary;
import materia.units.LengthUnit;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import cadkit.modeling.Part;
import cadkit.modeling.AssemblyModel;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.TypedProperty;
import machinekit.document.MachineKitDocuments;
import machinekit.document.MachineKitRecipes;

/** Materia project entrypoint for the MachineKit motor/shaft/bearing assembly. */
class MotorShaftBearingsPreview {
	/** Editable document for every registered part occurrence in the example. */
	public static function document():Document {
		var result = new Document();
		var example = new MotorShaftBearings();
		var definitions = new Map<String, cadkit.parametric.Definition>();
		for (entry in example.components()) {
			var recipe = entry.component.type;
			if (recipe == null) continue;
			var identity = recipe.key(entry.component.values());
			var definition = definitions.get(identity);
			if (definition == null) {
				definition = MachineKitDocuments.define(result, recipe, entry.component.values());
				definitions.set(identity, definition);
			}
			var instance = result.createInstance(entry.id, definition);
			instance.restoreProperty("machinekit.occurrence",
				TypedProperty.text("machinekit.occurrence", entry.id));
		}
		return result;
	}

	/** A saved document can be edited and passed back to rebuild the viewport. */
	public static function preview(?savedDocument:String):Bytes {
		MachineKitRecipes.register();
		var editable = document();
		try {
			var diagnostics:Array<String> = [];
			if (savedDocument != null) {
				MachineKitRecipes.reconcileDocument(editable, savedDocument, diagnostics);
			}
			var result = previewDocument(editable, diagnostics);
			MachineKitRecipes.forget(editable);
			editable.close();
			return result;
		} catch (error:Dynamic) {
			MachineKitRecipes.forget(editable);
			editable.close();
			throw error;
		}
	}

	static function previewDocument(editable:Document, diagnostics:Array<String>):Bytes {
		var example = new MotorShaftBearings();
		var instances = new Map<String, InstanceElement>();
		for (element in editable.allElements()) if (element.kind == "instance") {
			var instance:InstanceElement = cast element;
			var identity = instance.property("machinekit.occurrence");
			var id:String = identity == null ? instance.name : cast identity.value;
			if (instances.exists(id)) throw 'Duplicate MachineKit occurrence "$id"';
			instances.set(id, instance);
		}
		var model = example.assembly(instances);
		var parts:Array<SceneArtifactPart> = [];
		var definitionByOccurrence = new Map<String, String>();
		var definitionByIdentity = new Map<String, String>();
		for (entry in example.components()) {
			var instance = instances.get(entry.id);
			var identity = instance == null ? entry.component.designation :
				instance.definitionId.value + ":" + MachineKitRecipes.component(instance).type.key(
					MachineKitRecipes.component(instance).values());
			var definitionId = definitionByIdentity.get(identity);
			if (definitionId == null) {
				definitionId = entry.id;
				definitionByIdentity.set(identity, definitionId);
				if (instance == null) {
					addPart(parts, definitionId, entry.component.designation,
						entry.component.geometry(ComponentDetail.Preview), entry.component.materialId);
				} else {
					var component = MachineKitRecipes.component(instance);
					addPart(parts, definitionId, component.designation,
						new Part(editable.definitionOutput(instance, "body").cloneShape()), component.materialId);
				}
			}
			definitionByOccurrence.set(entry.id, definitionId);
		}
		for (bearingId in ["bearingA", "bearingB"]) {
			var shieldId = bearingId + "-shields";
			model.add(shieldId);
			model.connector(shieldId, "front", Solids.axial(0, 0, 0));
			model.mate(shieldId + "-seat", "fixed", bearingId, "front", shieldId, "front");
			definitionByOccurrence.set(shieldId, "bearingA-shields");
			if (bearingId == "bearingA") addPart(parts, shieldId,
				example.bearing.designation + " shields", example.bearing.shieldGeometry(), "black-oxide");
		}
		var definition = model.definition("motor-shaft-bearings");
		var state = model.initialState("motor-shaft-bearings").record();
		definition.definitions = [for (component in definition.definitions)
			if (definitionByOccurrence.get(component.id) == component.id) component];
		for (occurrence in definition.occurrences)
			occurrence.definition = definitionByOccurrence.get(occurrence.id);
		return SceneArtifact.encode({lengthUnit: "mm",
			metresPerUnit: LengthUnit.metresPerUnit("mm"), parts: parts,
			assembly: model.record(), assemblyDefinition: definition,
			assemblyState: state, recipeDocument: DocumentCodec.encode(editable), recipeDiagnostics: diagnostics});
	}

	static function addPart(parts:Array<SceneArtifactPart>, id:String, name:String, part:Part,
			materialId:String):Void {
		var material = MaterialLibrary.require(materialId);
		var color = material.visual.baseColor;
		try {
			var physical = part.massProperties();
			var mesh = part.shape.tessellateRelative();
			parts.push({
				id: id, name: name, red: color[0], green: color[1], blue: color[2],
				appearance: MaterialLibrary.appearance(materialId), materialId: materialId, vertexCount: mesh.vertexCount, indexCount: mesh.indexCount,
				volume: physical.volume,
				centerOfMass: [physical.centerOfMass.x, physical.centerOfMass.y, physical.centerOfMass.z],
				vertices: mesh.vertices, normals: mesh.normals, indices: mesh.indices,
				edgeSegments: mesh.edgeSegments, edgeIds: mesh.edgeIds,
				faceDescriptors: cadkit.parametric.GeometricConnectors.describeFaces(part.shape),
				faceRanges: [for (range in mesh.faceRanges) {
					faceIndex: range.faceIndex, firstIndex: range.firstIndex, indexCount: range.indexCount
				}]
			});
		} catch (error:Dynamic) {
			part.close();
			throw error;
		}
		part.close();
	}
}

function main():Void {
	var scene = SceneArtifact.decode(MotorShaftBearingsPreview.preview());
	var definition = scene.assemblyDefinition;
	if (scene.parts.length != 8 || definition == null || definition.occurrences.length != 13)
		throw "Bearing preview has missing render regions";
	for (part in scene.parts) if (part.materialId == null || part.materialDensity == null ||
		part.materialDensity <= 0 || part.materialSpec == null || part.volume == null ||
		part.volume <= 0 || part.centerOfMass == null || part.inertia == null || part.inertia.length != 9)
		throw 'Bearing preview has no resolved material for "${part.id}"';
	var shield:Null<SceneArtifactPart> = null;
	for (part in scene.parts) if (part.id == "bearingA-shields") shield = part;
	if (shield == null || shield.appearance == null || shield.appearance.finish != "black-oxide")
		throw "Bearing preview has no shared shield finish";
	for (bearingId in ["bearingA", "bearingB"]) {
		var shieldId = bearingId + "-shields";
		var attached = false, shared = false;
		for (joint in definition.joints)
			if (joint.parent == bearingId && joint.child == shieldId) attached = true;
		for (occurrence in definition.occurrences)
			if (occurrence.id == shieldId && occurrence.definition == "bearingA-shields") shared = true;
		if (!attached || !shared) throw 'Bearing shield "$shieldId" does not share its geometry';
	}
	Sys.println('Generated ${scene.parts.length} preview regions');
}
