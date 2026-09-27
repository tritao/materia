package machinekit.component;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Part;
import materia.project.AssemblyRecord.AssemblyFrame;
import materia.project.MaterialLibrary;

/** Geometry generator, named connector frames, and a BOM line for one machine part.
 * `geometry()` returns a new owned Part in the component's CAD frame; the caller closes it.
 */
class MachineComponent {
	public final designation:String;
	public final materialId:String;
	public var bom(get, never):BomItem;
	final description:String;
	/** Null for code-only parts and assemblies outside the v1 recipe registry. */
	public var type(get, never):Null<ComponentType>;
	final connectorList:Array<Connector> = [];

	function new(designation:String, description:String, ?material:String) {
		if (designation == null || designation.length == 0) throw "Machine component needs a designation";
		this.designation = designation;
		materialId = MaterialLibrary.fromSpec(material);
		this.description = description;
	}

	function get_bom():BomItem {
		var recipe = type;
		var valuesKey = recipe == null ? null : recipe.key(values());
		return {partNumber: recipe == null ? designation : recipe.partNumber(this),
			description: description, quantity: 1,
			material: MaterialLibrary.require(materialId).physical.spec,
			typeId: recipe == null ? null : recipe.id, valuesKey: valuesKey};
	}

	public function geometry(detail:ComponentDetail = Preview):Part
		throw 'Component "$designation" does not generate geometry';

	function get_type():Null<ComponentType> return MachineKitComponents.forComponent(this);

	public function values():ComponentValues {
		var recipe = type;
		if (recipe == null) throw 'Component "$designation" is code-only';
		return recipe.valuesOf(this);
	}

	public function connectors():Array<Connector>
		return connectorList.copy();

	public function connector(name:String):Connector {
		for (connector in connectorList) if (connector.name == name) return connector;
		throw 'Missing connector "$designation/$name"';
	}

	/** Adds an instance with every connector frame so it can be mated by name. */
	public function addTo(model:AssemblyModel, id:String, ?pose:AssemblyFrame):Void {
		model.add(id, pose);
		for (connector in connectorList) model.connector(id, connector.name, connector.frame);
	}

	function addConnector(name:String, role:ConnectorRole, frame:AssemblyFrame):Void {
		for (existing in connectorList) if (existing.name == name)
			throw 'Duplicate connector "$designation/$name"';
		connectorList.push({name: name, role: role, frame: frame});
	}
}
