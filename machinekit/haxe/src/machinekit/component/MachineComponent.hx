package machinekit.component;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.MassProperties.MassSource;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.MaterialLibrary;

/** Geometry generator, named connector frames, and a BOM line for one machine part.
 * `geometry()` returns a new owned Part in the component's CAD frame; the caller closes it.
 */
class MachineComponent {
	public final designation:String;
	public var materialId:String;
	/** True when this component was built from explicit, non-catalog specifications. */
	public final codeOnly:Bool;
	public var bom(get, never):BomItem;
	public final description:String;
	var cachedBom:Null<BomItem>;
	var cachedMass:Null<MassProperties>;
	var cachedMassMaterialId:Null<String>;
	var declaredMass:Null<MassProperties>;
	/** Null for code-only parts and assemblies outside the v1 recipe registry. */
	public var type(get, never):Null<ComponentType>;
	final connectorList:Array<Connector> = [];
	final portList:Array<ComponentPort> = [];
	final bridgeList:Array<PortBridge> = [];

	function new(designation:String, description:String, ?material:String, codeOnly:Bool = false) {
		if (designation == null || designation.length == 0) throw "Machine component needs a designation";
		this.designation = designation;
		materialId = MaterialLibrary.fromSpec(material);
		this.codeOnly = codeOnly;
		this.description = description;
	}

	public static function customDesignation(designation:String):String
		return StringTools.startsWith(designation, "CUSTOM-") ? designation : 'CUSTOM-$designation';

	public function materialSpec():String return MaterialLibrary.require(materialId).physical.spec;

	public function setMaterial(spec:String):Void {
		materialId = MaterialLibrary.fromSpec(spec);
		cachedBom = null;
		cachedMass = null;
		cachedMassMaterialId = null;
	}

	function get_bom():BomItem {
		if (cachedBom != null) return cachedBom;
		var recipe = type;
		var valuesKey = recipe == null ? null : recipe.key(values());
		cachedBom = {partNumber: recipe == null ? designation : recipe.partNumber(this),
			description: description, quantity: 1,
			material: MaterialLibrary.require(materialId).physical.spec,
			typeId: recipe == null ? null : recipe.id, valuesKey: valuesKey};
		return cachedBom;
	}

	public function geometry(detail:ComponentDetail = Preview):Part
		throw 'Component "$designation" does not generate geometry';

	/** The preview shape supplies volume and centroid; its density is kg/m³. */
	public function massProperties():MassProperties {
		if (declaredMass != null) return declaredMass;
		if (cachedMass != null && cachedMassMaterialId == materialId) return cachedMass;
		var part:Part;
		try part = geometry(Preview) catch (error:Dynamic) {
			if (Std.string(error) == 'Component "$designation" does not generate geometry')
				throw 'Component "$designation" has no geometry or declared mass';
			throw error;
		}
		try {
			var physical = part.massProperties();
			var mass = physical.volume * 1e-9 * MaterialLibrary.require(materialId).physical.density;
			cachedMass = new MassProperties(mass, physical.centerOfMass, Computed(Preview));
			cachedMassMaterialId = materialId;
		} catch (error:Dynamic) {
			part.close();
			throw error;
		}
		part.close();
		return cachedMass;
	}

	/** Vendor mass overrides the geometry estimate, including after material changes. */
	function declareMass(kg:Float, ?centreOfMass:Vector):Void {
		if (!Math.isFinite(kg) || kg <= 0) throw 'Component "$designation" needs a positive declared mass';
		declaredMass = new MassProperties(kg, centreOfMass == null ? new Vector() : centreOfMass, Declared);
	}

	public function toolSpecs():Array<ToolSpec> return [];

	public function tool(name:String, values:ComponentValues):Part {
		for (spec in toolSpecs()) if (spec.name == name) return buildTool(name, spec.resolve(values));
		throw 'Unknown tool "$name" for "$designation"';
	}

	function buildTool(name:String, values:ComponentValues):Part
		throw 'Unknown tool "$name" for "$designation"';

	public function componentType():Null<ComponentType> return null;

	function get_type():Null<ComponentType> return componentType();

	public function values():ComponentValues
		throw 'Component "$designation" is code-only';

	public function connectors():Array<Connector>
		return connectorList.copy();

	public function connector(name:String):Connector {
		for (connector in connectorList) if (connector.name == name) return connector;
		throw 'Missing connector "$designation/$name"';
	}

	public function ports():Array<ComponentPort> return portList.copy();

	public function port(name:String):ComponentPort {
		for (entry in portList) if (entry.name == name) return entry;
		throw 'Missing port "$designation/$name"';
	}

	public function bridges():Array<PortBridge> return bridgeList.copy();

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

	function addPort(entry:ComponentPort):Void {
		if (entry == null || entry.name == null || entry.name.length == 0 || entry.kind == null ||
			entry.role == null || entry.iface == null) throw 'Invalid port on "$designation"';
		switch entry.iface {
			case PushIn(tubeOd): if (!Math.isFinite(tubeOd) || tubeOd <= 0) throw 'Invalid port interface "$designation/${entry.name}"';
			case Thread(name): if (name == null || name.length == 0) throw 'Invalid port interface "$designation/${entry.name}"';
			case Plug(name, pins): if (name == null || name.length == 0 || pins <= 0) throw 'Invalid port interface "$designation/${entry.name}"';
			case Unspecified:
		}
		for (existing in portList) if (existing.name == entry.name)
			throw 'Duplicate port "$designation/${entry.name}"';
		if (entry.connector != null) connector(entry.connector);
		portList.push(entry);
	}

	function addBridge(from:String, to:String):Void {
		var input = port(from), output = port(to);
		if (from == to || input.kind != output.kind)
			throw 'Port bridge "$designation/$from->$to" needs distinct ports of the same kind';
		for (existing in bridgeList) if (existing.from == from && existing.to == to)
			throw 'Duplicate port bridge "$designation/$from->$to"';
		bridgeList.push({from: from, to: to});
	}
}
