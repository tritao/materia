package machinekit.printing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

typedef FanSpec = {
	var designation:String;
	/** True for a blower, whose air leaves sideways through the nozzle at the end of its housing. */
	var blower:Bool;
	var width:Float;
	var depth:Float;
	var height:Float;
	var volts:Float;
	var watts:Float;
}

/**
 * A DC fan or blower. Origin: the middle of the underside of the housing, with the fan's axis along +Z. A
 * blower's outlet is the face on +X; `outlet` is there. Rows are assumed 24 V parts.
 */
class Fan extends MachineComponent {
	static var table:Null<Catalog<FanSpec>>;
	static var recipe:Null<ComponentType>;
	public final spec:FanSpec;

	public static function catalog():Catalog<FanSpec> {
		if (table == null) table = new Catalog("fan", s -> s.designation, [
			{designation: "FAN-4010-24V", blower: false, width: 40.0, depth: 40.0, height: 10.0, volts: 24.0, watts: 1.5},
			{designation: "BLOWER-5015-24V", blower: true, width: 50.0, depth: 50.0, height: 15.0, volts: 24.0, watts: 2.4}
		], _ -> ({source: "Assumed common axial fan and blower sizes; no vendor verification", standard: null,
			standardEdition: null, dimensionKind: Unverified, conformance: GenericApproximation, verifiedFields: []}));
		return table;
	}

	public function new(designation:String = "FAN-4010-24V") {
		var row = catalog().get(designation);
		super(designation, '${row.blower ? "Blower" : "Fan"}, ${Std.string(Math.round(row.width))} x ${Std.string(Math.round(row.height))} mm, ${Std.string(Math.round(row.volts))} V', "plastic");
		spec = row;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		if (row.blower) addConnector("outlet", Face, Solids.axial(row.width / 2, 0, row.height / 2));
		addPort({name: "power", kind: ElectricalPower, role: Consumer, iface: Unspecified, required: false});
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.printing.fan", [
			ComponentRecipeSupport.catalog("designation", catalog(), "FAN-4010-24V")
		], v -> new Fan(v.token("designation")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return Std.isExactType(this, Fan) ? recipeType() : null;
	override public function values():ComponentValues
		return new ComponentValues().setToken("designation", designation).setToken("material", materialSpec());
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		if (!spec.blower || detail == Envelope)
			return Solids.named(Part.box(spec.width, spec.depth, spec.height), "housing");
		// The volute is a cylinder with a duct leaving it towards +X.
		var wheel = Solids.named(Part.cylinderSpan(spec.width / 2 - 1, 0, spec.height), "volute");
		var duct = Solids.named(Part.box(spec.width / 2, spec.depth * 0.4, spec.height).translated(new Vector(spec.width / 4, 0, 0)), "duct");
		return Solids.union([wheel, duct]);
	}
}
