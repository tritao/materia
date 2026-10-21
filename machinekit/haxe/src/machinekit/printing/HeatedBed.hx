package machinekit.printing;

import cadkit.modeling.Part;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

typedef HeatedBedSpec = {
	var designation:String;
	/** The aluminium plate and the silicone heater pad stuck under it, mm. */
	var width:Float;
	var depth:Float;
	var plateThickness:Float;
	var padThickness:Float;
	var volts:Float;
	var watts:Float;
	/** Thermal model (assumed): heat capacity of plate and pad, J/K; heat lost to still air at the surface, W/K. */
	var heatCapacity:Float;
	var lossCoefficient:Float;
	var maxTemperature:Float;
}

/**
 * A heated bed: an aluminium plate with a silicone heater under it. Origin: the middle of the plate's top
 * face, whose normal is +Z; `surface` is there. The plate's underside carries `mount`.
 */
class HeatedBed extends MachineComponent {
	static var table:Null<Catalog<HeatedBedSpec>>;
	static var recipe:Null<ComponentType>;
	public final spec:HeatedBedSpec;

	public static function catalog():Catalog<HeatedBedSpec> {
		if (table == null) table = new Catalog("heated bed", s -> s.designation, [
			{designation: "BED-ALU-235-24V-300W", width: 235.0, depth: 235.0, plateThickness: 4.0, padThickness: 1.5,
				volts: 24.0, watts: 300.0, heatCapacity: 640.0, lossCoefficient: 1.4, maxTemperature: 110.0}
		], _ -> ({source: "Assumed 235 mm aluminium bed with a 24 V silicone heater; thermal values are not measured", standard: null,
			standardEdition: null, dimensionKind: Unverified, conformance: GenericApproximation, verifiedFields: []}));
		return table;
	}

	public function new(designation:String = "BED-ALU-235-24V-300W") {
		var row = catalog().get(designation);
		super(designation, 'Heated bed, ${Std.string(Math.round(row.width))} mm, ${Std.string(Math.round(row.volts))} V, ${Std.string(Math.round(row.watts))} W',
			"aluminium 6061");
		spec = row;
		addConnector("surface", Face, Solids.axial(0, 0, 0));
		addConnector("mount", Mount, Solids.axial(0, 0, -row.plateThickness - row.padThickness));
		addPort({name: "heater", kind: ElectricalPower, role: Consumer, iface: Unspecified, required: false});
	}

	public function resistance():Float return spec.volts * spec.volts / spec.watts;

	public function thickness():Float return spec.plateThickness + spec.padThickness;

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.printing.heated-bed", [
			ComponentRecipeSupport.catalog("designation", catalog(), "BED-ALU-235-24V-300W")
		], v -> new HeatedBed(v.token("designation")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return Std.isExactType(this, HeatedBed) ? recipeType() : null;
	override public function values():ComponentValues
		return new ComponentValues().setToken("designation", designation).setToken("material", materialSpec());
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		// The envelope is the whole slab, heater pad included: the pad below the plate takes up real space.
		if (detail == Envelope)
			return Solids.named(Part.box(spec.width, spec.depth, spec.plateThickness + spec.padThickness)
				.translated(new cadkit.modeling.Vector(0, 0, -spec.plateThickness - spec.padThickness)), "bed-envelope");
		var plate = Solids.named(Part.box(spec.width, spec.depth, spec.plateThickness).translated(new cadkit.modeling.Vector(0, 0, -spec.plateThickness)), "plate");
		var pad = Solids.named(Part.box(spec.width, spec.depth, spec.padThickness)
			.translated(new cadkit.modeling.Vector(0, 0, -spec.plateThickness - spec.padThickness)), "heater");
		return Solids.union([plate, pad]);
	}
}
