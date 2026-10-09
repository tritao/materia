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

typedef ExtruderSpec = {
	var designation:String;
	/** The housing, mm, with the hobbed drive gear's shaft along its depth. */
	var width:Float;
	var height:Float;
	var depth:Float;
	/** Effective diameter of the hobbed gear where it grips the filament, mm: filament travels `pi * hob` per turn. */
	var hobDiameter:Float;
	var filamentDiameter:Float;
}

/**
 * The geared extruder body: housing, hobbed drive gear and spring idler. The reduction is not part of it: the
 * motor's pinion and the gear on the drive shaft are spur gears of the assembly, and the drive gear's angle is
 * what the extruder joint plans. Origin: the filament path at the grip, on the housing's front face (the side the
 * nozzle is on), filament along +Z downwards towards -Z; the drive shaft runs along +Y behind it.
 */
class Extruder extends MachineComponent {
	static var table:Null<Catalog<ExtruderSpec>>;
	static var recipe:Null<ComponentType>;
	public final spec:ExtruderSpec;

	public static function catalog():Catalog<ExtruderSpec> {
		if (table == null) table = new Catalog("extruder", s -> s.designation, [
			{designation: "GEARED-HOB-7.3", width: 42.0, height: 44.0, depth: 22.0, hobDiameter: 7.3, filamentDiameter: 1.75}
		], _ -> ({source: "Assumed geared extruder with a 7.3 mm hobbed gear; no vendor verification", standard: null,
			standardEdition: null, dimensionKind: Unverified, conformance: GenericApproximation, verifiedFields: []}));
		return table;
	}

	public function new(designation:String = "GEARED-HOB-7.3") {
		var row = catalog().get(designation);
		super(designation, 'Geared extruder, ${Std.string(row.hobDiameter)} mm hob', "aluminium 6061");
		spec = row;
		addConnector("filamentGrip", Axis, Solids.axial(0, 0, 0));
		addConnector("driveShaft", Axis, Solids.axial(row.hobDiameter / 2, 0, 0));
		addConnector("mount", Mount, Solids.axial(0, 0, -row.height / 2));
	}

	/** Radians the hobbed gear turns for each millimetre of filament it feeds. */
	public function rotationPerMillimetre():Float return 2.0 / spec.hobDiameter;

	/** Millimetres of filament for each radian of the drive gear. */
	public function filamentPerRadian():Float return spec.hobDiameter / 2.0;

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.printing.extruder", [
			ComponentRecipeSupport.catalog("designation", catalog(), "GEARED-HOB-7.3")
		], v -> new Extruder(v.token("designation")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return Std.isExactType(this, Extruder) ? recipeType() : null;
	override public function values():ComponentValues
		return new ComponentValues().setToken("designation", designation).setToken("material", materialSpec());
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		// The housing is centred on the filament axis; the drive shaft's bore runs through it along y beside the filament.
		var body = Solids.named(Part.box(spec.width, spec.depth, spec.height), "housing");
		if (detail == Envelope) return body;
		var shaftX = spec.hobDiameter / 2 + spec.filamentDiameter / 2;
		var bore = Solids.named(Part.cylinderAlongY(1.1, -spec.depth, spec.depth, shaftX, 0), "bore");
		var path = Solids.named(Part.cylinderSpan(spec.filamentDiameter / 2 + 0.1, -spec.height, spec.height), "filament");
		return Solids.cut(body, [bore, path]);
	}
}
