package machinekit.standard;

import cadkit.modeling.Part;
import machinekit.catalog.Catalog;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Set-screw shaft collar, sized by bore diameter, for an axial stop clamped onto a shaft.
 * CAD frame: axis along +Z, spanning z=0..width, matching `HexNut`/`FlatWasher`. Connectors:
 * `front`, `back` (faces) and `axis` (mid-width), all with +Y along +Z.
 */
class ShaftCollar extends MachineComponent {
	static var table:Null<Catalog<ShaftCollarSpec>>;

	public final spec:ShaftCollarSpec;
	public var boreDiameter(get, never):Float;
	public var width(get, never):Float;

	static function rows():Array<ShaftCollarSpec>
		return [
			{boreDiameter: 5, outerDiameter: 12, width: 8, setScrew: "M3"},
			{boreDiameter: 6, outerDiameter: 14, width: 9, setScrew: "M4"},
			{boreDiameter: 8, outerDiameter: 16, width: 11, setScrew: "M4"},
			{boreDiameter: 10, outerDiameter: 22, width: 13, setScrew: "M5"},
			{boreDiameter: 12, outerDiameter: 25, width: 15, setScrew: "M5"},
			{boreDiameter: 15, outerDiameter: 28, width: 17, setScrew: "M6"},
			{boreDiameter: 17, outerDiameter: 32, width: 19, setScrew: "M6"},
			{boreDiameter: 20, outerDiameter: 35, width: 21, setScrew: "M6"},
			{boreDiameter: 25, outerDiameter: 40, width: 23, setScrew: "M8"},
		];

	public static function catalog():Catalog<ShaftCollarSpec> {
		if (table == null)
			table = new Catalog("shaft collar bore diameter", spec -> Dimension.format(spec.boreDiameter), rows());
		return table;
	}

	/** Collar for exactly `boreDiameter`; throws when the catalog has no such size. */
	public static function forShaft(boreDiameter:Float):ShaftCollar
		return new ShaftCollar(catalog().get(Dimension.format(boreDiameter)));

	public function new(spec:ShaftCollarSpec) {
		if (!(spec.boreDiameter > 0) || !(spec.outerDiameter > spec.boreDiameter) || !(spec.width > 0))
			throw 'Shaft collar for ${Dimension.format(spec.boreDiameter)} mm shaft has inconsistent dimensions';
		var bore = Dimension.format(spec.boreDiameter);
		super('COLLAR-$bore', 'Shaft collar for $bore mm shaft, ${spec.setScrew} set screw', "steel");
		this.spec = spec;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, spec.width / 2));
		addConnector("back", Face, Solids.axial(0, 0, spec.width));
	}

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.cut(Solids.cylinder(spec.outerDiameter / 2, 0, spec.width),
			[Solids.cylinder(spec.boreDiameter / 2, -0.1, spec.width + 0.1)]);

	function get_boreDiameter():Float return spec.boreDiameter;
	function get_width():Float return spec.width;
}
