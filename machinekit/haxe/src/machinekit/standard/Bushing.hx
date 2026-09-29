package machinekit.standard;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentValue.*;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Dimension;
import materia.project.MaterialLibrary;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Plain sleeve bushing, sized by bore (shaft) diameter; outer diameter and length are derived
 * proportionally unless given explicitly.
 * CAD frame: axis along +Z, front face at z=0, back face at z=length, matching
 * `DeepGrooveBearing`. Connectors: `front`, `back` (faces) and `axis` (mid-length), all with +Y
 * along +Z.
 */
class Bushing extends MachineComponent {
	public final boreDiameter:Float;
	public final outerDiameter:Float;
	public final length:Float;

	public function new(boreDiameter:Float, ?outerDiameter:Float, ?length:Float) {
		if (!(boreDiameter > 0)) throw "Bushing needs a positive bore diameter";
		var wall = Math.max(1.5, boreDiameter * 0.15);
		var od = outerDiameter == null ? boreDiameter + 2 * wall : outerDiameter;
		var len = length == null ? boreDiameter * 1.5 : length;
		if (!(od > boreDiameter)) throw "Bushing outer diameter must be larger than the bore";
		if (!(len > 0)) throw "Bushing needs a positive length";
		var size = '${Dimension.format(boreDiameter)}x${Dimension.format(od)}x${Dimension.format(len)}';
		super('BUSHING-$size', 'Plain bushing $size', "bronze");
		this.boreDiameter = boreDiameter;
		this.outerDiameter = od;
		this.length = len;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, this.length / 2));
		addConnector("back", Face, Solids.axial(0, 0, this.length));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.cut(Part.cylinderSpan(outerDiameter / 2, 0, length),
			[Part.cylinderSpan(boreDiameter / 2, -0.1, length + 0.1)]);

	private static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null)
			recipeTypeCache = new ComponentType("machinekit.standard.bushing",
			[ComponentRecipeSupport.length("boreDiameter", 8), ComponentRecipeSupport.length("outerDiameter", 12), ComponentRecipeSupport.length("length", 10)],
			v -> new Bushing(v.number("boreDiameter"), v.number("outerDiameter"), v.number("length")));
		return recipeTypeCache;
	}

	override public function componentType():Null<ComponentType> return Std.isExactType(this, Bushing) ? recipeType() : null;

	override public function values():ComponentValues {
		return new ComponentValues().set("boreDiameter", Number(this.boreDiameter))
				.set("outerDiameter", Number(this.outerDiameter)).set("length", Number(this.length))
				.setToken("material", materialSpec());
	}

}
