package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Dimension;
import machinekit.component.Solids;

/** Solid rectangular bar, extruded along local +Z from z=0 to z=`length`, cross-section centred
 * on the axis.
 */
class FlatBar implements StructuralProfile {
	public final designation:String;
	public final description:String;
	public final width:Float;
	public final thickness:Float;

	public function new(width:Float, thickness:Float) {
		if (!(width > 0) || !(thickness > 0)) throw "Flat bar needs a positive width and thickness";
		this.width = width;
		this.thickness = thickness;
		var size = '${Dimension.format(width)}x${Dimension.format(thickness)}';
		designation = 'FLAT-$size';
		description = 'Flat bar $size';
	}

	public function geometry(length:Float):Part {
		if (!(length > 0)) throw "Flat bar needs a positive length";
		return Solids.prism([
			new Vector(-width / 2, -thickness / 2), new Vector(width / 2, -thickness / 2),
			new Vector(width / 2, thickness / 2), new Vector(-width / 2, thickness / 2),
		], 0, length);
	}

	public function profileDesignation():String return designation;
	public function profileDescription():String return description;
}
