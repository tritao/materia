package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Dimension;
import machinekit.component.Solids;

/** L-section angle iron, extruded along local +Z from z=0 to z=`length`. The outer corner sits
 * at the local origin, with legs running along +X (`legA`) and +Y (`legB`).
 */
class Angle implements StructuralProfile {
	public final designation:String;
	public final description:String;
	public final legA:Float;
	public final legB:Float;
	public final thickness:Float;

	public function new(legA:Float, legB:Float, thickness:Float) {
		if (!(legA > 0) || !(legB > 0) || !(thickness > 0)) throw "Angle needs positive legs and thickness";
		if (!(thickness < legA) || !(thickness < legB)) throw "Angle thickness must be less than either leg";
		this.legA = legA;
		this.legB = legB;
		this.thickness = thickness;
		var size = '${Dimension.format(legA)}x${Dimension.format(legB)}x${Dimension.format(thickness)}';
		designation = 'ANGLE-$size';
		description = 'Angle $size';
	}

	public function geometry(length:Float):Part {
		if (!(length > 0)) throw "Angle needs a positive length";
		return Solids.prism([
			new Vector(0, 0), new Vector(legA, 0), new Vector(legA, thickness),
			new Vector(thickness, thickness), new Vector(thickness, legB), new Vector(0, legB),
		], 0, length);
	}

	public function profileDesignation():String return designation;
	public function profileDescription():String return description;
}
