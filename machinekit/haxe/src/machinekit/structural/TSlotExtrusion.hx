package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Solids;

/** Square T-slot aluminium extrusion profile (2020/4040-style), with a T-slot channel centred
 * on each of its four faces and a central bore, extruded along local +Z from z=0 to z=`length`.
 * Slot dimensions are proportional to `size`, not from a vendor's literal table, sized with
 * margin so adjacent faces' slot heads and the bore never intersect and sever the corner posts.
 */
class TSlotExtrusion implements StructuralProfile {
	public final designation:String;
	public final description:String;
	public final size:Float;
	public final slotWidth:Float;
	public final throatDepth:Float;
	public final tWidth:Float;
	public final headDepth:Float;
	public final boreDiameter:Float;

	public function new(size:Float) {
		if (!(size > 0)) throw "T-slot extrusion needs a positive size";
		this.size = size;
		slotWidth = 0.3 * size;
		throatDepth = 0.15 * size;
		tWidth = 0.35 * size;
		headDepth = 0.1 * size;
		boreDiameter = 0.15 * size;
		if (!(throatDepth + headDepth < size / 2)) throw "T-slot extrusion size is too small for its slot proportions";
		if (!(tWidth < size)) throw "T-slot extrusion size is too small for its slot proportions";
		designation = 'TSLOT-${size}x${size}';
		description = 'T-slot extrusion ${size}x${size}';
	}

	public function profileDesignation():String
		return designation;

	public function profileDescription():String
		return description;

	public function geometry(length:Float):Part {
		if (!(length > 0)) throw "T-slot extrusion needs a positive length";
		var half = size / 2;
		var body = Solids.prism([
			new Vector(-half, -half), new Vector(half, -half), new Vector(half, half), new Vector(-half, half),
		], 0, length);
		var tools = [Solids.cylinder(boreDiameter / 2, -0.1, length + 0.1)];
		for (face in 0...4) tools.push(Solids.prism(slotPoints(face), 0, length));
		return Solids.cut(body, tools);
	}

	/** T-slot notch tool for one face, rotated 90 degrees `face` times from the +X face. */
	function slotPoints(face:Int):Array<Vector> {
		var half = size / 2, outer = half + 1;
		var points = [
			new Vector(outer, -slotWidth / 2), new Vector(half - throatDepth, -slotWidth / 2),
			new Vector(half - throatDepth, -tWidth / 2), new Vector(half - throatDepth - headDepth, -tWidth / 2),
			new Vector(half - throatDepth - headDepth, tWidth / 2), new Vector(half - throatDepth, tWidth / 2),
			new Vector(half - throatDepth, slotWidth / 2), new Vector(outer, slotWidth / 2),
		];
		return [for (p in points) rotate90(p, face)];
	}

	static function rotate90(p:Vector, times:Int):Vector {
		var x = p.x, y = p.y;
		for (i in 0...times) {
			var nx = -y, ny = x;
			x = nx;
			y = ny;
		}
		return new Vector(x, y);
	}
}
