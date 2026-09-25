package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Solids;

/** Rectangular (or, with `width == height`, square) hollow structural tube, extruded along
 * local +Z from z=0 to z=`length`, cross-section centred on the axis.
 */
class RectTube implements StructuralProfile {
	public final designation:String;
	public final description:String;
	public final width:Float;
	public final height:Float;
	public final wall:Float;

	public function new(width:Float, height:Float, wall:Float) {
		if (!(width > 0) || !(height > 0) || !(wall > 0)) throw "Rect tube needs a positive width, height, and wall";
		if (!(2 * wall < Math.min(width, height))) throw "Rect tube wall is too thick for its section";
		this.width = width;
		this.height = height;
		this.wall = wall;
		designation = 'RECT-${width}x${height}x${wall}';
		description = 'Rectangular tube ${width}x${height}x${wall}';
	}

	public function geometry(length:Float):Part {
		if (!(length > 0)) throw "Rect tube needs a positive length";
		var outer = Solids.prism(rectangle(width, height), 0, length);
		var inner = Solids.prism(rectangle(width - 2 * wall, height - 2 * wall), -0.1, length + 0.1);
		return Solids.cut(outer, [inner]);
	}

	public function profileDesignation():String return designation;
	public function profileDescription():String return description;

	static function rectangle(width:Float, height:Float):Array<Vector>
		return [new Vector(-width / 2, -height / 2), new Vector(width / 2, -height / 2),
			new Vector(width / 2, height / 2), new Vector(-width / 2, height / 2)];
}
