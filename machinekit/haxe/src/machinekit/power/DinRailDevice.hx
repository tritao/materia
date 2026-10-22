package machinekit.power;

import cadkit.modeling.Align;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Solids;

/**
 * The modular DIN-rail housing contactors, breakers and relays share, inside their envelope box: frame with x
 * along the rail, y from the back plate (-depth / 2) to the front (+depth / 2), z up the panel from 0 to height.
 * A full-height rear body carries the 35 mm rail-clip slot; a stepped terminal shoulder with screw recesses at
 * top and bottom; and a 45 mm front window with a label recess.
 */
class DinRailDevice {
	public static inline var RAIL_WIDTH:Float = 35;
	public static inline var RAIL_DEPTH:Float = 7.5;
	public static inline var FRONT_WINDOW:Float = 45;

	/** Seen as fitted: up the panel, its front window (+Y) towards the viewer. */
	public static function view():machinekit.component.ComponentView
		return new machinekit.component.ComponentView([0, 0, 1], [0, 1, 0]);

	public static function part(width:Float, depth:Float, height:Float):Part {
		if (!(width > 0) || !(depth > RAIL_DEPTH * 2) || !(height > RAIL_WIDTH))
			throw "DIN-rail device needs room for its rail slot";
		var back = -depth / 2, rear = depth * 0.55, shoulder = depth * 0.23;
		var shoulderLow = height * 0.12, shoulderHigh = height * 0.88;
		var window = Math.min(FRONT_WINDOW, height * 0.5), middle = height / 2;
		var terminals = Std.int(Math.max(1, Math.min(4, Math.floor(width / 9))));
		var pitch = width / terminals, screw = Math.min(pitch * 0.32, 3);
		return Solids.building([], owned -> {
			function box(dy:Float, dz:Float, y:Float, z:Float, tag:String):Part {
				var part = Solids.named(Part.box(width, dy, dz, Center, Min, Min).translated(new Vector(0, y, z)), tag);
				owned.push(part);
				return part;
			}
			var body = Solids.union([
				box(rear, height, back, 0, "rear-body"),
				box(shoulder, shoulderHigh - shoulderLow, back + rear, shoulderLow, "terminal-shoulder"),
				box(depth - rear - shoulder, window, back + rear + shoulder, middle - window / 2, "front")
			]);
			var tools:Array<Part> = [];
			function tool(part:Part):Void {
				owned.push(part);
				tools.push(part);
			}
			tool(Part.box(width + 1, RAIL_DEPTH, RAIL_WIDTH, Center, Min, Min).translated(new Vector(0, back - 0.1, middle - RAIL_WIDTH / 2)));
			var face = back + rear + shoulder;
			for (index in 0...terminals) {
				var x = -width / 2 + pitch * (index + 0.5);
				for (z in [shoulderLow + (middle - window / 2 - shoulderLow) * 0.5, shoulderHigh - (shoulderHigh - middle - window / 2) * 0.5])
					tool(Part.cylinderAlongY(screw, face - shoulder * 0.45, face + 0.1, x, z));
			}
			tool(Part.box(width * 0.7, 0.6, window * 0.45, Center, Min, Min).translated(new Vector(0, depth / 2 - 0.5, middle - window * 0.05)));
			return Solids.cut(body, tools);
		});
	}
}
