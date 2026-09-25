package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;

/** Involute rack: the straight-flank limit of a spur gear's tooth profile (infinite pitch
 * radius), for meshing with a `SpurGear` of the same module and pressure angle.
 *
 * CAD frame: pitch line at y=0, teeth pointing toward +Y, length along local +Z from z=0 to
 * z=length (a tooth gap centred at each end), extruded along local +X for `faceWidth`.
 */
class Rack {
	public final designation:String;
	public final description:String;
	public final moduleSize:Float;
	public final teethCount:Int;
	public final faceWidth:Float;
	public final pressureAngle:Float;
	public final length:Float;
	public final barHeight:Float;

	public function new(moduleSize:Float, teethCount:Int, faceWidth:Float,
			pressureAngle:Float = SpurGear.STANDARD_PRESSURE_ANGLE, barHeight:Float = -1) {
		if (!(moduleSize > 0)) throw "Rack needs a positive module";
		if (teethCount < 1) throw "Rack needs at least one tooth";
		if (!(faceWidth > 0)) throw "Rack needs a positive face width";
		if (!(pressureAngle > 0) || !(pressureAngle < Math.PI / 2))
			throw "Rack pressure angle must be between 0 and 90 degrees";
		this.moduleSize = moduleSize;
		this.teethCount = teethCount;
		this.faceWidth = faceWidth;
		this.pressureAngle = pressureAngle;
		this.barHeight = barHeight >= 0 ? barHeight : 1.5 * moduleSize;
		length = teethCount * Math.PI * moduleSize;
		designation = 'RACK-M${moduleSize}-${teethCount}T';
		description = 'Rack module ${moduleSize}, ${teethCount} teeth';
	}

	public function geometry():Part {
		var a = moduleSize, b = 1.25 * moduleSize, p = Math.PI * moduleSize, halfThickness = p / 4;
		var bottom = -b - barHeight;
		function halfWidth(y:Float):Float
			return halfThickness - y * Math.tan(pressureAngle);
		var points:Array<Vector> = [new Vector(bottom, 0)];
		for (k in 0...teethCount) {
			var center = (k + 0.5) * p;
			points.push(new Vector(-b, center - halfWidth(-b)));
			points.push(new Vector(a, center - halfWidth(a)));
			points.push(new Vector(a, center + halfWidth(a)));
			points.push(new Vector(-b, center + halfWidth(-b)));
		}
		points.push(new Vector(bottom, length));
		var sketch = Sketch.polygon(points, Plane.YZ());
		try {
			var result = sketch.extrude(faceWidth);
			sketch.close();
			return result;
		} catch (error:Dynamic) {
			sketch.close();
			throw error;
		}
	}
}
