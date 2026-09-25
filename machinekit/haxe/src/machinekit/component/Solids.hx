package machinekit.component;

import cadkit.modeling.Axis;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import materia.project.AssemblyFrames;
import materia.project.AssemblyRecord.AssemblyFrame;

/** Small construction helpers shared by generators. Inputs passed as `parts` are consumed. */
class Solids {
	/** Revolves a closed (radius, z) half-section about +Z. */
	public static function revolve(section:Array<{r:Float, z:Float}>):Part {
		var sketch = Sketch.polygon([for (point in section) new Vector(point.r, point.z)], Plane.XZ());
		try {
			var result = sketch.revolve(Axis.Z());
			sketch.close();
			return result;
		} catch (error:Dynamic) {
			sketch.close();
			throw error;
		}
	}

	/** Cylinder about a +Z axis through (x, y), spanning z0..z1. */
	public static function cylinder(radius:Float, z0:Float, z1:Float, x:Float = 0, y:Float = 0):Part {
		if (!(radius > 0) || !(z1 > z0)) throw "Cylinder needs a positive radius and length";
		var base = Part.cylinder(radius, z1 - z0);
		try {
			var result = base.translated(new Vector(x, y, z0));
			base.close();
			return result;
		} catch (error:Dynamic) {
			base.close();
			throw error;
		}
	}

	/** Extrudes a local XY polygon from z0 to z1. */
	public static function prism(points:Array<Vector>, z0:Float, z1:Float):Part {
		var sketch = Sketch.polygon(points, Plane.XY().offset(z0));
		try {
			var result = sketch.extrude(z1 - z0);
			sketch.close();
			return result;
		} catch (error:Dynamic) {
			sketch.close();
			throw error;
		}
	}

	/** Regular polygon with a flat-to-flat width, one flat facing +X. */
	public static function regularPolygon(sides:Int, acrossFlats:Float):Array<Vector> {
		var radius = acrossFlats / (2 * Math.cos(Math.PI / sides));
		return [for (i in 0...sides) {
			var angle = Math.PI / sides + 2 * Math.PI * i / sides;
			new Vector(radius * Math.cos(angle), radius * Math.sin(angle));
		}];
	}

	/** Fuses all parts, closing them. */
	public static function union(parts:Array<Part>):Part {
		if (parts.length == 0) throw "Union needs at least one part";
		var result = parts[0];
		try {
			for (i in 1...parts.length) {
				var next = result.combine(parts[i]);
				result.close();
				parts[i].close();
				parts[i] = null;
				result = next;
			}
			return result;
		} catch (error:Dynamic) {
			result.close();
			for (i in 1...parts.length) if (parts[i] != null) parts[i].close();
			throw error;
		}
	}

	/** Subtracts every tool from the base, closing the base and tools. */
	public static function cut(base:Part, tools:Array<Part>):Part {
		if (tools.length == 0) return base;
		return subtractClosing(base, union(tools));
	}

	static function subtractClosing(base:Part, tool:Part):Part {
		try {
			var result = base.subtract(tool);
			base.close();
			tool.close();
			return result;
		} catch (error:Dynamic) {
			base.close();
			tool.close();
			throw error;
		}
	}

	/** Frame at a point whose +Y (joint axis) points along +Z. */
	public static function axial(x:Float, y:Float, z:Float):AssemblyFrame
		return AssemblyFrames.alongY(x, y, z, 0, 0, 1);
}
