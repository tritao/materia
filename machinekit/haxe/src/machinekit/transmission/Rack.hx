package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentParameter;
import machinekit.component.ComponentParameterType.*;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentValue.*;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Involute rack: the straight-flank limit of a spur gear's tooth profile (infinite pitch
 * radius), for meshing with a `SpurGear` of the same module and pressure angle.
 *
 * CAD frame: pitch line at y=0, teeth pointing toward +Y, length along local +Z from z=0 to
 * z=length (a tooth gap centred at each end, so the root line runs out to square end faces),
 * extruded along local +X for `faceWidth`.
 */
class Rack extends MachineComponent {
	/** Assumed power efficiency of a lubricated rack and pinion. */
	public static inline var DEFAULT_EFFICIENCY:Float = 0.95;

	/** Resolve the coupling from these parts. */
	public static function relation(pinion:SpurGear, rack:Null<Rack>, alignment:Float):TransmissionRelation {
		if (rack != null && (rack.moduleSize != pinion.moduleSize || rack.pressureAngle != pinion.pressureAngle))
			throw new machinekit.transmission.TransmissionDesignError("Rack and pinion module or pressure angle differ; update the rack to match the pinion");
		var result = new TransmissionRelation(alignment * 2 / pinion.pitchDiameter, DEFAULT_EFFICIENCY,
			null, pinion.backlash + (rack == null ? 0 : rack.backlash));
		result.setBasis("efficiency", ValueBasis.Assumed, "rack efficiency");
		return result;
	}

	public final moduleSize:Float;
	public final teethCount:Int;
	public final faceWidth:Float;
	public final pressureAngle:Float;
	public final length:Float;
	public final barHeight:Float;
	/** Tangential reversal clearance at the pitch line, mm. */
	public final backlash:Float;

	public function new(moduleSize:Float, teethCount:Int, faceWidth:Float,
			pressureAngle:Float = SpurGear.STANDARD_PRESSURE_ANGLE, ?barHeight:Float, backlash:Float = 0) {
		if (!Math.isFinite(backlash) || backlash < 0) throw "Rack backlash must be finite and non-negative";
		if (!(moduleSize > 0)) throw "Rack needs a positive module";
		if (teethCount < 1) throw "Rack needs at least one tooth";
		if (!(faceWidth > 0)) throw "Rack needs a positive face width";
		if (!SpurGear.validPressureAngle(pressureAngle)) throw "Rack pressure angle must be between 14.5 and 25 degrees";
		var resolvedBarHeight = barHeight == null ? 1.5 * moduleSize : barHeight;
		if (!Math.isFinite(resolvedBarHeight) || resolvedBarHeight < 0)
			throw "Rack bar height must be finite and non-negative";
		var length = teethCount * Math.PI * moduleSize;
		var moduleText = Dimension.format(moduleSize);
		var designation = 'RACK-M$moduleText-${teethCount}T${backlash == 0 ? "" : "-B" + Dimension.format(backlash)}';
		var description = 'Rack module $moduleText, ${teethCount} teeth';
		super(designation, description, "steel");
		this.moduleSize = moduleSize;
		this.teethCount = teethCount;
		this.faceWidth = faceWidth;
		this.pressureAngle = pressureAngle;
		this.barHeight = resolvedBarHeight;
		this.backlash = backlash;
		this.length = length;
		addConnector("axis", ConnectorRole.Axis, Solids.axial(faceWidth / 2, 0, length / 2));
		addConnector("pitch", ConnectorRole.Pitch, Solids.axial(faceWidth / 2, 0, 0));
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var a = moduleSize, b = 1.25 * moduleSize, p = Math.PI * moduleSize, halfThickness = p / 4;
		var bottom = -b - barHeight;
		function halfWidth(y:Float):Float
			return halfThickness - y * Math.tan(pressureAngle);
		// Square ends: the root line runs out to z=0 and z=length before the bar drops to `bottom`.
		var points:Array<Vector> = [new Vector(bottom, 0), new Vector(-b, 0)];
		for (k in 0...teethCount) {
			var center = (k + 0.5) * p;
			points.push(new Vector(-b, center - halfWidth(-b)));
			points.push(new Vector(a, center - halfWidth(a)));
			points.push(new Vector(a, center + halfWidth(a)));
			points.push(new Vector(-b, center + halfWidth(-b)));
		}
		points.push(new Vector(-b, length));
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

	private static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null)
			recipeTypeCache = new ComponentType("machinekit.transmission.involute-rack", [
				ComponentRecipeSupport.length("moduleSize", 2), ComponentRecipeSupport.count("teethCount", 10),
				ComponentRecipeSupport.length("faceWidth", 10),
				new ComponentParameter("pressureAngle", Angle, Number(SpurGear.STANDARD_PRESSURE_ANGLE), "rad",
					SpurGear.MIN_PRESSURE_ANGLE, SpurGear.MAX_PRESSURE_ANGLE),
				ComponentRecipeSupport.length("barHeight", 3), ComponentRecipeSupport.length("backlash", 0)
			], v -> new Rack(v.number("moduleSize"), v.integer("teethCount"), v.number("faceWidth"),
				v.number("pressureAngle"), v.number("barHeight"), v.number("backlash")), true);
		return recipeTypeCache;
	}

	override public function componentType():Null<ComponentType> return Std.isExactType(this, Rack) ? recipeType() : null;

	override public function values():ComponentValues return new ComponentValues()
		.setNumber("moduleSize", moduleSize).setInteger("teethCount", teethCount).setNumber("faceWidth", faceWidth)
		.setNumber("pressureAngle", pressureAngle).setNumber("barHeight", barHeight).setNumber("backlash", backlash)
		.setToken("material", materialSpec());
}
