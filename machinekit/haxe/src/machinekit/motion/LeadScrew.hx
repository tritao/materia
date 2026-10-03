package machinekit.motion;

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

/** Nominal cylindrical envelope of a lead screw. The thread is semantic and shared with its nut;
 * the flank profile, reliefs and machined ends are not generated.
 */
class LeadScrew extends MachineComponent {
	/** Resolve the coupling from these parts. */
	public static function relation(screw:LeadScrew, nut:LeadScrewNut, alignment:Float):machinekit.transmission.TransmissionRelation {
		if (screw.thread.designation != nut.thread.designation) throw new machinekit.transmission.TransmissionDesignError("Lead screw and nut threads differ; update the nut to match the screw");
		var result = new machinekit.transmission.TransmissionRelation(2 * Math.PI * alignment / screw.thread.signedLead(),
			nut.efficiency(), null, nut.backlash(), nut.drag());
		result.setBasis("efficiency", machinekit.transmission.ValueBasis.Assumed, "nut friction");
		result.setBasis("backlash", machinekit.transmission.ValueBasis.Assumed, "nut backlash");
		result.setBasis("drag", machinekit.transmission.ValueBasis.Assumed, "nut drag");
		return result;
	}

	public final thread:LeadScrewThread;
	public final totalLength:Float;

	public function new(thread:LeadScrewThread, length:Float) {
		if (thread == null) throw "Lead screw needs a thread specification";
		if (!(length > 0) || !Math.isFinite(length)) throw "Lead screw needs a positive length";
		super('LEADSCREW-${thread.designation}-L${Dimension.format(length)}',
			'Lead screw ${thread.designation}, ${Dimension.format(length)} mm long', "steel");
		this.thread = thread;
		totalLength = length;
		addConnector("input", Axis, Solids.axial(0, 0, 0));
		addConnector("output", Axis, Solids.axial(0, 0, length));
	}

	/** Young's modulus of steel, Pa. */
	public static inline var YOUNGS_MODULUS:Float = 200e9;
	/** Density of steel, kg/m³. */
	public static inline var DENSITY:Float = 7850;

	/**
	 * Eigenvalue (lambda) of a uniform beam's first bending mode for the two ends' supports, from
	 * the standard Euler-Bernoulli beam results: fixed-free 1.875, simple-simple pi, fixed-simple
	 * 3.927, fixed-fixed 4.730. Two free ends leave nothing to hold the screw.
	 */
	public static function bendingEigenvalue(near:ScrewSupport, far:ScrewSupport):Float {
		var fixed = (near == Fixed ? 1 : 0) + (far == Fixed ? 1 : 0);
		var simple = (near == Simple ? 1 : 0) + (far == Simple ? 1 : 0);
		if (fixed == 2) return 4.730040745;
		if (fixed == 1 && simple == 1) return 3.926602312;
		if (fixed == 1) return 1.875104069;
		if (simple == 2) return Math.PI;
		throw "A lead screw needs at least one held end";
	}

	/**
	 * Speed, in rad/s, below which the screw is kept from whipping: `margin` of its first bending
	 * speed, omega = lambda^2 / L^2 * sqrt(E I / (rho A)) = lambda^2 d_r / (4 L^2) * sqrt(E / rho)
	 * for a solid round section of root diameter d_r. `unsupported` is the longest stretch between
	 * supports in mm (the whole screw by default; a nut rides along it, so the longest stretch is
	 * the one that counts). The usual margin is 80% of the critical speed. The root diameter
	 * ignores the thread's flanks, which add stiffness: this is conservative.
	 */
	public function criticalSpeed(near:ScrewSupport, far:ScrewSupport, ?unsupported:Float, margin:Float = 0.8):Float {
		var length = (unsupported == null ? totalLength : unsupported) / 1000;
		if (!(length > 0) || !(margin > 0 && margin <= 1)) throw "Critical speed needs a positive length and a margin in (0, 1]";
		var lambda = bendingEigenvalue(near, far);
		var root = thread.rootDiameter() / 1000;
		return margin * lambda * lambda * root / (4 * length * length) * Math.sqrt(YOUNGS_MODULUS / DENSITY);
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.cylinderSpan(thread.screwDiameter / 2, 0, totalLength);

	private static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null)
			recipeTypeCache = new ComponentType("machinekit.motion.lead-screw",
			ComponentRecipeSupport.threadParameters().concat([ComponentRecipeSupport.length("length", 100)]),
			v -> new LeadScrew(ComponentRecipeSupport.thread(v), v.number("length")));
		return recipeTypeCache;
	}

	override public function componentType():Null<ComponentType> return Std.isExactType(this, LeadScrew) ? recipeType() : null;

	override public function values():ComponentValues {
		return ComponentRecipeSupport.threadValues(this.thread).setNumber("length", this.totalLength)
			.setToken("material", materialSpec());
	}

}
