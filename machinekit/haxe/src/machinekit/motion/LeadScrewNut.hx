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
import machinekit.standard.ClearanceFit;
import machinekit.standard.SocketHeadCapScrew;

/** Explicit flange dimensions, mm, for catalogue nuts with independently specified geometry. */
typedef ScrewNutDimensions = {
	var bodyDiameter:Float;
	var bodyLength:Float;
	var flangeDiameter:Float;
	var flangeThickness:Float;
	var boltCircleDiameter:Float;
	var mountScrew:String;
}

/** ACME/trapezoidal lead screw nut: a flanged block with a bore matching the screw diameter and
 * a mounting bolt pattern on the flange face, for driving a carriage. The thread itself is
 * semantic (`LeadScrewThread` family, diameter, pitch, starts and hand), not modelled.
 * `travelPerRevolution()`/`rotationFor()` convert between screw rotation and nut travel; for a
 * right-hand thread, positive rotation about +Z moves the nut toward -Z.
 * CAD frame: axis along +Z, body from z=0 to z=bodyLength, flange from there to
 * z=bodyLength+flangeThickness. Connectors: `bore` (axis, mid-body) and `mount1`..`mountN` (on
 * the flange face), all with +Y along +Z.
 */
class LeadScrewNut extends MachineComponent {
	public final thread:LeadScrewThread;
	public final screwDiameter:Float;
	public final lead:Float;
	public final bodyDiameter:Float;
	public final bodyLength:Float;
	public final flangeDiameter:Float;
	public final flangeThickness:Float;
	public final boltCircleDiameter:Float;
	public final boltCount:Int;
	public final mountScrew:String;
	/** A barrel nut has no flange and is retained by its bracket. */
	public final flanged:Bool;
	public final kind:LeadScrewNutKind;
	final explicitDimensions:Bool;

	public function new(thread:LeadScrewThread, boltCount:Int = 4, flanged:Bool = true, kind:LeadScrewNutKind = AntiBacklash, ?dimensions:ScrewNutDimensions) {
		if (kind == null) throw "Lead screw nut needs a kind";
		if (thread == null) throw "Lead screw nut needs a thread specification";
		var screwDiameter = thread.screwDiameter;
		var lead = thread.lead;
		if (flanged && boltCount < 3) throw "Lead screw nut needs at least 3 mounting bolts";
		// Proportioned on the common T8 nut (10.2 mm body, 16 mm bolt circle, 22 mm flange, M3).
		// A barrel nut fits a narrow carriage gap; a flanged body follows the common T8 proportions.
		var bodyDia = dimensions == null ? screwDiameter * (flanged ? 1.3 : 1.2) : dimensions.bodyDiameter;
		var bodyLen = dimensions == null ? screwDiameter * 2 : dimensions.bodyLength;
		var flangeThick = dimensions == null ? Math.max(3, screwDiameter * 0.3) : dimensions.flangeThickness;
		var mountScrewSize = dimensions == null ? (screwDiameter <= 8 ? "M3" : screwDiameter <= 12 ? "M4" : "M5") : dimensions.mountScrew;
		var screw = SocketHeadCapScrew.catalog().get(mountScrewSize);
		// Holes keep 1 mm of material to the body and the screw heads 1 mm to the flange rim.
		var boltRadius = dimensions == null ? bodyDia / 2 + screw.clearanceMedium / 2 + 1 : dimensions.boltCircleDiameter / 2;
		if (flanged && 2 * boltRadius * Math.sin(Math.PI / boltCount) < screw.clearanceMedium + 1)
			throw "Lead screw nut bolt count leaves too little material between mounting holes";
		var flangeDia = dimensions == null ? 2 * Math.max(screwDiameter * 1.5, boltRadius + screw.headDiameter / 2 + 1) : dimensions.flangeDiameter;
		if (!(bodyDia > screwDiameter) || !(bodyLen > 0) || !(flangeThick > 0) ||
			(flanged && (!(boltRadius > bodyDia / 2 + screw.clearanceMedium / 2) ||
			!(flangeDia / 2 > boltRadius + screw.clearanceMedium / 2))))
			throw "Nut dimensions must leave material around its bore and mounting holes";
		var diameterText = Dimension.format(screwDiameter), leadText = Dimension.format(lead);
		super('LEADNUT-${thread.designation}${flanged ? "" : "-BARREL"}${kind == AntiBacklash ? "" : "-" + Std.string(kind)}', 'Lead screw nut, ${thread.designation}, $leadText mm lead', "bronze");
		this.thread = thread;
		this.screwDiameter = screwDiameter;
		this.lead = lead;
		bodyDiameter = bodyDia;
		bodyLength = bodyLen;
		flangeDiameter = flanged ? flangeDia : bodyDia;
		flangeThickness = flanged ? flangeThick : 0;
		boltCircleDiameter = flanged ? 2 * boltRadius : 0;
		this.boltCount = flanged ? boltCount : 0;
		this.flanged = flanged;
		this.kind = kind;
		explicitDimensions = dimensions != null;
		mountScrew = mountScrewSize;
		addConnector("bore", Axis, Solids.axial(0, 0, bodyLength / 2));
		addConnector("mountFace", Face, Solids.axial(0, 0, bodyLength + flangeThickness));
		var i = 1;
		for (point in boltPattern())
			addConnector('mount${i++}', Mount, Solids.axial(point.x, point.y, bodyLength + flangeThickness));
	}

	/** Assumed reversal clearance, mm: plain bronze 0.15, preloaded 0.05, ball nut 0.01. */
	public function backlash():Float return switch kind {
		case PlainBronze: 0.15;
		case AntiBacklash: 0.05;
		case BallNut: 0.01;
		case PreloadedBallNut: 0;
	};

	/** Assumed running torque, N m, including nut preload and support bearing drag. */
	public function drag():Float return switch kind {
		case PlainBronze: 0.01;
		case AntiBacklash: 0.02;
		case BallNut: 0.005;
		case PreloadedBallNut: 0.02;
	};

	/** Sliding nuts use a greased steel/bronze friction of 0.1; ball nuts assume 90% efficiency. */
	public function efficiency():Float return (kind == BallNut || kind == PreloadedBallNut) ? LeadScrewThread.BALL_EFFICIENCY : thread.efficiency(0.1);

	static function parseKind(value:String):LeadScrewNutKind return switch value {
		case "PlainBronze": PlainBronze;
		case "AntiBacklash": AntiBacklash;
		case "BallNut": BallNut;
		case "PreloadedBallNut": PreloadedBallNut;
		default: throw 'Unknown lead screw nut kind "$value"';
	};

	/** Mount bolt centres on the flange face, counter-clockwise from angle 0. */
	public function boltPattern():Array<{x:Float, y:Float}> {
		var r = boltCircleDiameter / 2;
		return [for (i in 0...boltCount) {
			var angle = 2 * Math.PI * i / boltCount;
			{x: r * Math.cos(angle), y: r * Math.sin(angle)};
		}];
	}

	/** Signed axial travel for one positive screw revolution. */
	public function travelPerRevolution():Float
		return thread.signedLead();

	/** Screw rotation, in radians, needed to travel `distance`. */
	public function rotationFor(distance:Float):Float
		return distance / thread.signedLead() * 2 * Math.PI;

	/** Screw that fits the nut's mounting holes. */
	public function mountScrewPart(length:Float):SocketHeadCapScrew
		return SocketHeadCapScrew.metric(mountScrew, length);

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.named(Part.cylinderSpan(bodyDiameter / 2, 0, bodyLength), "body");
		if (!flanged) return Solids.cut(body, [Part.cylinderSpan(screwDiameter / 2, -0.1, bodyLength + 0.1)]);
		var flange = Solids.named(Part.cylinderSpan(flangeDiameter / 2, bodyLength, bodyLength + flangeThickness), "flange");
		var solidPart = Solids.union([body, flange]);
		var boreTool = Solids.named(Part.cylinderSpan(screwDiameter / 2, -0.1, bodyLength + flangeThickness + 0.1), "bore");
		if (detail == Envelope) return Solids.cut(solidPart, [boreTool]);
		var screw = mountScrewPart(10);
		var tools = [boreTool];
		var bolt = 1;
		for (point in boltPattern())
			tools.push(Solids.named(Part.cylinderSpan(screw.clearanceDiameter(Medium) / 2, bodyLength - 0.1,
				bodyLength + flangeThickness + 0.1, point.x, point.y), "bolt" + bolt++));
		return Solids.cut(solidPart, tools);
	}

	private static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null)
			recipeTypeCache = new ComponentType("machinekit.motion.lead-screw-nut",
			ComponentRecipeSupport.threadParameters().concat([ComponentRecipeSupport.count("boltCount", 4), ComponentRecipeSupport.flag("flanged", true),
				ComponentRecipeSupport.choice("kind", ["PlainBronze", "AntiBacklash", "BallNut", "PreloadedBallNut"], "AntiBacklash")]),
			v -> new LeadScrewNut(ComponentRecipeSupport.thread(v), v.integer("boltCount"), v.boolean("flanged"), parseKind(v.token("kind"))));
		return recipeTypeCache;
	}

	override public function componentType():Null<ComponentType> return Std.isExactType(this, LeadScrewNut) && !explicitDimensions ? recipeType() : null;

	override public function values():ComponentValues {
		return ComponentRecipeSupport.threadValues(this.thread).setInteger("boltCount", this.boltCount).setBoolean("flanged", flanged).setToken("kind", Std.string(kind))
			.setToken("material", materialSpec());
	}

}
