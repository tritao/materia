package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Assumed 4 N m spring-applied shaft brake. The hub and disc rotate with the motor;
 * front, shell and back sectors attach to its mount. Separate sectors keep their convex
 * collision hulls clear of the shaft and released disc. Dimensions are mm, delays seconds.
 * These envelopes, steel mass allocation and ratings are reference values, not vendor data. */
class ShaftBrakePart extends MachineComponent implements MotorBrakeSpec {
	public final sectors:Int;
	public final bore:Float;
	public final role:String;
	public final sector:Int;
	public final depth:Float = 10;
	public final discThickness:Float = 2;
	public final releaseClearance:Float = 0.1;
	public final hubRadius:Float;
	public final discRadius:Float;
	public final bodyRadius:Float;
	static var recipe:Null<ComponentType>;

	public function new(bore:Float, role:String, sector:Int = 0) {
		if (!(bore > 0) || !Math.isFinite(bore) ||
			["hub", "disc", "front", "shell", "back"].indexOf(role) < 0 ||
			sector < 0 || sector >= sectorCount(bore, role) || ((role == "hub" || role == "disc") && sector != 0))
			throw "Shaft brake needs a positive bore and a valid part role/sector";
		super('SHAFT-BRAKE-4NM-$bore-$role-$sector', "Assumed spring-applied shaft brake " + role, "steel");
		this.bore = bore; this.role = role; this.sector = sector;
		sectors = sectorCount(bore, role);
		hubRadius = bore / 2 + 2;
		discRadius = bore / 2 + 8;
		bodyRadius = bore / 2 + 16;
		var section = sectionGeometry();
		addConnector("mount", Mount, Solids.axial(0, 0, section.start));
		addConnector("axis", Axis, Solids.axial(0, 0, (section.start + section.end) / 2));
		addConnector("end", Face, Solids.axial(0, 0, section.end));
	}

	/** Fewest sectors whose convex inner chord clears the actual rotating radius. */
	public static function sectorCount(bore:Float, role:String):Int {
		if (role == "hub" || role == "disc") return 1;
		var rotor = bore / 2 + (role == "shell" ? 8 : 2);
		var inner = rotor + 1;
		var count = 3;
		while (inner * Math.cos(Math.PI / count) <= rotor) {
			count++;
			if (count > 64) throw "Shaft brake bore exceeds the convex sector budget";
		}
		return count;
	}

	public function holdingTorque():Float return 4;
	public function releaseDelaySeconds():Float return 0.05;
	public function applicationDelaySeconds():Float return 0.03;
	override public function motorBrakeSpec():Null<MotorBrakeSpec>
		return role == "shell" && sector == 0 ? this : null;

	public function discStart():Float return (depth - discThickness) / 2;
	public function discEnd():Float return (depth + discThickness) / 2;
	override public function hasGeometry():Bool return true;
	function sectionGeometry():{inner:Float, outer:Float, start:Float, end:Float} {
		return switch role {
			case "hub": {inner: bore / 2, outer: hubRadius, start: releaseClearance, end: depth - releaseClearance};
			case "disc": {inner: hubRadius, outer: discRadius, start: discStart(), end: discEnd()};
			case "front": {inner: hubRadius + 1, outer: bodyRadius, start: 0, end: discStart() - releaseClearance};
			case "shell": {inner: discRadius + 1, outer: bodyRadius,
				start: discStart() - releaseClearance, end: discEnd() + releaseClearance};
			case "back": {inner: hubRadius + 1, outer: bodyRadius,
				start: discEnd() + releaseClearance, end: depth};
			case _: throw "Unknown shaft brake part role";
		};
	}
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var section = sectionGeometry();
		var inner = section.inner, outer = section.outer, start = section.start, end = section.end;
		return Solids.building([], parts -> {
			var ring = Solids.cut(Solids.named(Part.cylinderSpan(outer, start, end), role),
				[Solids.named(Part.cylinderSpan(inner, start - 0.1, end + 0.1), "bore")]);
			parts.push(ring);
			if (role == "hub" || role == "disc") return ring;
			var first = sector * 2 * Math.PI / sectors, last = (sector + 1) * 2 * Math.PI / sectors;
			var radius = outer * 2;
			var wedge = Part.prism([new Vector(0, 0),
				new Vector(radius * Math.cos(first), radius * Math.sin(first)),
				new Vector(radius * Math.cos(last), radius * Math.sin(last))], start - 0.1, end + 0.1);
			parts.push(wedge);
			var result = ring.intersect(wedge);
			ring.close(); wedge.close();
			return Solids.named(result, "sector");
		});
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.motion.shaft-brake-part", [
			ComponentRecipeSupport.length("bore", 19),
			ComponentRecipeSupport.choice("role", ["hub", "disc", "front", "shell", "back"], "shell"),
			ComponentRecipeSupport.count("sector", 0)
		], values -> new ShaftBrakePart(values.number("bore"), values.token("role"), values.integer("sector")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function values():ComponentValues return new ComponentValues().setNumber("bore", bore)
		.setToken("role", role).setInteger("sector", sector).setToken("material", materialSpec());
}
