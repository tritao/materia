package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Dimension;
import machinekit.component.Solids;

/**
 * A pulley or idler a belt runs around, in the belt's plane: its pitch-circle centre and radius,
 * and the way the belt wraps it. `side` is +1 when the belt wraps it counter-clockwise (as seen
 * from the plane's +Z) and -1 when it wraps it clockwise, as it does an idler pressing on the
 * belt's outside.
 */
class BeltWrap {
	public final x:Float;
	public final y:Float;
	public final radius:Float;
	public final side:Int;

	public function new(x:Float, y:Float, radius:Float, side:Int = 1) {
		if (!Math.isFinite(x) || !Math.isFinite(y)) throw "Belt wrap needs a finite centre";
		if (!(radius > 0) || !Math.isFinite(radius)) throw "Belt wrap needs a positive pitch radius";
		if (side != 1 && side != -1) throw "Belt wrap side must be 1 (counter-clockwise) or -1 (clockwise)";
		this.x = x;
		this.y = y;
		this.radius = radius;
		this.side = side;
	}
}

/** A straight run of the belt between two wraps: where it leaves one and meets the next. */
class BeltStrand {
	public final from:Int;
	public final to:Int;
	public final startX:Float;
	public final startY:Float;
	public final endX:Float;
	public final endY:Float;
	/** Unit direction of travel, from the start to the end. */
	public final dx:Float;
	public final dy:Float;
	public final length:Float;

	public function new(from:Int, to:Int, startX:Float, startY:Float, endX:Float, endY:Float, dx:Float, dy:Float, length:Float) {
		this.from = from;
		this.to = to;
		this.startX = startX;
		this.startY = startY;
		this.endX = endX;
		this.endY = endY;
		this.dx = dx;
		this.dy = dy;
		this.length = length;
	}
}

/** A point on the belt's pitch line in its plane, and the direction the belt travels through it. */
class BeltPoint {
	public final x:Float;
	public final y:Float;
	public final dx:Float;
	public final dy:Float;

	public function new(x:Float, y:Float, dx:Float, dy:Float) {
		this.x = x;
		this.y = y;
		this.dx = dx;
		this.dy = dy;
	}
}

/** What a belt is made of when it runs round two pulleys of one belt family, which a recipe can rebuild. */
class BeltPair {
	public final driverTeeth:Int;
	public final idlerTeeth:Int;
	public final centreDistance:Float;

	public function new(driverTeeth:Int, idlerTeeth:Int, centreDistance:Float) {
		this.driverTeeth = driverTeeth;
		this.idlerTeeth = idlerTeeth;
		this.centreDistance = centreDistance;
	}
}

/**
 * A closed timing belt running around pulleys and idlers given in a plane. The loop is the
 * pitch line: straight strands tangent to the wraps and arcs on them, in the order the wraps are
 * given. Its length divided by the pitch is the tooth count a belt of that length has.
 *
 * The component's frame is the belt's plane, X and Y, with the belt `width` along +Z from 0.
 * Geometry is the belt's band, `thickness` thick and centred on the pitch line, as a straight
 * sided polygon; it has no teeth, since the band alone sweeps cheaply and the teeth would not
 * change any clearance.
 *
 * `loop` lists the wraps in the order the belt visits them. The belt must not cross itself.
 */
class TimingBelt extends MachineComponent {
	public final beltProfile:TimingBeltProfile;
	public final pitch:Float;
	public final width:Float;
	public final thickness:Float;
	/** Length of the pitch line, in millimetres. */
	public final length:Float;
	/** The whole number of teeth nearest the length: what a belt built for it has. */
	public final teeth:Int;
	final loop:Array<BeltWrap>;
	final strandList:Array<BeltStrand>;
	/** How far the belt wraps each wrap, in radians, going its way round. */
	final sweeps:Array<Float>;
	/** Where the belt meets each wrap, as an angle round its centre. */
	final arrivals:Array<Float>;
	final pair:Null<BeltPair>;

	public function new(beltProfile:TimingBeltProfile, width:Float, loop:Array<BeltWrap>, ?pair:BeltPair) {
		var pitch = TimingPulley.profileDimensions(beltProfile).pitch;
		if (!(width > 0) || !Math.isFinite(width)) throw "Timing belt needs a positive width";
		if (loop == null || loop.length < 2) throw "Timing belt needs at least two pulleys";
		var strands:Array<BeltStrand> = [];
		for (i in 0...loop.length) strands.push(tangent(loop, i, (i + 1) % loop.length));
		var arrivals:Array<Float> = [], sweeps:Array<Float> = [];
		var total = 0.0;
		for (strand in strands) total += strand.length;
		for (k in 0...loop.length) {
			var wrap = loop[k];
			var coming = strands[(k + loop.length - 1) % loop.length], going = strands[k];
			var arrival = Math.atan2(coming.endY - wrap.y, coming.endX - wrap.x);
			var departure = Math.atan2(going.startY - wrap.y, going.startX - wrap.x);
			var sweep = wrap.side * (departure - arrival);
			sweep = sweep - 2 * Math.PI * Math.floor(sweep / (2 * Math.PI));
			if (sweep > 2 * Math.PI - 1e-9) sweep = 0;
			arrivals.push(arrival);
			sweeps.push(sweep);
			total += wrap.radius * sweep;
		}
		var count = Math.round(total / pitch);
		if (count < 1) throw "Timing belt is shorter than one tooth";
		var name = TimingPulley.profileDimensions(beltProfile).name;
		var key = 0;
		for (wrap in loop) key = (key * 31 + Math.round(wrap.x * 100) * 7 + Math.round(wrap.y * 100) * 13 +
			Math.round(wrap.radius * 100) * 17 + wrap.side) % 1000003;
		if (key < 0) key = -key;
		super('BELT-$name-W${Dimension.format(width)}-${count}T' + (pair == null ? '-L$key' : ""),
			'$name timing belt, ${Dimension.format(pitch)} mm pitch, ${count} teeth, ${Dimension.format(width)} mm wide', "rubber",
			pair == null);
		this.beltProfile = beltProfile;
		this.pitch = pitch;
		this.width = width;
		thickness = bandThickness(beltProfile, pitch);
		length = total;
		teeth = count;
		this.loop = loop.copy();
		strandList = strands;
		this.sweeps = sweeps;
		this.arrivals = arrivals;
		this.pair = pair;
	}

	/** Thickness of a belt's band, tooth and backing together, in millimetres. */
	static function bandThickness(profile:TimingBeltProfile, pitch:Float):Float
		return switch (profile) {
			case GT2: 1.38;
			case HTD3M: 2.4;
			case HTD5M: 3.8;
			case HTD8M: 5.6;
			case HTD14M: 10.0;
			case T5: 2.2;
			case XL: 2.3;
			case Custom(_, _, _): 0.7 * pitch;
		};

	/** The oriented tangent from wrap `a` to wrap `b`, leaving `a` with it on the belt's inside. */
	static function tangent(loop:Array<BeltWrap>, a:Int, b:Int):BeltStrand {
		var first = loop[a], second = loop[b];
		var cx = second.x - first.x, cy = second.y - first.y;
		var distance = Math.sqrt(cx * cx + cy * cy);
		// A wrap's signed radius is positive when the belt turns counter-clockwise round it, which
		// keeps its centre on the left of the direction of travel.
		var rho1 = first.side * first.radius, rho2 = second.side * second.radius;
		var drho = rho2 - rho1;
		if (!(distance > Math.abs(drho))) throw 'Timing belt pulleys $a and $b are too close, or one lies inside the other';
		var ux = cx / distance, uy = cy / distance;
		var along = drho / distance;
		var across = Math.sqrt(1 - along * along);
		// The left normal n of travel satisfies n . (c2 - c1) = rho2 - rho1.
		var nx = along * ux - across * uy, ny = along * uy + across * ux;
		var dx = ny, dy = -nx;
		var sx = first.x - rho1 * nx, sy = first.y - rho1 * ny;
		var ex = second.x - rho2 * nx, ey = second.y - rho2 * ny;
		return new BeltStrand(a, b, sx, sy, ex, ey, dx, dy, Math.sqrt(distance * distance - drho * drho));
	}

	/**
	 * A belt round two pulleys of the same family, a driver at the origin and an idler
	 * `centreDistance` away along +X, both wrapped counter-clockwise.
	 */
	public static function twoPulley(beltProfile:TimingBeltProfile, driverTeeth:Int, idlerTeeth:Int, centreDistance:Float, width:Float):TimingBelt {
		var pitch = TimingPulley.profileDimensions(beltProfile).pitch;
		if (driverTeeth < 8 || idlerTeeth < 8) throw "Timing belt pulleys need at least 8 teeth";
		return new TimingBelt(beltProfile, width, [
			new BeltWrap(0, 0, pitch * driverTeeth / (2 * Math.PI)),
			new BeltWrap(centreDistance, 0, pitch * idlerTeeth / (2 * Math.PI))
		], switch beltProfile { case Custom(_, _, _): null; default: new BeltPair(driverTeeth, idlerTeeth, centreDistance); });
	}

	public function wraps():Array<BeltWrap> return loop.copy();

	/** Assumed power efficiency of a timing belt drive. */
	public static inline var DEFAULT_EFFICIENCY:Float = 0.97;
	/** Assumed pulley and idler bearing drag, N m. */
	public static inline var DEFAULT_DRAG:Float = 0.005;

	/** Resolve from the current loop geometry and width, never a saved stiffness. */
	public static function relation(belt:TimingBelt, pulley:TimingPulley, alignment:Float, idler:Bool = false):TransmissionRelation {
		if (belt.beltProfile != pulley.beltProfile) throw new machinekit.transmission.TransmissionDesignError("Belt and pulley profiles differ; update the belt to match the pulley");
		var result = new TransmissionRelation(alignment * 2 / pulley.pitchDiameter, idler ? 1.0 : DEFAULT_EFFICIENCY,
			null, null, DEFAULT_DRAG);
		if (!idler) {
			result.setBasis("stiffness", ValueBasis.Assumed, "belt stiffness");
			result.setBasis("efficiency", ValueBasis.Assumed, "belt efficiency");
		}
		result.setBasis("drag", ValueBasis.Assumed, "belt drag");
		return result;
	}

	/** A tooth-clearance allowance at the pitch line, mm, pending measured family data. */
	public static function toothClearance(profile:TimingBeltProfile):Float
		return TimingPulley.profileDimensions(profile).pitch * 0.01;

	/** Same keeps direction about the belt normal; assembly attachments check the joint axes. */
	public static function reduction(belt:TimingBelt, driver:TimingPulley, driven:TimingPulley,
			alignment:Float):TransmissionRelation {
		if (driver.beltProfile != belt.beltProfile || driven.beltProfile != belt.beltProfile)
			throw new TransmissionDesignError("Belt and reduction pulley profiles differ; update the pulleys");
		var ratio = alignment * driver.teeth / driven.teeth;
		var result = new TransmissionRelation(ratio, DEFAULT_EFFICIENCY, null,
			toothClearance(belt.beltProfile) / (driven.pitchDiameter / 2) / Math.abs(ratio), DEFAULT_DRAG);
		result.setBasis("efficiency", ValueBasis.Assumed, "belt efficiency");
		result.setBasis("drag", ValueBasis.Assumed, "belt drag");
		result.setBasis("stiffness", ValueBasis.Assumed, "belt stiffness with pretension");
		result.setBasis("backlash", ValueBasis.Assumed, "belt tooth clearance");
		return result;
	}

	/** Free elastic lengths between two engaged pulleys; other wraps turn freely. */
	public function freePaths(first:Int, second:Int):Array<Float> {
		if (first == second || first < 0 || second < 0 || first >= loop.length || second >= loop.length)
			throw new TransmissionDesignError("Belt reduction needs two distinct wraps");
		var a = 0.0, index = first;
		while (index != second) {
			a += strandList[index].length;
			index = (index + 1) % loop.length;
			if (index != second) a += loop[index].radius * sweeps[index];
		}
		var b = length - a - loop[first].radius * sweeps[first] - loop[second].radius * sweeps[second];
		if (!(a > 0 && b > 0)) throw new TransmissionDesignError("Belt has no free elastic paths between its pulleys");
		return [a, b];
	}

	/**
	 * Tensile stiffness of the belt's cords, EA in N, per millimetre of belt width. Assumption, not a
	 * datasheet value (makers rate breaking strength and working tension, not stiffness): a 6 mm
	 * GT2 belt with fibreglass cords breaks near 400 N at 2.5% elongation or more, which puts EA
	 * for 6 mm at no more than about 16 kN, so 2500 N per mm of width (15 kN for 6 mm) is on the soft
	 * side of that. Replace it with a measured value for a particular belt.
	 */
	public static function cordStiffnessPerMm(profile:TimingBeltProfile):Float
		return switch profile {
			case GT2: 2500.0;
			case _: throw "No belt stiffness is recorded for this profile";
		};

	/**
	 * Stiffness at a carriage clamped on strand `strand`, in N/mm of carriage travel, in the worst
	 * place on the strand. The belt is anchored at the driving pulley, so the carriage is held by
	 * two springs in parallel: the free length of the strand between it and the driver (at most the
	 * strand's length, with the carriage at the far end) and the rest of the loop the other way
	 * round. Each is EA over its length, EA from `cordStiffnessPerMm` times the width. Pretension,
	 * tooth compliance and the clamp are left out, so a real belt is a little softer.
	 */
	public function carriageStiffness(strand:Int = 0):Float {
		var strands = strandList;
		if (strand < 0 || strand >= strands.length) throw "Belt has no such strand";
		var along = strands[strand].length;
		if (!(along > 0 && along < length)) throw "Belt strand is as long as the loop";
		var cords = cordStiffnessPerMm(beltProfile) * width;
		return cords * (1 / along + 1 / (length - along));
	}

	/** The straight strands, strand `i` running from wrap `i` to the next. */
	public function strands():Array<BeltStrand> return strandList.copy();

	/** Length of the belt as `teeth` whole teeth have it, in millimetres. */
	public function toothLength():Float return teeth * pitch;

	/** The tooth count the length falls at, before it is rounded. */
	public function exactTeeth():Float return length / pitch;

	/**
	 * How far a belt of `teeth` whole teeth is longer (positive) or shorter than the loop's pitch
	 * line, in millimetres.
	 */
	public function slack():Float return toothLength() - length;

	/**
	 * How far to move one end pulley of a loop of two parallel runs, along them, to make the loop
	 * exactly `teeth` teeth long: the loop lengthens twice what the centres separate. Positive
	 * moves the pulleys apart.
	 */
	public function centreAdjustment():Float return slack() / 2;

	/**
	 * Which way wrap `wrap` turns about the plane's +Z (+1 counter-clockwise, -1 clockwise) while a
	 * carriage clamped to strand `strand` moves along (`travelX`, `travelY`) in the plane, which
	 * the strand must run along or against.
	 */
	public function rotation(wrap:Int, strand:Int, travelX:Float, travelY:Float):Int {
		var run = strandList[strand];
		var along = run.dx * travelX + run.dy * travelY;
		if (!(Math.abs(along) > 0.5 * Math.sqrt(travelX * travelX + travelY * travelY)))
			throw 'Timing belt strand $strand does not run along the carriage\'s travel';
		return loop[wrap].side * (along > 0 ? 1 : -1);
	}

	/** Arc phase held by a stationary driving pulley, halfway through its contact. */
	public function anchorPhase(wrap:Int):Float return arrivals[wrap] + loop[wrap].side * sweeps[wrap] / 2;

	/** Distance from the start of strand 0 to a material point held on a driving pulley. */
	public function anchorDistance(wrap:Int, phase:Float):Float {
		var progress = loop[wrap].side * (phase - arrivals[wrap]);
		progress -= 2 * Math.PI * Math.floor(progress / (2 * Math.PI));
		if (progress > sweeps[wrap] + 1e-6)
			throw new TransmissionDesignError("Belt leaves its held drive anchor; update the wrap attachments");
		var distance = 0.0;
		for (index in 0...loop.length) {
			distance += strandList[index].length;
			var next = (index + 1) % loop.length;
			if (next == wrap) return distance + loop[wrap].radius * progress;
			distance += loop[next].radius * sweeps[next];
		}
		throw "Belt has no such drive wrap";
	}

	/** Distance to the unique straight span containing a physical clamp. A tangent is a wrap. */
	public function clampDistanceAt(x:Float, y:Float):Float {
		var found = -1;
		for (index in 0...strandList.length) {
			var run = strandList[index];
			var along = (x - run.startX) * run.dx + (y - run.startY) * run.dy;
			var across = (x - run.startX) * run.dy - (y - run.startY) * run.dx;
			if (Math.abs(across) <= 1e-5 && along >= -1e-5 && along <= run.length + 1e-5) {
				if (along <= 1e-5 || along >= run.length - 1e-5)
					throw new TransmissionDesignError("Belt clamp lies on a wrap; move its connector inside a straight span");
				if (found >= 0) throw new TransmissionDesignError("Belt clamp lies on several spans; give it one straight attachment");
				found = index;
			}
		}
		if (found < 0) throw new TransmissionDesignError("Belt clamp lies on no straight span; update its connector");
		return clampDistance(found, x, y);
	}

	/** Distance along a specified straight span; callers can derive that span with `clampDistanceAt`. */
	public function clampDistance(strand:Int, x:Float, y:Float):Float {
		if (strand < 0 || strand >= strandList.length) throw "Belt clamp strand is out of range";
		var run = strandList[strand];
		var along = (x - run.startX) * run.dx + (y - run.startY) * run.dy;
		var across = (x - run.startX) * run.dy - (y - run.startY) * run.dx;
		if (along < -1e-5 || along > run.length + 1e-5 || Math.abs(across) > 1e-5)
			throw new TransmissionDesignError("Belt clamp leaves its attached strand; update its connector or the belt path");
		var distance = along;
		for (index in 0...strand) distance += strandList[index].length + loop[index + 1].radius * sweeps[index + 1];
		return distance;
	}

	/**
	 * The pitch line at `distance` along the belt, from where strand 0 leaves its wrap, wrapping
	 * round at the belt's length.
	 */
	public function pointAt(distance:Float):BeltPoint {
		var rest = distance - length * Math.floor(distance / length);
		for (i in 0...strandList.length) {
			var run = strandList[i];
			if (rest <= run.length)
				return new BeltPoint(run.startX + run.dx * rest, run.startY + run.dy * rest, run.dx, run.dy);
			rest -= run.length;
			var k = (i + 1) % loop.length, wrap = loop[k];
			var arc = wrap.radius * sweeps[k];
			if (rest <= arc) {
				var angle = arrivals[k] + wrap.side * rest / wrap.radius;
				return new BeltPoint(wrap.x + wrap.radius * Math.cos(angle), wrap.y + wrap.radius * Math.sin(angle),
					-wrap.side * Math.sin(angle), wrap.side * Math.cos(angle));
			}
			rest -= arc;
		}
		var last = strandList[strandList.length - 1];
		return new BeltPoint(last.endX, last.endY, last.dx, last.dy);
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var left = outline(thickness / 2), right = outline(-thickness / 2);
		var outer = Math.abs(area(left)) >= Math.abs(area(right)) ? left : right;
		var inner = outer == left ? right : left;
		return Solids.cut(Solids.named(Part.prism(outer, 0, width), "outer"), [Solids.named(Part.prism(inner, -0.1, width + 0.1), "inner")]);
	}

	/** The pitch line offset `offset` to the left of the way the belt travels, as a polygon. */
	function outline(offset:Float):Array<Vector> {
		var points:Array<Vector> = [];
		for (i in 0...strandList.length) {
			var run = strandList[i];
			var nx = -run.dy * offset, ny = run.dx * offset;
			points.push(new Vector(run.startX + nx, run.startY + ny));
			points.push(new Vector(run.endX + nx, run.endY + ny));
			var k = (i + 1) % loop.length, wrap = loop[k];
			var radius = wrap.radius - wrap.side * offset;
			var steps = Math.ceil(sweeps[k] / (Math.PI / 24));
			for (step in 1...steps) {
				var angle = arrivals[k] + wrap.side * sweeps[k] * step / steps;
				points.push(new Vector(wrap.x + radius * Math.cos(angle), wrap.y + radius * Math.sin(angle)));
			}
		}
		return points;
	}

	static function area(points:Array<Vector>):Float {
		var sum = 0.0;
		for (i in 0...points.length) {
			var a = points[i], b = points[(i + 1) % points.length];
			sum += a.x * b.y - b.x * a.y;
		}
		return sum / 2;
	}

	private static var pairRecipeTypeCache:Null<ComponentType>;

	/** Belt round a driver and an idler, in plain tooth counts and a centre distance. */
	public static function pairRecipeType():ComponentType {
		if (pairRecipeTypeCache == null)
			pairRecipeTypeCache = new ComponentType("machinekit.transmission.timing-belt",
			[ComponentRecipeSupport.choice("profile", ["GT2", "HTD3M", "HTD5M", "HTD8M", "HTD14M", "T5", "XL"], "GT2"),
				ComponentRecipeSupport.count("driverTeeth", 20), ComponentRecipeSupport.count("idlerTeeth", 20),
				ComponentRecipeSupport.length("centreDistance", 200), ComponentRecipeSupport.length("width", 6)],
			v -> TimingBelt.twoPulley(ComponentRecipeSupport.profile(v.token("profile")), v.integer("driverTeeth"),
				v.integer("idlerTeeth"), v.number("centreDistance"), v.number("width")));
		return pairRecipeTypeCache;
	}

	override public function componentType():Null<ComponentType> return pair == null ? null : pairRecipeType();

	override public function values():ComponentValues {
		var layout = pair;
		if (layout == null) throw 'Belt "$designation" has no recipe values';
		return new ComponentValues().setToken("profile", Std.string(beltProfile))
			.setInteger("driverTeeth", layout.driverTeeth).setInteger("idlerTeeth", layout.idlerTeeth)
			.setNumber("centreDistance", layout.centreDistance).setNumber("width", width)
			.setToken("material", materialSpec());
	}
}
