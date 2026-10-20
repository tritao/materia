package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Polygon;
import cadkit.threads.ThreadSpec;
import cadkit.threads.Threads;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.structural.LevelingFootSpec;
import machinekit.structural.LevelingFootSpec.LevelingFootCodec;
import machinekit.structural.LevelingFootSpec.FootStemEnvelope;

/** Shared nominal foot hardware: bought foot, pad, screw, removable plate or nut.
 * Saved dimensions are layout assumptions. Supplier geometry and load qualification remain pending.
 */
class LevelingFoot extends AssumedCatalogComponent {
	static var recipe:Null<ComponentType>;
	final spec:LevelingFootSpec;

	public static function parameters():Array<ComponentParameter>
		return [P.text("part", LevelingFootCodec.encode(Foot(100, 20,
			machinekit.standard.FastenerThreads.metric(20, 2.5), 20, 120, 100, Nominal, NoCollar)))];

	public static function recipeType():ComponentType {
		if (recipe == null)
			recipe = new ComponentType("machinekit.structural.leveling-foot", parameters(), v -> new LevelingFoot(v));
		return recipe;
	}

	public static function of(spec:LevelingFootSpec):LevelingFoot
		return new LevelingFoot(new ComponentValues().setToken("part", LevelingFootCodec.encode(spec)));

	public function new(?overrides:ComponentValues) {
		var v = P.resolve(parameters(), overrides);
		var parsed = LevelingFootCodec.decode(v.token("part"));
		v.setToken("part", LevelingFootCodec.encode(parsed));
		super("leveling-foot", partLabel(parsed) + "; capacity unverified", "steel", parameters(), v);
		spec = parsed;
		var top = switch spec {
			case Foot(_, _, _, _, _, mount, _, _): mount;
			case Pad(_, height), Nut(_, height, _): height;
			case Screw(_, length, _): length;
			case Plate(_, _, height, _): height;
		};
		addConnector("floor", Mount, Solids.axial(0, 0, 0));
		addConnector("mount", Mount, Solids.axial(0, 0, specIsFoot() ? top : 0));
		addConnector("frame", Mount, Solids.axial(0, 0, top));
		var physicalTop = switch spec {
			case Foot(_, _, _, _, stemTop, _, _, _): stemTop;
			default: top;
		};
		addConnector("top", Mount, Solids.axial(0, 0, physicalTop));
	}

	static function partLabel(part:LevelingFootSpec):String return switch part {
		case Foot(d, _, thread, _, _, _, _, _): '${thread.designation} levelling foot, pad D$d';
		case Pad(d, h): 'Levelling pad D$d x $h';
		case Screw(thread, length, _): '${thread.designation} levelling stud L$length';
		case Plate(w, d, h, bore): 'Removable foot bearing plate $w x $d x $h, clearance D$bore';
		case Nut(profile, height, thread):
			var outside = switch profile {
				case Round(d): 'round D$d';
				case Hexagon(af): 'hexagon AF$af';
			};
			'${thread.designation} foot nut, $outside x $height';
	};

	public function specification():LevelingFootSpec
		return LevelingFootCodec.decode(parameterToken("part"));

	function specIsFoot():Bool return switch spec {
		case Foot(_, _, _, _, _, _, _, _): true;
		default: false;
	};

	override public function componentType():Null<ComponentType> return recipeType();

	static function radius(thread:ThreadSpec, envelope:FootStemEnvelope):Float return switch envelope {
		case Nominal: thread.majorDiameter / 2;
		case ThreadRoot: thread.externalMinorDiameter() / 2;
	};

	static function threadedCylinder(radius:Float, bottom:Float, top:Float, thread:ThreadSpec):Part {
		var raw = Solids.named(Part.cylinderSpan(radius, bottom, top), "thread");
		try {
			var result = new Part(Threads.attach(raw.shape, "thread:cyl.side", thread, AxisMinimum));
			raw.close();
			return result;
		} catch (error:Dynamic) {
			raw.close();
			throw error;
		}
	}

	override public function geometry(detail:ComponentDetail = Preview):Part return switch spec {
		case Foot(d, h, thread, bottom, top, _, envelope, collar):
			Solids.building([], owned -> {
				var pad = Part.cylinderSpan(d / 2, 0, h); owned.push(pad);
				var stem = threadedCylinder(radius(thread, envelope), bottom, top, thread); owned.push(stem);
				var pieces = [pad, stem];
				switch collar {
					case NoCollar:
					case RoundCollar(cd, ch, z):
						var ring = Part.cylinderSpan(cd / 2, z, z + ch); owned.push(ring); pieces.push(ring);
				}
				return Solids.union(pieces);
			});
		case Pad(d, h): Part.cylinderSpan(d / 2, 0, h);
		case Screw(thread, length, envelope): threadedCylinder(radius(thread, envelope), 0, length, thread);
		case Plate(w, d, h, bore):
			Solids.building([], owned -> {
				var blank = Part.box(w, d, h); owned.push(blank);
				var hole = Part.cylinderSpan(bore / 2, -1, h + 1); owned.push(hole);
				return Solids.cut(blank, [hole]);
			});
		case Nut(profile, height, thread):
			Solids.building([], owned -> {
				var blank = switch profile {
					case Round(d): Part.cylinderSpan(d / 2, 0, height);
					case Hexagon(af): Part.prism(Polygon.regular(6, af), 0, height);
				};
				owned.push(blank);
				var hole = threadedCylinder(thread.majorDiameter / 2, -1, height + 1, thread.mating()); owned.push(hole);
				return Solids.cut(blank, [hole]);
			});
	};
}
