package machinekit.structural;

import materia.casting.InsertHole;
import materia.assembly.AssemblyFrames as F;
import machinekit.design.DesignInterfaces;
import machinekit.design.ThreadedHolePolicy.ThreadedHoleChoice;

/** Recipe patterns regenerate the authoritative DesignInterfaces facet. */
class InsertHoles {
	public static function threadRecord(spec:cadkit.threads.ThreadSpec):InsertThread {
		return {
			designation: spec.designation,
			majorDiameter: spec.majorDiameter,
			pitch: spec.pitch,
			starts: spec.starts,
			internal: spec.internal,
			toleranceClass: spec.toleranceClass,
			hand: switch spec.hand {
				case RightHand: Right;
				case LeftHand: Left;
			},
			profile: switch spec.profile {
				case Metric: InsertThreadProfile.Metric;
				case Trapezoidal: InsertThreadProfile.Trapezoidal;
				case Acme: InsertThreadProfile.Acme;
				case Ball: InsertThreadProfile.Ball;
			}
		};
	}

	public static function threadSpec(record:InsertThread):cadkit.threads.ThreadSpec {
		return new cadkit.threads.ThreadSpec(record.designation, record.majorDiameter, record.pitch, record.starts,
			record.hand == Right ? materia.project.ThreadSpecData.ThreadHand.RightHand : materia.project.ThreadSpecData.ThreadHand.LeftHand, record.internal,
			record.toleranceClass, switch record.profile {
				case Metric: materia.project.ThreadSpecData.ThreadProfile.Metric;
				case Trapezoidal: materia.project.ThreadSpecData.ThreadProfile.Trapezoidal;
				case Acme: materia.project.ThreadSpecData.ThreadProfile.Acme;
				case Ball: materia.project.ThreadSpecData.ThreadProfile.Ball;
				default: throw "Invalid insert thread profile";
			});
	}

	public static function metricThread(thread:cadkit.threads.ThreadSpec):ThreadedHoleChoice {
		var spec = machinekit.standard.SocketHeadCapScrew.catalog().get(thread.designation);
		if (!thread.internal || thread.pitch != spec.pitch)
			throw "Insert hole choice requires a catalogue internal coarse metric thread";
		// ISO 273, medium series; use the shared catalogue machining dimensions.
		return {thread: thread,
			tapDrill: spec.tapDrill, clearanceDiameter: spec.clearanceMedium};
	}

	public static function tappedThrough(choice:ThreadedHoleChoice, depth:Float):String {
		var holes = records(through(choice.tapDrill, depth, choice.thread));
		holes[0].clearanceDiameter = choice.clearanceDiameter;
		return fromNeutral(holes);
	}

	/** Flat-bottom blind bore (flat-bottom drill or end mill), followed by tapping.
	 * Cylindrical depth includes the tip gap and bottoming-tap runout; back wall is
	 * checked against the actual insert thickness before a recipe is emitted. */
	public static function tappedBlind(choice:ThreadedHoleChoice, thickness:Float, material:materia.casting.InsertMaterial = C45Bright):String {
		var limits = machinekit.design.ThreadedHolePolicy.resolve(choice.thread,material);
		var depth = thickness - limits.minBack;
		var usable = depth - choice.thread.pitch * machinekit.design.ThreadedHolePolicy.TAP_RUNOUT_PITCHES;
		if (usable < limits.minEngagement + limits.minTipGap)
			throw "Blind insert thread needs thicker stock: " + choice.thread.designation;
		var holes = records(tappedThrough(choice, thickness));
		holes[0].limits = machinekit.design.ThreadedHolePolicy.neutral(limits);
		holes[0].extent = Blind;
		holes[0].depth = depth;
		holes[0].threadDepth = usable;
		return fromNeutral(holes);
	}

	public static function through(diameter:Float, depth:Float, ?thread:cadkit.threads.ThreadSpec):String {
		if (!Math.isFinite(diameter + depth) || diameter <= 0 || depth <= 0)
			throw "Insert drilling dimensions must be finite and positive";
		return haxe.Json.stringify({
			version: 5,
			holes: [
				{
					id: "fixing",
					frame: F.fromRotationMatrix(0, 0, depth, [1, 0, 0, 0, -1, 0, 0, 0, -1]),
					diameter: diameter,
					depth: depth,
					extent: "through",
					thread: thread == null ? null : threadRecord(thread),
					tapDrill: thread == null ? null : diameter,
					threadDepth: thread == null ? null : depth,
					relief: null,
					clearanceDiameter: thread == null ? diameter
						: machinekit.standard.SocketHeadCapScrew.catalog().get(thread.designation).clearanceMedium,
					counterbore: null,
					limits: thread == null ? machinekit.design.ThreadedHolePolicy.clearance(diameter)
						: machinekit.design.ThreadedHolePolicy.neutral(machinekit.design.ThreadedHolePolicy.resolve(thread, C45Bright))
				}
			]
		});
	}

	public static function fromNeutral(holes:Array<InsertHole>):String
		return haxe.Json.stringify({version: 5, holes: InsertHoleCodec.snapshot(holes)});

	public static function records(text:String):Array<InsertHole> {
		var raw:Dynamic = haxe.Json.parse(text);
		materia.casting.CastingFeatureCodec.fields(raw, ["version", "holes"]);
		if (raw.version != 5 || !Std.isOfType(raw.holes, Array))
			throw "Unsupported insert hole recipe";
		var holes:Array<Dynamic> = cast raw.holes;
		return InsertHoleCodec.snapshot(cast holes);
	}

	public static function facet(holes:Array<InsertHole>, width:Float, depth:Float, height:Float):DesignInterfaces {
		var rows:Array<HoleInterface> = [];
		for (h in holes) {
			var boreDiameter = h.relief == null ? h.diameter : h.relief.diameter;
			if (h.counterbore != null) boreDiameter = Math.max(boreDiameter, h.counterbore.diameter);
			if (boreDiameter >= Math.min(width, depth))
				throw "Insert hole consumes its seat: " + h.id;
			var thread:Null<BlindThreadInterface> = null;
			var limits = h.limits;
			if (h.thread != null) {
				var tap = h.tapDrill;
				if (tap == null)
					throw "Threaded insert requires its tap drill";
				thread = {
					spec: threadSpec(h.thread),
					tapDrill: tap,
					usableDepth: cast h.threadDepth,
					cylindricalDepth: h.depth,
					point: 0,
					stock: [width,depth,height][materia.casting.InsertHoleGeometry.axis(h)],
					minEngagement: InsertHoleCodec.threadLimits(h).minEngagement,
					minTipGap: InsertHoleCodec.threadLimits(h).minTipGap,
					minBack: limits.minBack
				};
			}
			rows.push({
				insertLimits: limits,
				relief: h.relief,
				insertClearanceDiameter: h.clearanceDiameter,
				id: h.id,
				frame: h.frame,
				radius: h.diameter / 2,
				depth: h.depth,
				minWall: limits.minWall,
				minClearance: limits.minClearance,
				through: h.extent == Through,
				thread: thread,
				counterbore: h.counterbore == null ? null : {
					radius: h.counterbore.diameter / 2,
					depth: h.counterbore.depth,
					bearingDepth: h.counterbore.depth
				},
				approach: null
			});
		}
		return new DesignInterfaces(rows, [], []);
	}

	public static function copy(facet:DesignInterfaces):Array<InsertHole> {
		return InsertHoleCodec.snapshot([
			for (h in facet.holes)
				{
					id: h.id,
					limits: h.insertLimits,
					frame: h.frame,
					diameter: 2 * h.radius,
					depth: h.depth,
					extent: h.through == true ? Through : Blind,
					thread: h.thread == null ? null : threadRecord(h.thread.spec),
					tapDrill: h.thread == null ? null : h.thread.tapDrill,
					threadDepth: h.thread == null ? null : h.thread.usableDepth,
					relief: h.relief == null ? null : {diameter: h.relief.diameter, start: h.relief.start},
					clearanceDiameter: h.insertClearanceDiameter,
					counterbore: h.counterbore == null ? null : {
						diameter: 2 * h.counterbore.radius,
						depth: h.counterbore.depth
					}
				}
		]);
	}
}
