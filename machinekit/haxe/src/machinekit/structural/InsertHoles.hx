package machinekit.structural;

import materia.casting.InsertHole;
import materia.assembly.AssemblyFrames as F;
import machinekit.design.DesignInterfaces;

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

	public static function tappedThrough(size:String):String {
		var screw = machinekit.standard.SocketHeadCapScrew.metric(size, 35);
		return through(screw.spec.tapDrill, screw.thread.mating());
	}

	public static function through(diameter:Float, ?thread:cadkit.threads.ThreadSpec):String {
		return haxe.Json.stringify({
			version: 1,
			holes: diameter == 0 ? [] : [
				{
					id: "fixing",
					frame: F.identity(),
					diameter: diameter,
					depth: null,
					extent: "through",
					thread: thread == null ? null : threadRecord(thread),
					tapDrill: thread == null ? null : diameter,
					counterbore: null
				}
			]
		});
	}

	public static function fromNeutral(holes:Array<InsertHole>):String
		return haxe.Json.stringify({version: 1, holes: InsertHoleCodec.snapshot(holes)});

	public static function records(text:String, height:Float):Array<InsertHole> {
		var raw:Dynamic = haxe.Json.parse(text);
		materia.casting.CastingFeatureCodec.fields(raw, ["version", "holes"]);
		if (raw.version != 1 || !Std.isOfType(raw.holes, Array))
			throw "Unsupported insert hole recipe";
		var holes:Array<Dynamic> = cast raw.holes;
		for (h in holes)
			if (h.extent == "through" && h.depth == null) {
				h.depth = height;
				h.frame = F.fromRotationMatrix(0, 0, height, [1, 0, 0, 0, -1, 0, 0, 0, -1]);
			}
		return InsertHoleCodec.snapshot(cast holes);
	}

	public static function facet(holes:Array<InsertHole>, width:Float, depth:Float, height:Float):DesignInterfaces {
		var rows:Array<HoleInterface> = [];
		for (h in holes) {
			if (h.diameter >= Math.min(width, depth))
				throw "Insert hole consumes its seat: " + h.id;
			var thread:Null<BlindThreadInterface> = null;
			if (h.thread != null) {
				var tap = h.tapDrill;
				if (tap == null)
					throw "Threaded insert requires its tap drill";
				thread = {
					spec: threadSpec(h.thread),
					tapDrill: tap,
					usableDepth: h.depth,
					cylindricalDepth: h.depth,
					point: 0,
					stock: height,
					minEngagement: 0.001,
					minTipGap: 0.001,
					minBack: 0.001
				};
			}
			rows.push({
				id: h.id,
				frame: h.frame,
				radius: h.diameter / 2,
				depth: h.depth,
				minWall: 0.001,
				minClearance: 0.001,
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
					frame: h.frame,
					diameter: 2 * h.radius,
					depth: h.depth,
					extent: h.through == true ? Through : Blind,
					thread: h.thread == null ? null : threadRecord(h.thread.spec),
					tapDrill: h.thread == null ? null : h.thread.tapDrill,
					counterbore: h.counterbore == null ? null : {
						diameter: 2 * h.counterbore.radius,
						depth: h.counterbore.depth
					}
				}
		]);
	}
}
