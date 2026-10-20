package machinekit.tooling;

import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingEnums.ToolingLengthUnit;
import materia.tooling.ToolingEnums.InsertRole;
import materia.tooling.ToolingEnums.InsertFinish;
import materia.tooling.ToolingPrism;
import materia.tooling.ToolingPrism.ToolingPoint2;
import materia.tooling.casting.CastingToolingInput;
import materia.tooling.casting.CastingToolingInput.CastingToolingFeature;
import materia.tooling.casting.CastingToolingInput.CastingToolingInsert;
import materia.tooling.casting.CastingToolingInput.CastingFeatureKind;
import machinekit.structural.MineralFoundation;
import machinekit.structural.CastingFeatures;

/** Copies design parameters through public APIs; never retains components or native shapes.
 * Envelope prisms are additive, and feature tools remain unclipped design cuts.
 * Their union/intersection must be resolved by the later mold planner.
 */
class FoundationToolingAdapter {
	public static function snapshot(foundation:MineralFoundation, id:String, revision:Int):CastingToolingInput {
		if (foundation == null)
			throw "Foundation tooling adapter needs a foundation";
		var v = foundation.body.values();
		var x = v.number("width") / 2,
			y = v.number("depth") / 2,
			c = v.number("cornerChamfer");
		if (c < 0)
			throw "Unsupported negative casting chamfer";
		var profile:Array<ToolingPoint2> = c == 0 ? [{x: -x, y: -y}, {x: x, y: -y}, {x: x, y: y}, {x: -x, y: y}] : [
			{
				x: -x + c,
				y: -y
			},
			{x: x - c, y: -y},
			{x: x, y: -y + c},
			{x: x, y: y - c},
			{x: x - c, y: y},
			{x: -x + c, y: y},
			{
				x: -x,
				y: y - c
			},
			{
				x: -x,
				y: -y + c
			}
		];
		var envelope:Array<ToolingPrism> = [
			{
				id: "body",
				profile: profile,
				height: v.number("height"),
				pose: F.identity()
			}
		];
		if (v.number("footWidth") > 0) {
			var f = v.number("footWidth") / 2,
				top = v.number("footHeight"),
				shoulder = v.number("footShoulder");
			envelope.push({
				id: "foot",
				profile: [
					{x: -f, y: 0},
					{x: f, y: 0},
					{x: f, y: shoulder},
					{x: x, y: top},
					{
						x: -x,
						y: top
					},
					{
						x: -f,
						y: shoulder
					}
				],
				height: 2 * y,
				pose: F.fromRotationMatrix(0, y, 0, [1, 0, 0, 0, 0, -1, 0, 1, 0])
			});
		}
		var inserts:Array<CastingToolingInsert> = [],
			recesses = new Map<String, String>();
		for (row in foundation.insertLocations()) {
			var i = row.part.values();
			inserts.push({
				id: row.id,
				width: i.number("width"),
				depth: i.number("depth"),
				height: i.number("height"),
				boreDiameter: i.number("boreDiameter"),
				pose: row.pose,
				role: cast i.token("role"),
				label: i.token("label"),
				positionTolerance: i.number("positionTolerance"),
				finish: sourceFinish(i.token("finish"))
			});
		}
		for (link in foundation.insertRecessLinks())
			recesses.set(link.recessId, link.insertId);
		var features:Array<CastingToolingFeature> = [];
		for (f in CastingFeatures.decode(v.token("voids"))) {
			var insertId = recesses.get(f.id);
			features.push({
				id: f.id,
				kind: insertId == null ? Void : InsertRecess,
				insertId: insertId,
				width: f.width,
				depth: f.depth,
				height: f.height,
				pose: f.pose
			});
		}
		return ToolingCodec.inputSnapshot({
			id: id,
			revision: revision,
			lengthUnit: ToolingLengthUnit.Millimetres,
			envelope: envelope,
			features: features,
			inserts: inserts
		});
	}

	static function sourceFinish(label:String):InsertFinish
		return switch label {
			case "ground after casting": InsertFinish.GroundAfterCasting;
			case "machined after casting": InsertFinish.MachinedAfterCasting;
			case "replicated after casting": InsertFinish.ReplicatedAfterCasting;
			case "as cast": InsertFinish.SourceAsCast;
			default: throw "Unsupported source insert finish: " + label;
		};
}
