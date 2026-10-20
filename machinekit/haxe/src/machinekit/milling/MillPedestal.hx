package machinekit.milling;

import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import materia.casting.CastingPatterns;
import machinekit.structural.CastingFeaturePatterns;
import machinekit.structural.MineralFoundation;
import materia.assembly.AssemblyFrames;

/** Mill layout on the shared casting/foundation components. Dimensions are conceptual, in mm.
 * Two chambers and a cross rib retain a front service passage. A steel cap is cast into the roof.
 */
class MillPedestal extends MineralFoundation {
	public function new(width:Float, depth:Float, height:Float) {
		if (!Math.isFinite(width + depth + height) || width < 600 || depth < 600 || height < 600)
			throw "Mill pedestal needs finite dimensions of at least 600 mm";
		var features = CastingPatterns.grid(width, depth, height, 90, 100, 90, 25, 1, 2);
		features.push(CastingFeaturePatterns.box("service-opening", ServiceOpening,
			[CastingPatterns.face("body.y-min")], 360, 100, 380, 0, -depth / 2 + 40, 150));
		features.push(CastingFeaturePatterns.box("rib-passage", Clearance, [], 360, 27, 380, 0, 0, 150));
		var v = new ComponentValues().setNumber("width", width)
			.setNumber("depth", depth)
			.setNumber("height", height)
			.setNumber("cornerChamfer", 30)
			.setToken("features", CastingPatterns.encode(features));
		var cap = new CastInsert(new ComponentValues().setNumber("width", width)
			.setNumber("depth", depth)
			.setNumber("height", 20)
			.setToken("role", "generic-mount")
			.setToken("label", "mill support cap"));
		super(v, [{
			id: "cap",
			part: cap,
			openings: [
				CastingPatterns.face("body.z-max"), CastingPatterns.face("body.x-min"), CastingPatterns.face("body.x-max"),
				CastingPatterns.face("body.y-min"), CastingPatterns.face("body.y-max"),
				CastingPatterns.face("body.side-1"), CastingPatterns.face("body.side-3"),
				CastingPatterns.face("body.side-5"), CastingPatterns.face("body.side-7")
			],
			pose: AssemblyFrames.translation(0, 0, height - 12)
		}]);
	}
}
