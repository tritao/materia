package machinekit.milling;

import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import materia.casting.InsertSpec.InsertRole;
import materia.casting.CastingPatterns;
import machinekit.structural.CastingFeaturePatterns;
import machinekit.structural.MineralFoundation;
import materia.assembly.AssemblyFrames;

/** Mill layout on the shared casting/foundation components. Dimensions are conceptual, in mm.
 * Two chambers and a cross rib retain a front service passage. A steel cap is cast into the roof.
 */
class MillPedestal extends MineralFoundation {
	public function new(width:Float, depth:Float, height:Float, ?details:MillPedestalDetails) {
		var d = details == null ? new MillPedestalDetails() : details;
		if (!Math.isFinite(width + depth + height) || width < d.minimumDimension || depth < d.minimumDimension || height < d.minimumDimension)
			throw "Mill pedestal needs finite dimensions of at least " + d.minimumDimension + " mm";
		var features = CastingPatterns.grid(width, depth, height, d.wall, d.sole, d.roof, d.rib, d.chamberColumns, d.chamberRows);
		features.push(CastingFeaturePatterns.box("service-opening", ServiceOpening,
			[CastingPatterns.face("body.y-min")], d.serviceWidth, d.serviceDepth, d.serviceHeight, 0, -depth / 2 + d.serviceCentreInset, d.serviceFloor));
		features.push(CastingFeaturePatterns.box("rib-passage", Clearance, [], d.serviceWidth, d.ribPassageDepth, d.serviceHeight, 0, 0, d.serviceFloor));
		var v = new ComponentValues().setNumber("width", width)
			.setNumber("depth", depth)
			.setNumber("height", height)
			.setNumber("cornerChamfer", d.cornerChamfer)
			.setToken("features", CastingPatterns.encode(features));
		var cap = new CastInsert(new ComponentValues().setNumber("width", width)
			.setNumber("depth", depth)
			.setNumber("height", d.capThickness)
			.setToken("role", InsertRole.GenericMount)
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
			pose: AssemblyFrames.translation(0, 0, height - d.capEmbedment)
		}]);
	}
}
