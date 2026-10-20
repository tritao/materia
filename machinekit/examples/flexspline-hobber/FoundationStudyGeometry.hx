import materia.casting.Box3;
import materia.casting.BoxAlgebra;
import materia.casting.CastingBounds;
import materia.casting.CastingFeature.CastingFeatureKind;

/** Analysis regions are not source cuts: candidates have no authored source kind. */
typedef FoundationStudyRegion = {
	var id:String;
	var kind:Null<CastingFeatureKind>;
	var box:Box3;
}

/** Exact non-insert cavities for the interface studies; inserts are queried separately. */
class FoundationStudyGeometry {
	public static function region(id:String, w:Float, d:Float, h:Float, x:Float, y:Float, z:Float):FoundationStudyRegion
		return {id: id, kind: null, box: {x: x - w / 2, y: y - d / 2, z: z, width: w, depth: d, height: h}};

	public static function bounds(row:FoundationStudyRegion):Array<Float>
		return BoxAlgebra.limits(row.box);

	public static function designRegions():Array<FoundationStudyRegion> {
		var body = new machinekit.hobbing.MineralCastBed().body;
		var kinds = new Map<String, CastingFeatureKind>();
		for (feature in body.castingFeatures())
			kinds.set(feature.id, feature.kind);
		var result:Array<FoundationStudyRegion> = [];
		for (cavity in body.finishedCavities()) {
			var kind = kinds.get(cavity.featureId);
			if (kind == InsertRecess)
				continue;
			for (box in materia.casting.CastingCavities.boxes(cavity))
				result.push({id: cavity.featureId, kind: kind, box: box});
		}
		return result;
	}

	public static function insertRegion(row:machinekit.hobbing.FoundationLayout.InsertLocation):FoundationStudyRegion
		return {id: row.id, kind: InsertRecess,
			box: CastingBounds.box({kind: Box, width: row.width, depth: row.depth, height: row.height, pose: row.pose})};
}
