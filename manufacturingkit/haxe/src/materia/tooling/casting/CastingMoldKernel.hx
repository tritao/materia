package materia.tooling.casting;

import materia.casting.CastingRegion;
import materia.tooling.ToolingDocument;

/** Geometric moments in mm: first moments are zero for empty material. These
 * are design-geometry results, not physical capacity or qualification evidence.
 */
typedef CastingRegionMoments = {
	var volumeMm3:Float;
	var firstMomentMm4:{x:Float,y:Float,z:Float};
}

typedef CastingMassProperties = {
	var massKg:Float;
	var centerOfMassMoldMm:{x:Float,y:Float,z:Float};
}

/** An application supplies exact boolean geometry for the non-box planning
 * path. ManufacturingKit stays independent of CAD libraries. The orthogonal
 * path retains its existing exact algebra and does not need this service.
 */
interface CastingMoldKernel {
	public function moments(region:CastingRegion):CastingRegionMoments;
	public function castingMass(document:ToolingDocument):CastingMassProperties;
	/** True interior intersection with source resin, clipped at the declared fill plane. */
	public function resinIntersection(document:ToolingDocument,region:CastingRegion):Float;
}
