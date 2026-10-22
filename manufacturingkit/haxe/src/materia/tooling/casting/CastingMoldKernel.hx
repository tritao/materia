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
	public function section(region:CastingRegion,plane:materia.assembly.AssemblyRecord.AssemblyFrame,
		affectedId:String):Array<materia.casting.CastingSection.CastingSectionContour>;

	/** A candidate section is accepted only when its complete extrusion has
	 * symmetric difference within tolerance of the source solid. Null means
	 * the source is not a representable line/arc prism along this direction.
	 */
	public function prismaticSection(region:CastingRegion,direction:{x:Float,y:Float,z:Float},
		toleranceMm3:Float,affectedId:String):Null<materia.casting.CastingSection.CastingPrismSection>;
	public function castingMass(document:ToolingDocument):CastingMassProperties;
	/** True interior intersection with source resin, clipped at the declared fill plane. */
	public function resinIntersection(document:ToolingDocument,region:CastingRegion):Float;
}
