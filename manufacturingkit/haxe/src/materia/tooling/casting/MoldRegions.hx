package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.Box3;
import materia.casting.BoxAlgebra;
import materia.casting.CastingRegion;
import materia.casting.CastingRegion.CastingRegionCodec;
import materia.casting.CastingFeature.CastingGeometrySolids;
import materia.casting.CastingFeature.CastingGeometrySolid;
import materia.tooling.casting.CastingMoldKernel.CastingRegionMoments;

/** Exact planning queries. Orthogonal expressions retain the shared box algebra;
 * other expressions require the application's exact kernel. Stock bounds never
 * enter material moments or interior-overlap decisions.
 */
class MoldRegions {
	public static function feature(document:materia.tooling.ToolingDocument,id:String):CastingRegion {
		var found=materia.casting.CastingCavities.derive(document.input.envelope,document.input.features)
			.filter(cavity -> cavity.featureId==id);
		if (found.length!=1) throw "Unknown tooling feature: "+id;
		// The authoritative cavity already knows whether its planar fragments
		// form exact boxes, including clipping vertices on collinear edges.
		if (materia.casting.CastingCavities.supportsBoxes(found[0]))
			return moved(CastingRegion.Union([for (piece in materia.casting.CastingCavities.boxes(found[0])) box(piece)]),
				document.setup.sourceToMold);
		return moved(materia.casting.CastingCavities.region(found[0]),document.setup.sourceToMold);
	}

	public static function envelope(document:materia.tooling.ToolingDocument):CastingRegion
		return moved(materia.casting.CastingCavities.envelopeRegion(document.input.envelope),document.setup.sourceToMold);

	public static function moved(region:CastingRegion,pose:AssemblyFrame):CastingRegion return switch region {
		case Placed(geometry): CastingRegion.Placed({solid:geometry.solid,pose:F.compose(pose,geometry.pose)});
		case Drafted(id,opening,frame,depth,degrees): CastingRegion.Drafted(id,opening,F.compose(pose,frame),depth,degrees);
		case Union(children): CastingRegion.Union([for (child in children) moved(child,pose)]);
		case Intersection(children): CastingRegion.Intersection([for (child in children) moved(child,pose)]);
		case Difference(body,cuts): CastingRegion.Difference(moved(body,pose),[for (cut in cuts) moved(cut,pose)]);
	};

	public static function box(value:Box3):CastingRegion
		return CastingRegion.Placed({solid:CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(value.width,value.depth,value.height)),
			pose:F.translation(value.x+value.width/2,value.y+value.depth/2,value.z)});

	public static function boxes(region:CastingRegion):Null<Array<Box3>>
		return orthogonal(CastingRegionCodec.snapshot(region));

	public static function moments(region:CastingRegion,affectedId:String):CastingRegionMoments {
		var normalized=CastingRegionCodec.snapshot(region),pieces=orthogonal(normalized);
		if (pieces!=null) {
			var volume=0.0,x=0.0,y=0.0,z=0.0;
			for (piece in pieces) {
				var v=piece.width*piece.depth*piece.height;
				volume+=v;
				x+=v*(piece.x+piece.width/2);
				y+=v*(piece.y+piece.depth/2);
				z+=v*(piece.z+piece.height/2);
			}
			return {volumeMm3:volume,firstMomentMm4:{x:x,y:y,z:z}};
		}
		var kernel=CastingMoldPlanner.geometry;
		if (kernel==null) throw "Exact tooling geometry kernel required: "+affectedId;
		try {
			var result=kernel.moments(normalized),first=result.firstMomentMm4;
			for (value in [result.volumeMm3,first.x,first.y,first.z])
				if (!Math.isFinite(value)) throw "Nonfinite region moments";
			if (result.volumeMm3<0 || (result.volumeMm3==0 && (first.x!=0 || first.y!=0 || first.z!=0)))
				throw "Invalid empty/material region moments";
			return result;
		} catch (error:Dynamic) {
			throw "Tooling geometry "+affectedId+": "+Std.string(error);
		}
	}

	public static function volume(region:CastingRegion,affectedId:String):Float
		return moments(region,affectedId).volumeMm3;

	public static function overlaps(a:CastingRegion,b:CastingRegion,toleranceMm3:Float,affectedId:String):Bool {
		if (!Math.isFinite(toleranceMm3) || toleranceMm3<0) throw "Invalid tooling contact tolerance";
		return volume(CastingRegion.Intersection([a,b]),affectedId)>toleranceMm3;
	}

	static function orthogonal(region:CastingRegion):Null<Array<Box3>> return switch region {
		case Placed(geometry):
			var candidate=geometry;
			switch geometry.solid {
				case Prism(profile,height):
					var bounds=materia.casting.CastingProfileGeometry.bounds(profile);
					if (profile.length!=4 || profile.filter(segment -> segment.center!=null
						|| (segment.start.x!=bounds.x && segment.start.x!=bounds.x+bounds.width)
						|| (segment.start.y!=bounds.y && segment.start.y!=bounds.y+bounds.height)).length>0) return null;
					candidate={solid:CastingGeometrySolid.Cut(materia.casting.CastingSolid.Box(bounds.width,bounds.height,height)),
						pose:F.compose(geometry.pose,F.translation(bounds.x+bounds.width/2,bounds.y+bounds.height/2,0))};
				default:
			}
			if (!CastingGeometrySolids.isBox(candidate.solid)) return null;
			try {
				return [materia.casting.CastingBounds.box(candidate)];
			} catch (error:Dynamic) {
				if (Std.string(error)!="Non-axis-aligned casting box") throw error;
				return null;
			}
		case Drafted(_,_,_,_,_):null;
		case Union(children):
			var all:Array<Box3> =[];
			for (child in children) {
				var next=orthogonal(child);
				if (next==null) return null;
				all=all.concat(next);
			}
			BoxAlgebra.subtract(all,[]);
		case Intersection(children):
			var result=orthogonal(children[0]);
			if (result==null) return null;
			for (index in 1...children.length) {
				var next=orthogonal(children[index]);
				if (next==null) return null;
				var intersections:Array<Box3> =[];
				for (a in result) for (b in next) {
					var common=BoxAlgebra.intersection(a,b);
					if (common!=null) intersections.push(common);
				}
				result=BoxAlgebra.subtract(intersections,[]);
			}
			result;
		case Difference(body,cuts):
			var source=orthogonal(body),removed:Array<Box3> =[];
			if (source==null) return null;
			for (cut in cuts) {
				var next=orthogonal(cut);
				if (next==null) return null;
				removed=removed.concat(next);
			}
			BoxAlgebra.subtract(source,removed);
	};
}
