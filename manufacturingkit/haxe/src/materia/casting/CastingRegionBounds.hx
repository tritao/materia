package materia.casting;

import materia.casting.CastingRegion;
import materia.casting.CastingFeature;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Conservative bounds for blanks and broad-phase checks only. A boolean
 * region's bounds never substitute for its material, cavity or displaced volume.
 */
class CastingRegionBounds {
	public static function box(region:CastingRegion):Null<Box3> return switch region {
		case Placed(geometry):
			var width=0.0,depth=0.0,height=CastingGeometrySolids.height(geometry.solid);
			var local:{x:Float,y:Float,width:Float,height:Float};
			switch geometry.solid {
				case Prism(profile,_):local=CastingProfileGeometry.bounds(profile);
				default:
					switch CastingGeometrySolids.mouth(geometry.solid) {
						case Box(w,d,_):width=w;depth=d;
						case Cylinder(diameter,_):width=diameter;depth=diameter;
					}
					switch geometry.solid {
						case Insert(CrimpedSocket(_,tail)):
							width=Math.max(width,tail.width);
							depth=Math.max(depth,tail.thickness+2*Math.abs(tail.wave));
						default:
					}
					local={x:-width/2,y:-depth/2,width:width,height:depth};
			}
			placed(local,height,geometry.pose);
		case Drafted(id,opening,pose,depth,degrees):
			var sections=CastingDraftSections.sections(id,opening,pose,depth,degrees);
			var a=CastingProfileGeometry.bounds(sections.opening),b=CastingProfileGeometry.bounds(sections.inner);
			var x=Math.min(a.x,b.x),y=Math.min(a.y,b.y);
			placed({x:x,y:y,width:Math.max(a.x+a.width,b.x+b.width)-x,height:Math.max(a.y+a.height,b.y+b.height)-y},depth,pose);
		case Union(regions):
			var result:Null<Box3> =null;
			for (region in regions) {
				var next=box(region);
				if (next==null) continue;
				if (result==null) result=next;
				else {
					var x=Math.min(result.x,next.x),y=Math.min(result.y,next.y),z=Math.min(result.z,next.z);
					result={x:x,y:y,z:z,width:Math.max(result.x+result.width,next.x+next.width)-x,
						depth:Math.max(result.y+result.depth,next.y+next.depth)-y,height:Math.max(result.z+result.height,next.z+next.height)-z};
				}
			}
			result;
		case Intersection(regions):
			var result=box(regions[0]);
			for (index in 1...regions.length) {
				var next=box(regions[index]);
				if (result==null || next==null) {result=null;break;}
				result=BoxAlgebra.intersection(result,next);
			}
			result;
		case Difference(body,_):box(body);
	};
	static function placed(local:{x:Float,y:Float,width:Float,height:Float},height:Float,pose:AssemblyFrame):Box3 {
		var low=[Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY];
		var high=[Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY];
		for (x in [local.x,local.x+local.width]) for (y in [local.y,local.y+local.height]) for (z in [0.0,height]) {
			var point=F.transformPoint(pose,x,y,z),coordinates=[point.x,point.y,point.z];
			for (axis in 0...3) {low[axis]=Math.min(low[axis],coordinates[axis]);high[axis]=Math.max(high[axis],coordinates[axis]);}
		}
		return {x:low[0],y:low[1],z:low[2],width:high[0]-low[0],depth:high[1]-low[1],height:high[2]-low[2]};
	}
}
