package materia.casting;

import materia.assembly.AssemblyFrames as F;
import materia.casting.InterfaceRequirement;
import materia.casting.InterfaceRequirement.InterfaceRequirements;
import materia.casting.InsertSpec.InsertBody;

/** Verified nominal layer over the complete matched stock face, in insert coordinates. Hole manufacturing, finishing
 * process capability and physical inspection are separate qualifications.
 */
enum InterfaceStockResult {
	NoLayer;
	Remove(layer:Box3);
	Replicate(layer:Box3);
	Invalid(reason:String);
	Unsupported(reason:String);
}

class InterfaceStocks {
	/** Numerical source-geometry identity threshold, not a stock design allowance. */
	static inline var EPS_MM = 1e-8;

	public static function assess(body:InsertBody,authored:InterfaceRequirement):InterfaceStockResult {
		InsertSpec.InsertBodyCodec.validate(body);
		var requirement=InterfaceRequirements.snapshot([authored])[0];
		var frame=switch requirement.geometry {
			case Plane(frame): frame;
			case Line(_):
				return requirement.allowanceMm==0 ? NoLayer
					: Unsupported("An adjusted line does not define the normal or footprint of a stock layer");
		};
		var normal=F.transformVector(frame,0,0,1),axis=CastingBounds.axis(normal);
		if (axis==null) return Unsupported("Finishing stock on a non-axis-aligned interface requires an exact finite face profile");
		var dimensions=switch body {
			case Primitive(solid): switch solid {
				case Box(width,depth,height): {width:width,depth:depth,height:height};
				case Cylinder(_,height):
					if (axis!=2 || requirement.route!=InterfaceRoute.AsSupplied && requirement.route!=InterfaceRoute.AsCast)
					return Unsupported("Non-box finishing layers require an exact profile-capable stock model");
					var boundary=normal.z>0 ? height : 0.0;
					return Math.abs(frame.z-boundary)<=EPS_MM ? NoLayer : Invalid("As-made interface is not on the cylindrical end face");
			};
			case CrimpedSocket(_,_): return Unsupported("Catalogue retention geometry requires its own finite precision face reference");
		};
		var low=[-dimensions.width/2,-dimensions.depth/2,0.0],
			high=[dimensions.width/2,dimensions.depth/2,dimensions.height],
			coordinates=[frame.x,frame.y,frame.z],normals=[normal.x,normal.y,normal.z];
		var sign=normals[axis]>0 ? 1.0 : -1.0,outer=sign>0 ? high[axis] : low[axis],finished=coordinates[axis];
		var offset=sign*(outer-finished);
		if (requirement.route==InterfaceRoute.AsSupplied || requirement.route==InterfaceRoute.AsCast)
			return Math.abs(offset)<=EPS_MM ? NoLayer : Invalid("As-made interface is not on the insert boundary");
		if (requirement.allowanceMm==0) return Invalid("Finishing requires a declared positive stock thickness or replication gap");
		if (requirement.allowanceMm<=EPS_MM) return Unsupported("Declared finishing layer is below source-geometry resolution");
		var addition=requirement.route==InterfaceRoute.Replicated;
		var actual=addition ? -offset : offset;
		if (Math.abs(actual-requirement.allowanceMm)>EPS_MM)
			return Invalid("Declared allowance "+requirement.allowanceMm+" mm differs from the modeled layer "+actual+" mm");
		if (!addition && sign*(finished-(sign>0 ? low[axis] : high[axis]))<=EPS_MM)
			return Invalid("Finished interface leaves no retained insert material");
		var start=low.copy(),size=[dimensions.width,dimensions.depth,dimensions.height];
		start[axis]=Math.min(outer,finished);size[axis]=actual;
		var layer:Box3={x:start[0],y:start[1],z:start[2],width:size[0],depth:size[1],height:size[2]};
		return addition ? Replicate(layer) : Remove(layer);
	}
}
