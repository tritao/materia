package materia.casting;

import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertManufacturing.InsertBlank;
import materia.casting.InsertSpec.InsertBody;

enum InsertBlankFitResult {
	Unselected;
	Fits;
	Invalid(reason:String);
	Unsupported(reason:String);
}

/** Exact containment for primitive stock. Vertex tests are exact for a convex
 * box inside convex stock; cylinder supports are analytic, never box volumes.
 */
class InsertBlankFit {
	static inline var EPS_MM = 1e-8;
	public static function assess(body:InsertBody,blank:InsertBlank):InsertBlankFitResult {
		InsertSpec.InsertBodyCodec.validate(body);
		var selected=switch blank {
			case InsertBlank.Unselected:return InsertBlankFitResult.Unselected;
			case Stock(reference,solid,frame):{solid:solid,frame:frame};
		};
		CastingSolid.CastingSolidCodec.validate(selected.solid);
		materia.assembly.AssemblyCodec.validateFrame(selected.frame);
		// Use the same normalized rigid rotation as the native frame realization.
		selected.frame=F.fromRotationMatrix(selected.frame.x,selected.frame.y,selected.frame.z,F.toRotationMatrix(selected.frame));
		var solid=switch body {
			case Primitive(solid):solid;
			case CrimpedSocket(_,_):return Unsupported("Retention-tail blank containment requires its exact formed profile");
		};
		var points:Array<{x:Float,y:Float,z:Float}> = [];
		switch solid {
			case Box(width,depth,height):
				for (x in [-width/2,width/2]) for (y in [-depth/2,depth/2]) for (z in [0.0,height])
					points.push(F.transformPoint(selected.frame,x,y,z));
			case Cylinder(diameter,height):
				var axis=F.transformVector(selected.frame,0,0,1),start=F.transformPoint(selected.frame,0,0,0),
					end=F.transformPoint(selected.frame,0,0,height),radius=diameter/2;
				switch selected.solid {
					case Cylinder(stockDiameter,stockHeight):
						var transverse=Math.sqrt(axis.x*axis.x+axis.y*axis.y);
						if ((height+radius)*transverse>EPS_MM)
							return Unsupported("Oblique cylinder-in-cylinder blank containment requires exact curved-solid qualification");
						var offset=Math.max(Math.sqrt(start.x*start.x+start.y*start.y),Math.sqrt(end.x*end.x+end.y*end.y));
						if (offset+radius>stockDiameter/2+EPS_MM || Math.min(start.z,end.z)-radius*transverse < -EPS_MM
							|| Math.max(start.z,end.z)+radius*transverse>stockHeight+EPS_MM)
							return Invalid("Selected round stock does not contain the insert body");
						return Fits;
					case Box(width,depth,stockHeight):
						var starts=[start.x,start.y,start.z],ends=[end.x,end.y,end.z],directions=[axis.x,axis.y,axis.z];
						var low=[-width/2,-depth/2,0.0],high=[width/2,depth/2,stockHeight];
						for (index in 0...3) {
							var span=radius*Math.sqrt(Math.max(0,1-directions[index]*directions[index]));
							if (Math.min(starts[index],ends[index])-span<low[index]-EPS_MM
								|| Math.max(starts[index],ends[index])+span>high[index]+EPS_MM)
								return Invalid("Selected rectangular stock does not contain the insert body");
						}
						return Fits;
				}
		}
		for (point in points) switch selected.solid {
			case Box(width,depth,height):
				if (Math.abs(point.x)>width/2+EPS_MM || Math.abs(point.y)>depth/2+EPS_MM
					|| point.z < -EPS_MM || point.z>height+EPS_MM)
					return Invalid("Selected rectangular stock does not contain the insert body");
			case Cylinder(diameter,height):
				if (Math.sqrt(point.x*point.x+point.y*point.y)>diameter/2+EPS_MM || point.z < -EPS_MM || point.z>height+EPS_MM)
					return Invalid("Selected round stock does not contain the insert body");
		}
		return Fits;
	}
}
