package materia.casting;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingCavities.CastingFace3;
import materia.casting.FoundationDatumScheme.FoundationDatum;
import materia.casting.FoundationDatumScheme.FoundationDatumInsert;

/** Nominal reference-frame consistency, in source coordinates. These numerical
 * tolerances are identity checks, not manufacturing tolerances or proof that a
 * physical contact patch is available after cutting, lining or finishing.
 */
@:allow(materia.casting)
class FoundationDatumGeometry {
	static inline var POSITION_EPS_MM = 1e-7;
	static inline var DIRECTION_EPS = 1e-8;

	static function validate(scheme:FoundationDatumScheme, envelope:Array<CastingPrism>,
			features:Array<CastingFeature>, inserts:Array<FoundationDatumInsert>):Void {
		var bodyFaces = CastingCavities.boundary(envelope);
		var cavities = CastingCavities.derive(envelope, features);
		for (datum in FoundationDatumScheme.FoundationDatums.ordered(scheme)) switch datum.reference {
			case BodyFace(faceId): face(datum,bodyFaces.filter(face -> face.id==faceId),1);
			case FeatureFace(featureId,faceId):
				var cavity = cavities.filter(cavity -> cavity.featureId==featureId)[0];
				switch cavity.geometry {
					case Prisms(pieces):
						// A cavity boundary points into material; the casting face points into the cavity.
						face(datum,CastingCavities.boundary(pieces).filter(face -> face.id==faceId),-1);
					case SolidIntersection(_,_,_),UnsupportedGeometry(_):
						// Reference validation emits the unresolved feature record; no surrogate frame.
				}
			case InsertHole(insertId,holeId):
				var insert = inserts.filter(insert -> insert.id==insertId)[0];
				var hole = insert.holes.filter(hole -> hole.id==holeId)[0];
				var entry = F.compose(insert.pose,hole.frame);
				if (distance(datum.frame,entry)>POSITION_EPS_MM || !aligned(datum.frame,entry,0,0,1))
					throw "Datum frame does not match source hole entry: " + datum.id;
			case InsertInterface(insertId,interfaceId):
				var insert = inserts.filter(insert -> insert.id==insertId)[0];
				var requirement = insert.interfaces.filter(requirement -> requirement.id==interfaceId)[0];
				switch requirement.geometry {
					case Plane(frame):
						var source = F.compose(insert.pose,frame), normal = F.transformVector(source,0,0,1);
						var separation = normal.x*(datum.frame.x-source.x)+normal.y*(datum.frame.y-source.y)
							+normal.z*(datum.frame.z-source.z);
						if (Math.abs(separation)>POSITION_EPS_MM || !aligned(datum.frame,source,0,0,1))
							throw "Datum frame does not match source interface plane: " + datum.id;
					case Line(frame):
						var source = F.compose(insert.pose,frame), direction = F.transformVector(source,1,0,0);
						var x=datum.frame.x-source.x,y=datum.frame.y-source.y,z=datum.frame.z-source.z;
						var along=x*direction.x+y*direction.y+z*direction.z;
						var dx=x-along*direction.x,dy=y-along*direction.y,dz=z-along*direction.z;
						if (Math.sqrt(dx*dx+dy*dy+dz*dz)>POSITION_EPS_MM || !aligned(datum.frame,source,1,0,0))
							throw "Datum frame does not match source interface line: " + datum.id;
				}
		}
	}

	static function aligned(a:AssemblyFrame,b:AssemblyFrame,x:Float,y:Float,z:Float):Bool {
		var first=F.transformVector(a,x,y,z),second=F.transformVector(b,x,y,z);
		var dx=first.x-second.x,dy=first.y-second.y,dz=first.z-second.z;
		return Math.sqrt(dx*dx+dy*dy+dz*dz)<=DIRECTION_EPS;
	}

	static function distance(a:AssemblyFrame,b:AssemblyFrame):Float {
		var x=a.x-b.x,y=a.y-b.y,z=a.z-b.z;
		return Math.sqrt(x*x+y*y+z*z);
	}

	static function face(datum:FoundationDatum,faces:Array<CastingFace3>,materialNormalSign:Float):Void {
		var direction=F.transformVector(datum.frame,0,0,1);
		for (face in faces) {
			var points=face.points,normal={x:0.0,y:0.0,z:0.0};
			// Newell's area normal tolerates collinear vertices introduced by exact clipping.
			for (i in 0...points.length) {
				var a=points[i],b=points[(i+1)%points.length];
				normal.x+=(a.y-b.y)*(a.z+b.z);
				normal.y+=(a.z-b.z)*(a.x+b.x);
				normal.z+=(a.x-b.x)*(a.y+b.y);
			}
			var length=Math.sqrt(normal.x*normal.x+normal.y*normal.y+normal.z*normal.z);
			if (length<=1e-12) continue;
			normal.x/=length;normal.y/=length;normal.z/=length;
			var dx=direction.x-materialNormalSign*normal.x,dy=direction.y-materialNormalSign*normal.y,
				dz=direction.z-materialNormalSign*normal.z;
			if (Math.sqrt(dx*dx+dy*dy+dz*dz)>DIRECTION_EPS) continue;
			var origin=points[0];
			var planeDistance=normal.x*(datum.frame.x-origin.x)+normal.y*(datum.frame.y-origin.y)
				+normal.z*(datum.frame.z-origin.z);
			if (Math.abs(planeDistance)>POSITION_EPS_MM) continue;
			var inside=true;
			for (i in 0...points.length) {
				var a=points[i],b=points[(i+1)%points.length];
				var x=b.x-a.x,y=b.y-a.y,z=b.z-a.z;
				var px=datum.frame.x-a.x,py=datum.frame.y-a.y,pz=datum.frame.z-a.z;
				var cross=normal.x*(y*pz-z*py)+normal.y*(z*px-x*pz)+normal.z*(x*py-y*px);
				if (cross < -POSITION_EPS_MM*Math.sqrt(x*x+y*y+z*z)) inside=false;
			}
			if (inside) return;
		}
		throw "Datum frame does not match referenced face: " + datum.id;
	}
}
