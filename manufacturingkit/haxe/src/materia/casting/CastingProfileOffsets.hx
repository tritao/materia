package materia.casting;

import materia.casting.CastingPrism.CastingPoint2;
import materia.casting.CastingProfile.CastingSegment;
import materia.casting.CastingProfile.CastingCurveKind;

/** Parallel wall sections for draft. This changes finished geometry; it is not
 * router compensation. A positive distance moves the contour into its interior.
 * Topology changes (vanishing arcs, reversed edges, crossing walls) are errors.
 */
class CastingProfileOffsets {
	static inline var EPS=1e-7;

	public static function inward(profile:Array<CastingSegment>,distance:Float):Array<CastingSegment> {
		var source=CastingProfile.CastingProfileCodec.snapshot(profile);
		if (!Math.isFinite(distance) || distance<0) throw "Invalid draft section offset";
		if (distance==0) return source;
		var orientation=CastingProfileGeometry.area(source)>0 ? 1.0 : -1.0;
		var shifted=[for (segment in source) shift(segment,distance*orientation)];
		var joins:Array<CastingPoint2> =[];
		for (index in 0...source.length) {
			var previous=(index+source.length-1)%source.length;
			var a=shifted[previous],b=shifted[index];
			if (CastingProfiles.distance(a.end,b.start)<EPS) {
				joins.push({x:(a.end.x+b.start.x)/2,y:(a.end.y+b.start.y)/2});
				continue;
			}
			var candidates=intersections(a,b);
			if (candidates.length==0) throw "Draft section walls do not meet";
			var corner=source[index].start;
			candidates.sort((x,y) -> Reflect.compare(CastingProfiles.distance(corner,x),CastingProfiles.distance(corner,y)));
			joins.push(candidates[0]);
		}
		var result:Array<CastingSegment> =[];
		for (index in 0...source.length) {
			var segment=shifted[index],a=joins[index],b=joins[(index+1)%source.length];
			if (segment.center==null) {
				var dx=segment.end.x-segment.start.x,dy=segment.end.y-segment.start.y;
				if ((b.x-a.x)*dx+(b.y-a.y)*dy<=EPS) throw "Draft section edge collapses";
				result.push(CastingProfiles.line(a,b));
			} else {
				var center=segment.center;
				var sweep=Math.atan2(b.y-center.y,b.x-center.x)-Math.atan2(a.y-center.y,a.x-center.x);
				if (segment.sweep>0 && sweep<=0) sweep+=2*Math.PI;
				if (segment.sweep<0 && sweep>=0) sweep-=2*Math.PI;
				if (Math.abs(sweep-segment.sweep)>Math.PI) throw "Draft section arc changes topology";
				result.push({kind:CastingCurveKind.Arc,start:a,end:b,center:center,sweep:sweep});
			}
		}
		CastingProfileGeometry.validate(result);
		if (CastingProfileGeometry.area(result)*orientation<=EPS
			|| Math.abs(CastingProfileGeometry.area(result))>=Math.abs(CastingProfileGeometry.area(source)))
			throw "Draft section collapses";
		return result;
	}

	static function shift(segment:CastingSegment,distance:Float):CastingSegment {
		var center=segment.center;
		if (center==null) {
			var dx=segment.end.x-segment.start.x,dy=segment.end.y-segment.start.y,length=Math.sqrt(dx*dx+dy*dy);
			return CastingProfiles.line({x:segment.start.x-dy/length*distance,y:segment.start.y+dx/length*distance},
				{x:segment.end.x-dy/length*distance,y:segment.end.y+dx/length*distance});
		}
		var radius=CastingProfiles.distance(center,segment.start),next=radius-distance*(segment.sweep>0 ? 1 : -1);
		if (next<=EPS) throw "Draft section arc collapses";
		return {kind:CastingCurveKind.Arc,center:center,sweep:segment.sweep,
			start:{x:center.x+(segment.start.x-center.x)*next/radius,y:center.y+(segment.start.y-center.y)*next/radius},
			end:{x:center.x+(segment.end.x-center.x)*next/radius,y:center.y+(segment.end.y-center.y)*next/radius}};
	}

	/** Intersections of unbounded support lines/circles, before trimming. */
	static function intersections(a:CastingSegment,b:CastingSegment):Array<CastingPoint2> {
		var ca=a.center,cb=b.center;
		if (ca==null && cb==null) {
			var ax=a.end.x-a.start.x,ay=a.end.y-a.start.y,bx=b.end.x-b.start.x,by=b.end.y-b.start.y;
			var determinant=ax*by-ay*bx;
			if (Math.abs(determinant)<1e-12) return [];
			var t=((b.start.x-a.start.x)*by-(b.start.y-a.start.y)*bx)/determinant;
			return [{x:a.start.x+t*ax,y:a.start.y+t*ay}];
		}
		if (ca==null || cb==null) {
			var line:CastingSegment=ca==null ? a : b;
			var circle:CastingSegment=ca==null ? b : a;
			var center=circle.center;
			if (center==null) throw "Missing draft arc center";
			var dx=line.end.x-line.start.x,dy=line.end.y-line.start.y,fx=line.start.x-center.x,fy=line.start.y-center.y;
			var aa=dx*dx+dy*dy,bb=2*(fx*dx+fy*dy),radius=CastingProfiles.distance(center,circle.start);
			var discriminant=bb*bb-4*aa*(fx*fx+fy*fy-radius*radius);
			if (discriminant < -EPS) return [];
			return [for (sign in [-1.0,1.0]) {
				var t=(-bb+sign*Math.sqrt(Math.max(0,discriminant)))/(2*aa);
				{x:line.start.x+t*dx,y:line.start.y+t*dy};
			}];
		}
		var distance=CastingProfiles.distance(ca,cb),r=CastingProfiles.distance(ca,a.start),q=CastingProfiles.distance(cb,b.start);
		if (distance<EPS || distance>r+q+EPS || distance<Math.abs(r-q)-EPS) return [];
		var x=(r*r-q*q+distance*distance)/(2*distance),h=Math.sqrt(Math.max(0,r*r-x*x));
		var ux=(cb.x-ca.x)/distance,uy=(cb.y-ca.y)/distance;
		return [for (sign in [-1.0,1.0]) {x:ca.x+x*ux-sign*h*uy,y:ca.y+x*uy+sign*h*ux}];
	}
}
