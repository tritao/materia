package machinekit.tooling;

import cadkit.modeling.Part;
import cadkit.modeling.Model;
import cadkit.modeling.Scope;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import cadkit.modeling.Sketch;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingProfile.CastingSegment;
import materia.casting.CastingSection.CastingSectionContour;
import materia.casting.CastingProfileGeometry as G;
import materia.casting.CastingProfiles as P;

/** Exact BREP sections in the declared plane's XY coordinates, shared by
 * drawings and machining. Only straight/circular boundaries become line/arc
 * records; unsupported curves and ambiguous junctions never become polygons.
 */
class CastingSectionContours {
	static inline var EPS=1e-7;

	public static function section(part:Part,plane:AssemblyFrame,sourceId:String):Array<CastingSectionContour> {
		var scope=new Scope();
		try {
			var checked=materia.casting.CastingRecordCodec.frame(plane);
			if (part.isEmpty()) {scope.close();return [];}
			var local=scope.own(machinekit.assembly.PosedParts.moved(part,F.inverse(checked)));
			var bounds=local.shape.bounds(),low=Vector.fromNative(bounds.get_min()),high=Vector.fromNative(bounds.get_max());
			var margin=Math.max(1,high.subtract(low).length());
			var face=scope.own(Sketch.rectangle(high.x-low.x+2*margin,high.y-low.y+2*margin,
				new Plane(new Vector((low.x+high.x)/2,(low.y+high.y)/2,0),Vector.X(),Vector.Z())));
			// Filled planar intersection removes internal seams at blind-hole
			// floors. A boundary-edge section alone can mistake those for holes.
			var common=scope.own(new Model(local.shape.common(face.shape)));
			var filled=scope.own(new Model(common.shape.refined()));
			var segments:Array<CastingSegment> =[];
			for (faceIndex in 0...filled.shape.subshapeCount(CadKit.ShapeKind.Face)) {
				var region=scope.own(new Model(filled.shape.subshape(CadKit.ShapeKind.Face,faceIndex)));
				for (index in 0...region.shape.subshapeCount(CadKit.ShapeKind.Edge)) {
					var edge=scope.own(new Model(region.shape.subshape(CadKit.ShapeKind.Edge,index)));
					var kind=edge.shape.curveKind();
					if (kind!=CadKit.CurveKind.Line && kind!=CadKit.CurveKind.Circle)
						throw "Unsupported section curve (kind "+kind+")";
					var a=Vector.fromNative(edge.shape.positionAt(0)),b=Vector.fromNative(edge.shape.positionAt(1));
					if (Math.abs(a.z)>EPS || Math.abs(b.z)>EPS) throw "Section edge is outside its plane";
					if (kind==CadKit.CurveKind.Line) {
						if (a.subtract(b).length()>EPS) segments.push(P.line({x:a.x,y:a.y},{x:b.x,y:b.y}));
					} else {
						var axis=edge.shape.edgeAxis(),center=Vector.fromNative(axis.get_origin()),radius=axis.get_radius();
						if (Math.abs(Vector.fromNative(axis.get_direction()).z)<1-EPS || radius<=EPS)
							throw "Section circle is outside its plane";
						// Endpoints use increasing curve parameters. Topological edge
						// orientation reverses tangentAt(), but not positionAt().
						var sign=axis.get_direction().get_z()>0 ? 1.0 : -1.0;
						var sweep=sign*edge.shape.edgeLength()/radius;
						if (Math.abs(Math.abs(sweep)-2*Math.PI)<EPS) {
							for (arc in P.circle(center.x,center.y,radius)) segments.push(arc);
						} else segments.push({kind:Arc,start:{x:a.x,y:a.y},end:{x:b.x,y:b.y},
							center:{x:center.x,y:center.y},sweep:sweep});
					}
				}
			}
			scope.close();
			var contours=loops(segments),result:Array<CastingSectionContour> =[];
			for (contour in contours) {
				var depth=0,point=contour[0].start;
				for (other in contours) if (other!=contour && G.inside(other,point.x,point.y)) depth++;
				result.push({contour:contour,depth:depth});
			}
			result.sort((a,b) -> a.depth!=b.depth ? a.depth-b.depth : compare(a.contour[0],b.contour[0]));
			return result;
		} catch (error:Dynamic) {
			scope.close();
			throw "Casting section "+sourceId+": "+Std.string(error);
		}
	}

	static function compare(a:CastingSegment,b:CastingSegment):Int {
		for (pair in [[a.start.x,b.start.x],[a.start.y,b.start.y],[a.end.x,b.end.x],[a.end.y,b.end.y]]) {
			var order=Reflect.compare(pair[0],pair[1]);
			if (order!=0) return order;
		}
		return 0;
	}

	static function reverse(segment:CastingSegment):CastingSegment
		return {kind:segment.kind,start:segment.end,end:segment.start,center:segment.center,sweep:-segment.sweep};

	static function loops(edges:Array<CastingSegment>):Array<Array<CastingSegment>> {
		for (edge in edges) for (point in [edge.start,edge.end]) {
			var degree=0;
			for (other in edges) {
				if (P.distance(point,other.start)<=EPS) degree++;
				if (P.distance(point,other.end)<=EPS) degree++;
			}
			if (degree!=2) throw degree<2 ? "Open section contour" : "Ambiguous section junction";
		}
		edges.sort(compare);
		var result:Array<Array<CastingSegment>> =[];
		while (edges.length>0) {
			var first=edges.shift(),loop=[first],end=first.end;
			while (P.distance(end,first.start)>EPS) {
				var next=-1,backward=false;
				for (index in 0...edges.length) {
					var forward=P.distance(end,edges[index].start)<=EPS;
					var back=P.distance(end,edges[index].end)<=EPS;
					if (forward || back) {
						if (next>=0 || (forward && back)) throw "Ambiguous section junction";
						next=index;backward=back;
					}
				}
				if (next<0) throw "Open section contour";
				var edge=edges.splice(next,1)[0];
				if (backward) edge=reverse(edge);
				loop.push(edge);end=edge.end;
			}
			if (G.area(loop)<0) loop=[for (index in 0...loop.length) reverse(loop[loop.length-1-index])];
			var start=0;
			for (index in 1...loop.length) if (compare(loop[index],loop[start])<0) start=index;
			loop=loop.slice(start).concat(loop.slice(0,start));
			G.validate(loop);
			result.push(loop);
		}
		return result;
	}
}
