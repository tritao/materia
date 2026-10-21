package machinekit.tooling;

import cadkit.Shape;
import cadkit.modeling.Model;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Scope;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;

/** Native geometry backend for the existing critical-level fill analysis.
 * Bounds size the clipping tools only. Every returned region, section and volume
 * is an exact BREP Boolean; neither voxels nor sampled levels are introduced.
 * The caller owns returned parts and closes this backend independently.
 */
class CastingFillGeometry {
	public final sourceId:String;
	final source:Part;
	final low:Vector;
	final high:Vector;
	final padding:Float;
	var closed=false;
	var heightsCache:Null<Array<Float>>;
	static inline var EPS=1e-7;

	public function new(sourceId:String,part:Part) {
		if (part.volume()<=EPS || part.shape.subshapeCount(CadKit.ShapeKind.Solid)==0)
			throw "Fill geometry needs a positive solid: "+sourceId;
		var bounds=part.shape.bounds();
		this.sourceId=sourceId;
		low=Vector.fromNative(bounds.get_min());
		high=Vector.fromNative(bounds.get_max());
		padding=Math.max(1,high.subtract(low).length());
		source=new Part(part.shape.cloneShape());
	}

	function ensureOpen():Void {
		if (closed) throw "Closed fill geometry: "+sourceId;
	}

	/** The complete height extrema of each conic/straight boundary edge, plus
	 * its vertices. Finite planes enclose the whole XY projection, so exact
	 * shape distance measures height rather than a distance to a plane edge.
	 * Higher-order boundary curves need a stationary-point solver; never use
	 * their bounds as a claimed critical level.
	 */
	public function criticalHeights():Array<Float> {
		ensureOpen();
		if (heightsCache!=null) return heightsCache.copy();
		var scope=new Scope();
		try {
			var lower=low.z-padding,upper=high.z+padding;
			var bottom=scope.own(plane(lower)),top=scope.own(plane(upper));
			var heights:Array<Float> = [];
			for (index in 0...source.shape.subshapeCount(CadKit.ShapeKind.Vertex)) {
				var vertex=scope.own(new Model(source.shape.subshape(CadKit.ShapeKind.Vertex,index)));
				heights.push(vertex.shape.position().get_z());
			}
			for (index in 0...source.shape.subshapeCount(CadKit.ShapeKind.Edge)) {
				var edge=scope.own(new Model(source.shape.subshape(CadKit.ShapeKind.Edge,index)));
				var kind=edge.shape.curveKind();
				if (kind!=CadKit.CurveKind.Line && kind!=CadKit.CurveKind.Circle
					&& kind!=CadKit.CurveKind.Ellipse && kind!=CadKit.CurveKind.Parabola
					&& kind!=CadKit.CurveKind.Hyperbola)
					throw "Unsupported fill critical-height curve: "+sourceId;
				heights.push(lower+edge.shape.distance(bottom.shape));
				heights.push(upper-edge.shape.distance(top.shape));
			}
			heights.sort(Reflect.compare);
			var result:Array<Float> = [];
			for (height in heights)
				if (result.length==0 || height-result[result.length-1]>EPS) result.push(height);
			scope.close();
			heightsCache=result.copy();
			return result;
		} catch (error:Dynamic) {scope.close();throw error;}
	}

	function plane(height:Float):Sketch {
		return Sketch.rectangle(high.x-low.x+2*padding,high.y-low.y+2*padding,
			new Plane(new Vector((low.x+high.x)/2,(low.y+high.y)/2,height),Vector.X(),Vector.Z()));
	}

	function clipped(minZ:Float,maxZ:Float):Part {
		ensureOpen();
		if (!Math.isFinite(minZ+maxZ) || maxZ<=minZ) throw "Invalid fill clipping interval: "+sourceId;
		return Scope.run(scope -> {
			var local=scope.own(Part.box(high.x-low.x+2*padding,high.y-low.y+2*padding,maxZ-minZ));
			var tool=scope.own(local.translated(new Vector((low.x+high.x)/2,(low.y+high.y)/2,minZ)));
			var common=scope.own(new Model(source.shape.common(tool.shape)));
			// At an exact event plane the common may also contain a cap face or
			// edge. Only solids are air/resin volume; retain no lower-dimensional
			// contact as a connection or as a stand-in solid.
			var solids:Array<Shape> = [];
			for (index in 0...common.shape.subshapeCount(CadKit.ShapeKind.Solid))
				solids.push(scope.own(new Model(common.shape.subshape(CadKit.ShapeKind.Solid,index))).shape);
			return scope.own(new Part(Shape.compound(solids)));
		});
	}

	public function below(height:Float):Part {
		ensureOpen();
		if (!Math.isFinite(height)) throw "Invalid fill height: "+sourceId;
		if (height<=low.z-padding) return new Part(Shape.compound([]));
		if (height>=high.z+padding) return new Part(source.shape.cloneShape());
		return clipped(low.z-padding,height);
	}

	public function above(height:Float):Part {
		ensureOpen();
		if (!Math.isFinite(height)) throw "Invalid fill height: "+sourceId;
		if (height>=high.z+padding) return new Part(Shape.compound([]));
		if (height<=low.z-padding) return new Part(source.shape.cloneShape());
		return clipped(height,high.z+padding);
	}

	public function section(height:Float):Model {
		ensureOpen();
		if (!Math.isFinite(height)) throw "Invalid fill section height: "+sourceId;
		return new Model(source.shape.section(new Vector(0,0,height).native(),Vector.Z().native()));
	}

	/** Boolean solids preserve positive-area connectivity. A mere point or edge
	 * at an event plane cannot join two air volumes into one solid.
	 */
	public function airComponents(height:Float):Array<Part> {
		var region=above(height),parts:Array<Part> = [];
		try {
			for (index in 0...region.shape.subshapeCount(CadKit.ShapeKind.Solid)) {
				var part=new Part(region.shape.subshape(CadKit.ShapeKind.Solid,index));
				if (part.volume()>EPS) parts.push(part); else part.close();
			}
			region.close();
			parts.sort((a,b) -> {
				var x=a.massProperties(),y=b.massProperties();
				for (difference in [x.centerOfMass.x-y.centerOfMass.x,x.centerOfMass.y-y.centerOfMass.y,
					x.centerOfMass.z-y.centerOfMass.z,x.volume-y.volume])
					if (Math.abs(difference)>EPS) return difference<0 ? -1 : 1;
				return 0;
			});
			return parts;
		} catch (error:Dynamic) {
			if (!region.shape.isClosed()) region.close();
			for (part in parts) part.close();
			throw error;
		}
	}

	/** Geometry identity within one fingerprinted source/level. A duplicate key
	 * among disconnected components is rejected by the fill analysis rather
	 * than resolved by native topology order. Rounding is for identity only.
	 */
	public static function componentKey(part:Part):String {
		var p=part.massProperties(),c=p.centerOfMass,t=p.inertia;
		var values=[p.volume,p.surfaceArea,c.x,c.y,c.z,t.xx,t.xy,t.xz,t.yy,t.yz,t.zz];
		var quantized=[for (value in values) Math.fround(value*1e6)/1e6];
		var vertices:Array<Array<Float>> = [];
		for (index in 0...part.shape.subshapeCount(CadKit.ShapeKind.Vertex)) {
			var vertex=part.shape.subshape(CadKit.ShapeKind.Vertex,index);
			try {
				var point=vertex.position();
				vertices.push([for (value in [point.get_x(),point.get_y(),point.get_z()]) Math.fround(value*1e6)/1e6]);
				vertex.close();
			} catch (error:Dynamic) {vertex.close();throw error;}
		}
		vertices.sort((a,b) -> Reflect.compare(materia.tooling.ToolingCodec.canonical(a),materia.tooling.ToolingCodec.canonical(b)));
		// No bounding box enters identity: native bounds may include triangulation
		// tolerances and are reserved for sizing Boolean tools.
		return "region-"+haxe.crypto.Sha256.encode(materia.tooling.ToolingCodec.canonical({moments:quantized,vertices:vertices})).substr(0,24);
	}

	public function highestPoint():Vector {
		ensureOpen();
		var heights=criticalHeights(),height=heights[heights.length-1];
		var scope=new Scope();
		try {
			var cap=scope.own(plane(height));
			var common=scope.own(new Model(source.shape.common(cap.shape)));
			var points:Array<Vector> = [];
			for (index in 0...common.shape.subshapeCount(CadKit.ShapeKind.Vertex)) {
				var vertex=scope.own(new Model(common.shape.subshape(CadKit.ShapeKind.Vertex,index)));
				points.push(Vector.fromNative(vertex.shape.position()));
			}
			points.sort((a,b) -> a.x!=b.x ? Reflect.compare(a.x,b.x) : Reflect.compare(a.y,b.y));
			if (points.length==0 || Math.abs(points[0].z-height)>1e-5)
				throw "Unsupported exact fill ceiling point: "+sourceId;
			var result=points[0];scope.close();return result;
		} catch (error:Dynamic) {scope.close();throw error;}
	}

	/** Project all tooling at/above the fill plane, including a contacting face.
	 * The existing exact swept-volume construction supplies the projection.
	 */
	public function projectedOpening(height:Float):Model {
		ensureOpen();
		return Scope.run(scope -> {
			var contact=scope.own(section(height)),upper=scope.own(above(height));
			if (upper.volume()<=EPS) return contact;
			var sweep=scope.own(SweptRemoval.sweep(upper,new Vector(0,0,-(high.z-height+padding))));
			var projected=scope.own(new Model(sweep.shape.section(new Vector(0,0,height).native(),Vector.Z().native())));
			return scope.own(new Model(projected.shape.fuse(contact.shape)));
		});
	}

	/** Conservative projected widths of potential ceiling patches. Refinement
	 * joins same-domain faces before measuring, so Boolean seams cannot turn a
	 * broad roof into apparently narrow strips. A non-axial curved face uses
	 * its full projected width; it may overestimate its upward-facing portion.
	 */
	public function ceilingWidths(opening:Model):Array<{widthMm:Float,face:Model}> {
		ensureOpen();
		var refined=source.refined(),result:Array<{widthMm:Float,face:Model}> = [];
		try {
			for (index in 0...refined.shape.subshapeCount(CadKit.ShapeKind.Face)) {
				var face=new Model(refined.shape.subshape(CadKit.ShapeKind.Face,index));
				try {
					var kind=face.shape.surfaceKind(),possible=false;
					if (kind==CadKit.SurfaceKind.Plane) possible=face.shape.faceNormal().get_z()>1e-10;
					else if (kind==CadKit.SurfaceKind.Cylinder) {
						var axis=Vector.fromNative(face.shape.faceAxis().get_direction()).normalized();
						possible=Math.abs(axis.z)<1-1e-10;
					} else if (kind==CadKit.SurfaceKind.Cone) {
						var axis=Vector.fromNative(face.shape.faceAxis().get_direction()).normalized();
						possible=Math.abs(axis.z)<1-1e-10 || face.shape.faceNormal().get_z()>1e-10;
					} else throw "Unsupported fill ceiling surface: "+sourceId;
					if (!possible) {face.close();continue;}
					var remaining=new Model(face.shape.cut(opening.shape));face.close();face=remaining;
					if (face.shape.area()<=EPS) {face.close();continue;}
					var bounds=face.shape.bounds(),a=bounds.get_min(),b=bounds.get_max();
					result.push({widthMm:Math.min(b.get_x()-a.get_x(),b.get_y()-a.get_y()),face:face});
				} catch (error:Dynamic) {if (!face.shape.isClosed()) face.close();throw error;}
			}
			refined.close();return result;
		} catch (error:Dynamic) {
			if (!refined.shape.isClosed()) refined.close();
			for (item in result) item.face.close();
			throw error;
		}
	}

	public function close():Void {
		if (!closed) {closed=true;source.close();}
	}
}
