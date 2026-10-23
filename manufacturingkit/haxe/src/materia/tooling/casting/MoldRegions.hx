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
import materia.tooling.ToolingCodec;
import materia.tooling.panel.PanelCodec;
import materia.tooling.casting.CastingMoldPlan.MoldVolume;
import materia.tooling.casting.CastingMoldPlan.MoldPartMaterial;
import materia.tooling.casting.CastingMoldPlan.MoldVolumeProfile;
import materia.tooling.casting.CastingMoldPlan.MoldVolumeCut;
import materia.tooling.casting.CastingMoldPlan.MoldBox;

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

	/** Single finished tooling expression from the same operation profiles used
	 * by panel CAD, drawings and CAM. Exact cuts are never replaced by bounds.
	 */
	public static function tooling(volume:CastingMoldPlan.MoldVolume):CastingRegion {
		var v=MoldVolumeRecords.copy(volume);
		var terms=[for (piece in v.pieces) box(piece)].concat(v.regions);
		for (part in v.profiles) terms.push(CastingRegion.Difference(profile(part),part.subtract));
		var body=CastingRegion.Union(terms);
		return CastingRegion.Difference(body,[for (cut in v.cuts) CastingRegion.Placed({
			solid:CastingGeometrySolid.Prism(materia.tooling.panel.PanelContours.operation(cut.operation),cut.operation.depth),pose:cut.pose})]);
	}

	public static function profile(part:CastingMoldPlan.MoldVolumeProfile):CastingRegion
		return CastingRegion.Placed({solid:CastingGeometrySolid.Prism(part.contour,part.depth),pose:part.pose});

	public static function section(contours:Array<materia.casting.CastingSection.CastingSectionContour>,
			pose:AssemblyFrame,depth:Float):CastingRegion {
		function prism(contour:Array<materia.casting.CastingProfile.CastingSegment>):CastingRegion
			return Placed({solid:Prism(contour,depth),pose:pose});
		var pieces:Array<CastingRegion>=[];
		for (outer in contours) if (outer.depth%2==0) {
			var holes:Array<CastingRegion>=[];
			for (hole in contours) if (hole.depth==outer.depth+1
				&& materia.casting.CastingProfileGeometry.inside(outer.contour,hole.contour[0].start.x,hole.contour[0].start.y))
				holes.push(prism(hole.contour));
			pieces.push(Difference(prism(outer.contour),holes));
		}
		return Union(pieces);
	}

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
		return orthogonal(CastingRegionCodec.copy(region));

	public static function exactBounds(region:CastingRegion,id:String):Box3 {
		var pieces=boxes(region);
		if (pieces==null) {
			if (CastingMoldPlanner.geometry==null) throw "Exact manufacturing bounds kernel required: "+id;
			return CastingMoldPlanner.geometry.bounds(region,id);
		}
		if (pieces.length==0) throw "Empty tooling manufacturing volume: "+id;
		var low=[Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY];
		var high=[Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY];
		for (piece in pieces) {
			var a=[piece.x,piece.y,piece.z],b=[piece.x+piece.width,piece.y+piece.depth,piece.z+piece.height];
			for (axis in 0...3) {low[axis]=Math.min(low[axis],a[axis]);high[axis]=Math.max(high[axis],b[axis]);}
		}
		return {x:low[0],y:low[1],z:low[2],width:high[0]-low[0],depth:high[1]-low[1],height:high[2]-low[2]};
	}

	public static function moments(region:CastingRegion,affectedId:String):CastingRegionMoments {
		var normalized=CastingRegionCodec.copy(region),pieces=orthogonal(normalized);
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

class MoldVolumeRecords {
	/** Plain geometry records belong to the delivered fixture recipe, not a
	 * separately versioned intermediate volume document. */
	static function raw(volume:MoldVolume):Dynamic {
		var text=ToolingCodec.canonical(volume);
		if (haxe.io.Bytes.ofString(text).length>524288) throw "Mold volume exceeds limit";
		return haxe.Json.parse(text);
	}
	public static function copy(volume:MoldVolume):MoldVolume return snapshot(raw(volume));
	public static function record(volume:MoldVolume):Dynamic return raw(copy(volume));
	public static function snapshot(raw:Dynamic):MoldVolume return read(raw);

	static function object(r:Dynamic, keys:Array<String>):Void {
		if (r == null || !Reflect.isObject(r) || Std.isOfType(r, Array) || Std.isOfType(r, String))
			throw "Invalid mold volume object";
		for (k in Reflect.fields(r))
			if (keys.indexOf(k) < 0)
				throw "Unknown mold volume field: " + k;
		for (k in keys)
			if (!Reflect.hasField(r, k))
				throw "Missing mold volume field: " + k;
	}

	static function number(r:Dynamic, k:String):Float {
		var v:Dynamic = Reflect.field(r, k);
		if ((!Std.isOfType(v, Int) && !Std.isOfType(v, Float)) || !Math.isFinite(v) || Math.abs(v) > 1000000)
			throw "Invalid mold volume number";
		return v;
	}

	static function text(r:Dynamic, k:String):String {
		var v:Dynamic = Reflect.field(r, k);
		if (!Std.isOfType(v, String))
			throw "Invalid mold volume text";
		var result:String = cast v;
		PanelCodec.identifier(result);
		return result;
	}

	static function array(r:Dynamic, k:String, limit:Int):Array<Dynamic> {
		var v:Dynamic = Reflect.field(r, k);
		if (!Std.isOfType(v, Array))
			throw "Invalid mold volume array";
		var result:Array<Dynamic> = cast v;
		if (result.length > limit)
			throw "Mold volume count exceeded";
		return result;
	}

	static function boxes(raw:Array<Dynamic>):Array<MoldBox> {
		return [
			for (b in raw) {
				object(b, ["x", "y", "z", "width", "depth", "height"]);
				var box:MoldBox = {
					x: number(b, "x"),
					y: number(b, "y"),
					z: number(b, "z"),
					width: number(b, "width"),
					depth: number(b, "depth"),
					height: number(b, "height")
				};
				if (box.width <= 0 || box.depth <= 0 || box.height <= 0) throw "Invalid mold box dimensions";
				box;
			}
		];
	}

	static function frame(raw:Dynamic):materia.assembly.AssemblyRecord.AssemblyFrame {
		object(raw, ["x", "y", "z", "qx", "qy", "qz", "qw"]);
		var pose:materia.assembly.AssemblyRecord.AssemblyFrame = {
			x: number(raw, "x"),
			y: number(raw, "y"),
			z: number(raw, "z"),
			qx: number(raw, "qx"),
			qy: number(raw, "qy"),
			qz: number(raw, "qz"),
			qw: number(raw, "qw")
		};
		materia.assembly.AssemblyCodec.validateFrame(pose);
		if (Math.abs(pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw - 1) > 1e-8)
			throw "Mold profile needs unit frame";
		return pose;
	}

	static function read(r:Dynamic):MoldVolume {
		object(r, ["id", "material", "pieces", "regions", "profiles", "cuts"]);
		var material:MoldPartMaterial = switch text(r, "material") {
			case "eps": EPSStock;
			case "steel": SteelStock;
			case "former": FormerStock;
			default: throw "Invalid mold volume material";
		};
		var pieces = boxes(array(r, "pieces", 2048)),
			profiles:Array<MoldVolumeProfile> = [];
		for (i in 0...pieces.length)
			for (j in i + 1...pieces.length)
				if (BoxAlgebra.intersection(pieces[i], pieces[j]) != null)
					throw "Mold volume pieces overlap";
		pieces.sort((a, b) -> {
			for (key in ["x", "y", "z", "width", "depth", "height"]) {
				var compare = Reflect.compare(Reflect.field(a, key), Reflect.field(b, key));
				if (compare != 0)
					return compare;
			}
			return 0;
		});
		var cuts:Array<MoldVolumeCut> = [], cutIds = new Map<String, Bool>();
		for (c in array(r, "cuts", 128)) {
			object(c, ["operation", "pose"]);
			var operation = PanelCodec.operationSnapshot(Reflect.field(c, "operation")),
				id = operation.id;
			if (cutIds.exists(id)
				|| operation.accessVia != null
				|| (operation.kind != materia.tooling.panel.PanelPlan.PanelOperationKind.Drill
					&& operation.kind != materia.tooling.panel.PanelPlan.PanelOperationKind.Pocket))
				throw "Invalid/duplicate mold cut";
			cutIds.set(id, true);
			cuts.push({operation: operation, pose: frame(Reflect.field(c, "pose"))});
		}
		cuts.sort((a, b) -> Reflect.compare(a.operation.id, b.operation.id));

		for (p in array(r, "profiles", 32)) {
			object(p, ["panelId", "operationId", "contour", "pose", "depth", "subtract"]);
			var pose = frame(Reflect.field(p, "pose"));
			var depth = number(p, "depth");
			if (depth <= 0)
				throw "Invalid mold profile depth";
			var subtract=[for (region in array(p,"subtract",128)) materia.casting.CastingRegion.CastingRegionCodec.snapshot(region)];
			subtract.sort((a,b) -> Reflect.compare(ToolingCodec.canonical(a),ToolingCodec.canonical(b)));
			profiles.push({
				panelId: text(p, "panelId"),
				operationId: text(p, "operationId"),
				contour: PanelCodec.contourSnapshot(Reflect.field(p, "contour")),
				pose: pose,
				depth: depth,
				subtract: subtract
			});
		}
		var regions=[for (region in array(r,"regions",128)) materia.casting.CastingRegion.CastingRegionCodec.snapshot(region)];
		regions.sort((a,b) -> Reflect.compare(ToolingCodec.canonical(a),ToolingCodec.canonical(b)));
		if (pieces.length == 0 && profiles.length == 0 && regions.length == 0)
			throw "Invalid mold volume pieces";
		return {
			id: text(r, "id"),
			material: material,
			pieces: pieces,
			regions: regions,
			profiles: profiles,
			cuts: cuts
		};
	}
}
