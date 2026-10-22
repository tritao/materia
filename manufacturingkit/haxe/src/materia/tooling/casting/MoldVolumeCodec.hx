package materia.tooling.casting;
import materia.casting.BoxAlgebra;

import materia.tooling.ToolingCodec;
import materia.tooling.panel.PanelCodec;
import materia.tooling.casting.CastingMoldPlan.MoldVolume;
import materia.tooling.casting.CastingMoldPlan.MoldPartMaterial;
import materia.tooling.casting.CastingMoldPlan.MoldVolumeProfile;
import materia.tooling.casting.CastingMoldPlan.MoldVolumeCut;
import materia.tooling.casting.MoldPlan.MoldBox;

/** Exact boxes and operation-derived sealing profiles; no tessellation or compensation. */
class MoldVolumeCodec {
	public static function encode(v:MoldVolume):String {
		var s = ToolingCodec.canonical({format: "materia.mold-volume", version: 4, volume: snapshot(v)});
		if (haxe.io.Bytes.ofString(s).length > 524288)
			throw "Mold volume exceeds limit";
		return s;
	}

	public static function decode(s:String):MoldVolume {
		if (s == null || haxe.io.Bytes.ofString(s).length > 524288)
			throw "Mold volume exceeds limit";
		var r:Dynamic = haxe.Json.parse(s);
		object(r, ["format", "version", "volume"]);
		if (Reflect.field(r, "format") != "materia.mold-volume" || Reflect.field(r, "version") != 4)
			throw "Unsupported mold volume format";
		return read(Reflect.field(r, "volume"));
	}

	public static function snapshot(v:MoldVolume):MoldVolume
		return read(v);

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
			profiles.push({
				panelId: text(p, "panelId"),
				operationId: text(p, "operationId"),
				contour: PanelCodec.contourSnapshot(Reflect.field(p, "contour")),
				pose: pose,
				depth: depth,
				subtract: boxes(array(p, "subtract", 2048))
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
