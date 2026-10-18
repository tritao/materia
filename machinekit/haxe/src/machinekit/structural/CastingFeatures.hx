package machinekit.structural;

import haxe.Json;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

/** Box features are centred in X/Y with their bottom at local Z zero. */
typedef CastingBox = {
	final id:String;
	final width:Float;
	final depth:Float;
	final height:Float;
	final pose:AssemblyFrame;
}

/** Typed layout helpers and a recipe codec for cores, service openings and insert recesses.
 * These describe design geometry; they do not specify moulds or a casting process.
 */
class CastingFeatures {
	public static function box(id:String, width:Float, depth:Float, height:Float,
			x:Float = 0, y:Float = 0, z:Float = 0):CastingBox
		return {id: id, width: width, depth: depth, height: height, pose: F.translation(x, y, z)};

	public static function encode(features:Array<CastingBox>):String {
		validate(features);
		return Json.stringify(features);
	}

	public static function decode(text:String):Array<CastingBox> {
		var raw:Dynamic = Json.parse(text);
		if (!Std.isOfType(raw, Array)) throw "Casting features must be an array";
		var rows:Array<Dynamic> = cast raw;
		if (rows.length > 256) throw "Too many casting features";
		var result:Array<CastingBox> = [];
		for (row in rows) {
			if (row == null || !Std.isOfType(Reflect.field(row, "id"), String)) throw "Casting feature needs an id";
			var pose = Reflect.field(row, "pose");
			result.push({id: Reflect.field(row, "id"), width: number(row, "width"), depth: number(row, "depth"),
				height: number(row, "height"), pose: {x: number(pose, "x"), y: number(pose, "y"), z: number(pose, "z"),
				qx: number(pose, "qx"), qy: number(pose, "qy"), qz: number(pose, "qz"), qw: number(pose, "qw")}});
		}
		validate(result);
		return result;
	}

	static function number(row:Dynamic, field:String):Float {
		var value:Dynamic = row == null ? null : Reflect.field(row, field);
		if (!(Std.isOfType(value, Float) || Std.isOfType(value, Int))) throw 'Invalid casting feature $field';
		var result:Float = cast value;
		if (!Math.isFinite(result)) throw 'Invalid casting feature $field';
		return result;
	}

	public static function validate(features:Array<CastingBox>):Void {
		if (features == null || features.length > 256) throw "Invalid casting features";
		var ids:Map<String, Bool> = [];
		for (row in features) {
			if (row == null || row.id == null || row.id.length == 0 || ids.exists(row.id) || row.pose == null ||
				!Math.isFinite(row.width + row.depth + row.height) || row.width <= 0 || row.depth <= 0 || row.height <= 0)
				throw "Casting features need unique ids and positive finite dimensions";
			ids.set(row.id, true);
			var p = row.pose;
			if (!Math.isFinite(p.x + p.y + p.z + p.qx + p.qy + p.qz + p.qw) ||
				Math.abs(p.qx*p.qx + p.qy*p.qy + p.qz*p.qz + p.qw*p.qw - 1) > 1e-8)
				throw "Casting feature needs a finite rigid frame";
		}
	}

	/** Regular cores leave outside walls, a sole, a roof and full-height cross ribs. */
	public static function grid(width:Float, depth:Float, height:Float, wall:Float, sole:Float,
			roof:Float, rib:Float, columns:Int = 2, rows:Int = 2):Array<CastingBox> {
		if (!Math.isFinite(width + depth + height + wall + sole + roof + rib) ||
			wall <= 0 || sole <= 0 || roof <= 0 || rib <= 0 || columns < 1 || rows < 1 || columns > 256 || rows > 256 || columns * rows > 256)
			throw "Invalid casting wall, rib or core count";
		var w = (width - 2 * wall - (columns - 1) * rib) / columns;
		var d = (depth - 2 * wall - (rows - 1) * rib) / rows;
		var h = height - sole - roof;
		if (w <= 0 || d <= 0 || h <= 0) throw "Casting cores leave no usable space";
		return [for (x in 0...columns) for (y in 0...rows)
			box('core-$x-$y', w, d, h, -width / 2 + wall + w / 2 + x * (w + rib),
				-depth / 2 + wall + d / 2 + y * (d + rib), sole)];
	}
}
