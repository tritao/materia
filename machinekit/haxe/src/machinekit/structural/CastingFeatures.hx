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

	/** Layout boxes are serialized through the shared neutral casting codec. */
	public static function encode(features:Array<CastingBox>):String {
		validate(features);
		return materia.casting.CastingPatterns.encode([for (row in features) {
			id:row.id, kind:materia.casting.CastingFeature.CastingFeatureKind.Clearance,
			geometry:{kind:materia.casting.CastingFeature.CastingGeometryKind.Box,width:row.width,depth:row.depth,height:row.height,pose:row.pose},
			openings:[],draft:{degrees:0,axis:{x:0,y:0,z:1}},cornerRadius:0,owner:null,adjustable:[],derivedFrom:null
		}]);
	}
	/** Preserve legacy layout cuts while explicitly declaring their exterior openings. */
	public static function encodeForBody(features:Array<CastingBox>,width:Float,depth:Float,height:Float):String {
		// Reservation splits can leave cut fragments wholly outside the body.
        // They have no CAD effect and are not valid finished casting features.
        var active:Array<CastingBox> = [];
        for(row in features) {
            var b=CastingCells.bounds(row);
            if(Math.min(b[3],width/2)>Math.max(b[0],-width/2)+1e-9 &&
               Math.min(b[4],depth/2)>Math.max(b[1],-depth/2)+1e-9 &&
               Math.min(b[5],height)>Math.max(b[2],0)+1e-9)active.push(row);
        }
        var records=materia.casting.CastingPatterns.decode(encode(active));
		var envelope=materia.casting.CastingEnvelope.build(width,depth,height);
		for(row in records)row.openings=materia.casting.CastingCavities.reachedOpenings(envelope,row.geometry);
		return materia.casting.CastingPatterns.encode(records);
	}
	public static function decode(text:String):Array<CastingBox> {
		return [for (row in materia.casting.CastingPatterns.decode(text))
			{id:row.id,width:row.geometry.width,depth:row.geometry.depth,height:row.geometry.height,pose:row.geometry.pose}];
	}
	/** Read only: historical baseline report, never a new component recipe. */
	public static function decodeLegacyBaseline(text:String):Array<CastingBox> {
		var rows:Array<Dynamic> = cast Json.parse(text), result:Array<CastingBox> = [];
		for(row in rows)result.push({id:row.id,width:row.width,depth:row.depth,height:row.height,pose:row.pose});
		validate(result);return result;
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
