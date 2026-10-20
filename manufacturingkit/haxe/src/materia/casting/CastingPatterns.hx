package materia.casting;

import materia.casting.CastingFeature;
import materia.assembly.AssemblyFrames as F;

/** Shared design patterns. Cuts carry design intent; openings are explicit. */
class CastingPatterns {
	public static function box(id:String, kind:CastingFeatureKind, openings:Array<CastingOpening>, width:Float, depth:Float, height:Float, x:Float = 0,
			y:Float = 0, z:Float = 0, ?owner:String, ?adjustable:Array<CastingDimension>):CastingFeature
		return {
			id: id,
			kind: kind,
			geometry: {
				kind: Box,
				width: width,
				depth: depth,
				height: height,
				pose: F.translation(x, y, z)
			},
			openings: openings,
			draft: {degrees: 0, axis: {x: 0, y: 0, z: 1}},
			cornerRadius: 0,
			owner: owner,
			derivedFrom: null,
			adjustable: adjustable == null ? [] : adjustable
		};

	public static function face(target:String):CastingOpening
		return {kind: BodyFace, target: target};

	public static function feature(target:String):CastingOpening
		return {kind: Feature, target: target};

	public static function encode(features:Array<CastingFeature>):String
		return CastingFeatureCodec.encode(features);

	public static function decode(text:String):Array<CastingFeature>
		return CastingFeatureCodec.decode(text);

	public static function grid(width:Float, depth:Float, height:Float, wall:Float, sole:Float, roof:Float, rib:Float, columns:Int = 2,
			rows:Int = 2):Array<CastingFeature> {
		if (columns < 1 || rows < 1 || columns * rows > 256 || wall <= 0 || sole < 0 || roof < 0 || rib <= 0)
			throw "Invalid casting grid";
		var w = (width - 2 * wall - (columns - 1) * rib) / columns,
			d = (depth - 2 * wall - (rows - 1) * rib) / rows,
			h = height - sole - roof;
		if (w <= 0 || d <= 0 || h <= 0)
			throw "Casting cores leave no usable space";
		return [
			for (x in 0...columns)
				for (y in 0...rows)
					box('core-$x-$y', Lightening, sole == 0 ? [face("body.z-min")] : [], w, d, h,
						-width / 2
						+ wall
						+ w / 2
						+ x * (w + rib),
						-depth / 2
						+ wall
						+ d / 2
						+ y * (d + rib), sole, null, [CastingDimension.Width, CastingDimension.Depth, CastingDimension.Height])
		];
	}
}
