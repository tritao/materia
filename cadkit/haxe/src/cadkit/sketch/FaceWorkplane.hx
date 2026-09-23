package cadkit.sketch;

import CadKit;
import cadkit.Shape;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.ParametricError;
import cadkit.parametric.SelectionRecipe;

/** Derives an oriented workplane from exactly one selected planar face. */
class FaceWorkplane {
	public static function resolve(shape:Shape, selection:SelectionRecipe, xDirection:Vector, offset:Float = 0,
		flip:Bool = false):Plane {
		if (selection.kind != "face" || selection.geometry != "plane" || selection.expectedCount != 1)
			throw new ParametricError("an attached sketch requires a selection for exactly one planar face");
		if (!Math.isFinite(offset))
			throw new ParametricError("attached sketch offset must be finite");
		var matches = selection.resolve(shape);
		var face = matches[0];
		try {
			var result = fromFace(face, xDirection, offset, flip);
			face.close();
			return result;
		} catch (error:Dynamic) {
			face.close();
			throw error;
		}
	}

	/** Derive a workplane from one already-resolved face without taking its ownership. */
	public static function fromFace(face:Shape, xDirection:Vector, offset:Float = 0,
		flip:Bool = false):Plane {
		if (face.kind() != CadKit.ShapeKind.Face || face.surfaceKind() != CadKit.SurfaceKind.Plane)
			throw new ParametricError("attached sketch support must be a planar face");
		if (!Math.isFinite(offset))
			throw new ParametricError("attached sketch offset must be finite");
		var normal = Vector.fromNative(face.faceNormal());
		if (flip)
			normal = normal.scale(-1);
		var projectedX = xDirection.subtract(normal.scale(xDirection.dot(normal)));
		if (projectedX.length() < 1e-10)
			throw new ParametricError("attached sketch X direction must not be parallel to the face normal");
		var origin = Vector.fromNative(face.center()).add(normal.scale(offset));
		return new Plane(origin, projectedX, normal);
	}
}
