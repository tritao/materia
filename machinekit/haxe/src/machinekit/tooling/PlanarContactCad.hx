package machinekit.tooling;

import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;

enum PlanarCoverageAreas {
	NotEvaluated;
	Evaluated(targetMm2:Float,uncoveredMm2:Float);
}

typedef PlanarCoverage = {
	var status:ToolingCheckStatus;
	var message:String;
	var areas:PlanarCoverageAreas;
}

/** Exact nominal planar face coverage, with the real boundaries and holes.
 * A volume section through an interior plane is not a boundary contact.
 * Physical hardware, alignment evidence and replication chemistry stay separate.
 */
class PlanarContactCad {
	// Numerical coincidence only; neither is a manufacturing accuracy allowance.
	static inline var FRAME_MM = 1e-8;
	static inline var FRAME_DIRECTION = 1e-10;

	public static function coverage(support:Part,supportFrame:AssemblyFrame,target:Part,targetFrame:AssemblyFrame):PlanarCoverage {
		materia.assembly.AssemblyCodec.validateFrame(supportFrame);
		materia.assembly.AssemblyCodec.validateFrame(targetFrame);
		var shapes:Array<Shape> = [];
		function own(shape:Shape):Shape {shapes.push(shape);return shape;}
		function finish(status:ToolingCheckStatus,message:String,areas:PlanarCoverageAreas):PlanarCoverage {
			for (shape in shapes) shape.close();
			return {status:status,message:message,areas:areas};
		}
		try {
			var actual=boundary(support,supportFrame,shapes),wanted=boundary(target,targetFrame,shapes);
			if (actual==null) return finish(ToolingCheckStatus.Fail,"Supporting reference is not an actual planar stock boundary.",NotEvaluated);
			if (wanted==null) return finish(ToolingCheckStatus.Fail,"Source interface is not an actual realized planar boundary.",NotEvaluated);
			var normal=axis(supportFrame),other=axis(targetFrame);
			var delta=new Vector(targetFrame.x-supportFrame.x,targetFrame.y-supportFrame.y,targetFrame.z-supportFrame.z);
			if (normal.cross(other).length()>FRAME_DIRECTION || Math.abs(delta.dot(normal))>FRAME_MM)
				return finish(ToolingCheckStatus.Unsupported,
					"Exact coverage requires coincident nominal planes; tolerance-aware projection or a varying layer is not implemented.",NotEvaluated);
			var overlap=own(support.shape.common(target.shape));
			if (overlap.subshapeCount(CadKit.ShapeKind.Solid)>0)
				return finish(ToolingCheckStatus.Fail,"Supporting stock overlaps the target material instead of touching its boundary.",NotEvaluated);
			var missing=own(wanted.planarCut(actual));
			var area=wanted.area(),uncovered=missing.subshapeCount(CadKit.ShapeKind.Face)>0 ? missing.area() : 0;
			return missing.subshapeCount(CadKit.ShapeKind.Face)>0
				? finish(ToolingCheckStatus.Fail,"Support leaves required contact area uncovered: "+uncovered+" mm2.",Evaluated(area,uncovered))
				: finish(ToolingCheckStatus.Pass,"Exact planar booleans cover the source boundary, including its holes; physical qualification is separate.",Evaluated(area,0));
		} catch (error:Dynamic) {
			for (shape in shapes) shape.close();
			throw error;
		}
	}

	static function boundary(part:Part,frame:AssemblyFrame,owned:Array<Shape>):Null<Shape> {
		var normal=axis(frame),origin=new Vector(frame.x,frame.y,frame.z),result:Null<Shape> = null;
		for (index in 0...part.shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face=part.shape.subshape(CadKit.ShapeKind.Face,index);owned.push(face);
			if (face.surfaceKind()!=CadKit.SurfaceKind.Plane) continue;
			var faceNormal=Vector.fromNative(face.faceNormal()).normalized();
			if (faceNormal.cross(normal).length()>FRAME_DIRECTION
				|| Math.abs(Vector.fromNative(face.center()).subtract(origin).dot(normal))>FRAME_MM) continue;
			if (result==null) result=face;
			else {result=result.planarFuse(face);owned.push(result);}
		}
		return result;
	}

	static function axis(frame:AssemblyFrame):Vector {
		var n=F.transformVector(frame,0,0,1);return new Vector(n.x,n.y,n.z).normalized();
	}

}
