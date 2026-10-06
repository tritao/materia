package machinekit.pressbrake.planning;

import cadkit.modeling.Location;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.BendZone;
import machinekit.pressbrake.ToolContactPrism;

/** Exact coordinate extrema can certify separation even when extrusion axes differ. */
class PressBendToolBounds {
	public static function separated(zone:BendZone,placement:Location,tool:ToolContactPrism):Bool {
		for (axis in 0...3) {
			var stock = projection(zone,placement,axis);
			var low = axis == 0 ? tool.start : Math.POSITIVE_INFINITY;
			var high = axis == 0 ? tool.end : Math.NEGATIVE_INFINITY;
			if (axis != 0) for (loop in tool.section.loops()) for (segment in loop) for (point in segment.extrema()) {
				var value = axis == 1 ? point.x : point.y;
				low = Math.min(low,value); high = Math.max(high,value);
			}
			if (stock[1] <= low+1e-8 || high <= stock[0]+1e-8) return true;
		}
		return false;
	}
	/** Radial and extrusion coordinates are affine; extrema lie at their endpoints. */
	public static function projection(zone:BendZone,placement:Location,axis:Int):Array<Float> {
		return projectionAlong(zone,placement,axis == 0 ? new Vector(1,0,0) : axis == 1 ? new Vector(0,1,0) : new Vector(0,0,1));
	}
	public static function projectionAlong(zone:BendZone,placement:Location,axis:Vector):Array<Float> {
		var fractions = [0.0,1.0], angle = Math.abs(zone.angle);
		if (angle > 0) {
			var frame = placement.compose(zone.frame);
			var x = frame.direction(new Vector(1,0,0)).dot(axis);
			var z = frame.direction(new Vector(0,0,1)).dot(axis)*(zone.angle > 0 ? 1 : -1);
			var event = Math.atan2(-x,z);
			while (event < 0) event += Math.PI;
			while (event <= angle) { fractions.push(event/angle); event += Math.PI; }
		}
		var low = Math.POSITIVE_INFINITY, high = Math.NEGATIVE_INFINITY;
		for (fraction in fractions) for (along in [0.0,zone.width]) for (depth in [0.0,zone.thickness]) {
			var value = placement.plane.toWorld(zone.point(fraction,along,depth)).dot(axis);
			low = Math.min(low,value); high = Math.max(high,value);
		}
		return [low,high];
	}
}
