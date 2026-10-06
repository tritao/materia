package robotkit.manipulation;

import robotkit.spatial.Quat;
import robotkit.spatial.Vec3;

/** A planar face from the part's local CAD frame. */
typedef GraspPlane = {var origin:Vec3; var normal:Vec3;}

/** A face-derived parallel-jaw grasp, in the part's local frame. */
typedef GraspFrame = {
	var contact:Vec3;
	var widthAxis:Vec3;
	/** Outward from the part, toward the robot on the approach side. */
	var approachAxis:Vec3;
	var widthMm:Float;
	/** Tool TCP orientation relative to the part: jaw axis on +X, fingers point along +Z. */
	var toolRotation:Quat;
}

/** Selects a width-compatible pinch between opposite planar faces. */
class Grasp {
	/**
	 * Pick the opposing face pair nearest `nominalWidthMm`, with the approach direction perpendicular to
	 * the pinch axis. The part's collision hull supplies actual extents and fingertip contact at the
	 * far-side support plane after insertion; a grasp that does not fit the jaw range or finger length is refused.
	 */
	public static function find(faces:Array<GraspPlane>, hull:Array<Vec3>, approach:Vec3,
			minimumWidthMm:Float, nominalWidthMm:Float, maximumWidthMm:Float,
			fingerLengthMm:Float, ?preferredWidthAxis:Vec3):GraspFrame {
		if (faces == null || hull == null || hull.length < 4 || approach == null ||
			!(minimumWidthMm > 0) || !(minimumWidthMm <= nominalWidthMm) ||
			!(nominalWidthMm <= maximumWidthMm) || !(fingerLengthMm > 0) ||
			!Math.isFinite(minimumWidthMm + nominalWidthMm + maximumWidthMm + fingerLengthMm))
			throw "Grasp search needs planar faces, a hull, and valid gripper limits";
		var normal = approach.normalized();
		var preferred = preferredWidthAxis == null ? null : preferredWidthAxis.normalized();
		var selected:Null<GraspFrame> = null;
		var selectedScore = Math.POSITIVE_INFINITY;
		var selectedAlignment = -1.0;
		for (first in 0...faces.length) for (second in first + 1...faces.length) {
			var a = faces[first], b = faces[second];
			if (a == null || b == null || a.origin == null || b.origin == null ||
				a.normal == null || b.normal == null) continue;
			var firstNormal:Vec3, secondNormal:Vec3;
			try {
				firstNormal = a.normal.normalized();
				secondNormal = b.normal.normalized();
			} catch (_:Dynamic) continue;
			if (firstNormal.dot(secondNormal) > -0.999 || Math.abs(firstNormal.dot(normal)) > 0.05) continue;
			var axis = canonical(firstNormal);
			var depth = normal.cross(axis).normalized();
			var minWidth = Math.POSITIVE_INFINITY, maxWidth = Math.NEGATIVE_INFINITY;
			var minDepth = Math.POSITIVE_INFINITY, maxDepth = Math.NEGATIVE_INFINITY;
			var minApproach = Math.POSITIVE_INFINITY, maxApproach = Math.NEGATIVE_INFINITY;
			var validHull = true;
			for (point in hull) {
				if (point == null) { validHull = false; break; }
				var width = point.dot(axis), depthValue = point.dot(depth), height = point.dot(normal);
				minWidth = Math.min(minWidth, width); maxWidth = Math.max(maxWidth, width);
				minDepth = Math.min(minDepth, depthValue); maxDepth = Math.max(maxDepth, depthValue);
				minApproach = Math.min(minApproach, height); maxApproach = Math.max(maxApproach, height);
			}
			if (!validHull) continue;
			// The selected planar faces must be the hull's opposing support faces;
			// an internal coplanar feature must not masquerade as a jaw contact.
			var firstPlane = a.origin.dot(axis), secondPlane = b.origin.dot(axis);
			var planeToleranceMm = 0.05;
			var supportsWidth = (Math.abs(firstPlane - minWidth) <= planeToleranceMm &&
				Math.abs(secondPlane - maxWidth) <= planeToleranceMm) ||
				(Math.abs(secondPlane - minWidth) <= planeToleranceMm &&
				Math.abs(firstPlane - maxWidth) <= planeToleranceMm);
			if (!supportsWidth) continue;
			var widthMm = maxWidth - minWidth, fingerReach = maxApproach - minApproach;
			if (widthMm < minimumWidthMm - 1e-6 || widthMm > maximumWidthMm + 1e-6 ||
				fingerReach > fingerLengthMm + 1e-6) continue;
			var score = Math.abs(widthMm - nominalWidthMm);
			var alignment = preferred == null ? 0.0 : Math.abs(axis.dot(preferred));
			if (score > selectedScore + 1e-6 ||
				(Math.abs(score - selectedScore) <= 1e-6 && alignment <= selectedAlignment)) continue;
			var contact = axis.scale((minWidth + maxWidth) / 2)
				.add(depth.scale((minDepth + maxDepth) / 2))
				.add(normal.scale(minApproach));
			// The TCP sits at the finger tips. Local +Z runs from the flange through
			// the fingers into the part, opposite the outward approach direction.
			var toolZ = normal.negate();
			var toolY = toolZ.cross(axis).normalized();
			// Quat stores columns. Preserve the tool's X, Y and Z basis as columns.
			var rotation = Quat.fromRotationMatrix([
				axis.x, axis.y, axis.z,
				toolY.x, toolY.y, toolY.z,
				toolZ.x, toolZ.y, toolZ.z
			]);
			selected = {contact: contact, widthAxis: axis, approachAxis: normal,
				widthMm: widthMm, toolRotation: rotation};
			selectedScore = score;
			selectedAlignment = alignment;
		}
		if (selected == null)
			throw 'No opposing planar grasp fits ${minimumWidthMm}–${maximumWidthMm} mm jaws and ${fingerLengthMm} mm fingers';
		return selected;
	}

	static function canonical(axis:Vec3):Vec3 {
		var value = axis.normalized();
		var sign = Math.abs(value.x) > 1e-9 ? value.x : Math.abs(value.y) > 1e-9 ? value.y : value.z;
		return sign < 0 ? value.negate() : value;
	}
}
