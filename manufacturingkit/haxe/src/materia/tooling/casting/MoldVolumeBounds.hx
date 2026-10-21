package materia.tooling.casting;
import materia.casting.BoxAlgebra;

import materia.tooling.casting.CastingMoldPlan.MoldVolume;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.casting.CastingProfileGeometry;
import materia.assembly.AssemblyFrames as F;

/** Conservative occupied boxes for loads/broad-phase checks. Curved profiles use
 * their complete bounds and cuts are deliberately not credited as displaced volume.
 * Never use these boxes to assert exact reconstruction or collision.
 */
class MoldVolumeBounds {
	public static function pieces(volume:MoldVolume):Array<MoldBox> {
		var boxes = [for (box in volume.pieces) BoxAlgebra.copy(box)];
		for (profile in volume.profiles) {
			var bounds = CastingProfileGeometry.bounds(profile.contour);
			var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
			var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
			for (x in [bounds.x, bounds.x + bounds.width])
				for (y in [bounds.y, bounds.y + bounds.height])
					for (z in [0.0, profile.depth]) {
						var point = F.transformPoint(profile.pose, x, y, z),
							coordinates = [point.x, point.y, point.z];
						for (axis in 0...3) {
							low[axis] = Math.min(low[axis], coordinates[axis]);
							high[axis] = Math.max(high[axis], coordinates[axis]);
						}
					}
			boxes.push({
				x: low[0],
				y: low[1],
				z: low[2],
				width: high[0] - low[0],
				depth: high[1] - low[1],
				height: high[2] - low[2]
			});
		}
		return BoxAlgebra.subtract(boxes, []);
	}
}
