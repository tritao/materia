package app;

import haxe.io.Bytes;
import nativekit.scene.GeometryData;
import toolpathkit.tool.CutterProfile;
import toolpathkit.tool.CutterSegment;

/** A tool's shape as scene geometry: its cutter profile revolved about the tool axis. */
class CutterGeometry {
	static inline final SIDES = 32;
	/** Pieces each profile arc is drawn with. */
	static inline final ARC_STEPS = 8;
	/** Steel grey, as RGBA bytes in vertex order. */
	static final STEEL = [0x9E, 0xA3, 0xA8, 0xFF];

	/**
	 * `profile` in the tool's frame (metres, tip at the origin, axis along +Z). `place` maps a point of
	 * that frame into the geometry's frame and `turn` maps a direction.
	 */
	public static function revolved(profile:CutterProfile, place:(Float, Float, Float) -> Array<Float>,
			turn:(Float, Float, Float) -> Array<Float>):GeometryData {
		// The half-section from the tip up its side, closed across the top back to the axis.
		var section:Array<{r:Float, z:Float}> = [{r: 0.0, z: 0.0}];
		for (segment in profile.segments) switch segment {
			case Line(_, _, r1, z1, _): section.push({r: r1, z: z1});
			case Arc(cr, cz, r0, z0, r1, z1, _):
				var from = Math.atan2(z0 - cz, r0 - cr), to = Math.atan2(z1 - cz, r1 - cr);
				var radius = Math.sqrt((r0 - cr) * (r0 - cr) + (z0 - cz) * (z0 - cz));
				// The minor arc: unwrap the sweep into (-pi, pi].
				var sweep = to - from;
				while (sweep > Math.PI) sweep -= 2 * Math.PI;
				while (sweep <= -Math.PI) sweep += 2 * Math.PI;
				for (step in 1...ARC_STEPS + 1) {
					var angle = from + sweep * step / ARC_STEPS;
					section.push({r: cr + radius * Math.cos(angle), z: cz + radius * Math.sin(angle)});
				}
		}
		section.push({r: 0.0, z: profile.height()});
		// Each section edge is a band of quads with its own normals, so edges stay crisp.
		var bands = [for (index in 0...section.length - 1)
			if (Math.abs(section[index].r - section[index + 1].r) + Math.abs(section[index].z - section[index + 1].z) > 1e-12)
				index];
		var vertices = bands.length * (SIDES + 1) * 2, triangles = bands.length * SIDES * 2;
		var positions = Bytes.alloc(vertices * 12), normals = Bytes.alloc(vertices * 12);
		var colors = Bytes.alloc(vertices * 4), indices = Bytes.alloc(triangles * 12);
		var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
		var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
		var vertex = 0, triangle = 0;
		for (band in bands) {
			var a = section[band], b = section[band + 1];
			// Outward normal of the edge in the (r, z) half-plane.
			var dr = b.r - a.r, dz = b.z - a.z, size = Math.sqrt(dr * dr + dz * dz);
			var nr = dz / size, nz = -dr / size;
			var first = vertex;
			for (side in 0...SIDES + 1) {
				var angle = 2 * Math.PI * side / SIDES, c = Math.cos(angle), s = Math.sin(angle);
				var normal = turn(nr * c, nr * s, nz);
				for (end in [a, b]) {
					var point = place(end.r * c, end.r * s, end.z);
					for (axis in 0...3) {
						positions.setFloat(vertex * 12 + axis * 4, point[axis]);
						normals.setFloat(vertex * 12 + axis * 4, normal[axis]);
						low[axis] = Math.min(low[axis], point[axis]);
						high[axis] = Math.max(high[axis], point[axis]);
					}
					for (channel in 0...4) colors.set(vertex * 4 + channel, STEEL[channel]);
					vertex++;
				}
			}
			for (side in 0...SIDES) {
				var lower = first + side * 2, next = first + side * 2 + 2;
				for (corner in [lower, next, lower + 1, lower + 1, next, next + 1]) {
					indices.setInt32(triangle * 4, corner);
					triangle++;
				}
			}
		}
		var geometry = new GeometryData();
		geometry.addStream(1, 2, positions, vertices, 12);
		geometry.addStream(2, 2, normals, vertices, 12);
		geometry.addStream(6, 4, colors, vertices, 4);
		geometry.setIndexBuffer(indices, triangles * 3);
		geometry.setBounds(low[0], low[1], low[2], high[0], high[1], high[2]);
		return geometry;
	}
}
