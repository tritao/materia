package cadkit.threads;

import haxe.io.Bytes;
import materia.project.ThreadSpecData;
import materia.project.ThreadSpecData.ThreadProfile;

/**
 * A cosmetic thread texture for `ThreadSurface` coordinates: analytic flanks and root shading for the thread's
 * profile, as a tangent-space normal map or a colour (shade) map, RGBA8 with explicit mip levels whose normals
 * are renormalized. One period of the texture is one turn per start along u and one pitch along v.
 */
class ThreadTexture {
	public static inline var SIZE:Int = 128;
	public static inline var MIPS:Int = 8;

	/** All mip levels, largest first, each SIZE >> level square. */
	public static function pixels(spec:ThreadSpecData, normal:Bool):Bytes {
		var side = SIZE, count = 0;
		while (side > 0) {
			count += side * side * 4;
			side = Std.int(side / 2);
		}
		var result = Bytes.alloc(count);
		var flank = Math.tan((spec.profile == Metric ? 60.0 : spec.profile == Acme ? 75.5 : 75.0) * Math.PI / 180);
		var helix = spec.pitch * spec.starts / (Math.PI * spec.majorDiameter);
		for (y in 0...SIZE)
			for (x in 0...SIZE) {
				var phase = (x - y) / SIZE;
				phase -= Math.floor(phase);
				var distance = Math.abs(phase - 0.5);
				// A flat crest and root avoid infinitely sharp texture cusps.
				var height = Math.max(0, Math.min(1, (distance - 0.12) / 0.28));
				var slope = distance > 0.12 && distance < 0.4 ? (phase < 0.5 ? -flank : flank) : 0.0;
				if (spec.profile == Ball) {
					// Smooth rounded raceway with a flat land. Cosmetic only: this is not
					// a supplier-qualified ball-contact section or a CAD displacement.
					var width = 0.36;
					var t = Math.min(1, distance / width);
					height = 0.5 - 0.5 * Math.cos(Math.PI * t);
					slope = distance < width ? (phase < 0.5 ? -1 : 1) * 0.2 * Math.PI / (2 * width) * Math.sin(Math.PI * t) : 0;
				}
				var offset = (y * SIZE + x) * 4;
				if (normal) {
					var nx = -slope * helix, ny = slope, nz = 1.0;
					var length = Math.sqrt(nx * nx + ny * ny + nz * nz);
					result.set(offset, channel(nx / length));
					result.set(offset + 1, channel(ny / length));
					result.set(offset + 2, channel(nz / length));
				} else {
					var color = Std.int(Math.round(255 * (0.96 + 0.04 * height)));
					result.set(offset, color);
					result.set(offset + 1, color);
					result.set(offset + 2, color);
				}
				result.set(offset + 3, 255);
			}
		var previous = 0, next = SIZE * SIZE * 4;
		side = Std.int(SIZE / 2);
		while (side > 0) {
			for (y in 0...side)
				for (x in 0...side) {
					var value = [0.0, 0.0, 0.0];
					for (dy in 0...2)
						for (dx in 0...2) {
							var source = previous + ((y * 2 + dy) * side * 2 + x * 2 + dx) * 4;
							for (axis in 0...3)
								value[axis] += normal ? result.get(source + axis) / 127.5 - 1 : result.get(source + axis) * 1.0;
						}
					var offset = next + (y * side + x) * 4;
					if (normal) {
						var length = Math.sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
						for (axis in 0...3)
							result.set(offset + axis, channel(value[axis] / length));
					} else
						for (axis in 0...3)
							result.set(offset + axis, Std.int(Math.round(value[axis] / 4)));
					result.set(offset + 3, 255);
				}
			previous = next;
			next += side * side * 4;
			side = Std.int(side / 2);
		}
		return result;
	}

	static function channel(value:Float):Int
		return Std.int(Math.round((value * 0.5 + 0.5) * 255));
}
