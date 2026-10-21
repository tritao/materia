import haxe.io.Bytes;
import haxeon.editor.ViewportLook;

/**
 * The editor's studio look as an environment map for the docs' `<model-viewer>`: an equirectangular Radiance HDR
 * of the look's room (sky over ground, meeting at the horizon) and its lights as soft panels, placed for one
 * catalogue camera. The editor's rig turns with its camera; model-viewer's environment is fixed in the world, so
 * each catalogue view gets its own map and a part's first frame is lit as the editor would light it from there.
 *
 * Axes are glTF's (Y up). A light of intensity `I` and panel radius `r` is a Gaussian lobe of radiance
 * `I / r^2 * exp(-angle^2 / r^2)`, which gives the irradiance `pi * I` of the editor's directional light, so
 * diffuse lighting matches too; model-viewer adds the roughness blur the editor's shader computes analytically.
 */
class StudioEnvironment {
	static inline var WIDTH = 256;
	static inline var HEIGHT = 128;

	/** The map for a camera at `theta` degrees about the vertical (from +Z towards +X) and `phi` down from it. */
	public static function hdr(look:ViewportLook, theta:Float, phi:Float):Bytes {
		var t = theta * Math.PI / 180, p = phi * Math.PI / 180;
		// Camera basis: z toward the viewer, x to its right, y up on screen.
		var z = [Math.sin(p) * Math.sin(t), Math.cos(p), Math.sin(p) * Math.cos(t)];
		var x = normalize(cross([0, 1, 0], z));
		var y = cross(z, x);
		var panels = [
			for (light in look.lights) {
				var d = light.direction;
				// Toward the light, in world axes.
				var toward = [for (axis in 0...3) -(d[0] * x[axis] + d[1] * y[axis] + d[2] * z[axis])];
				{toward: toward, light: light};
			}
		];
		var header = Bytes.ofString('#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y $HEIGHT +X $WIDTH\n');
		var out = Bytes.alloc(header.length + WIDTH * HEIGHT * 4);
		out.blit(0, header, 0, header.length);
		var offset = header.length;
		for (row in 0...HEIGHT) {
			// three.js reads the top row as straight up and column u as the direction (cos a, y, sin a),
			// a = (u - 0.5) * 2 pi.
			var elevation = Math.PI / 2 - (row + 0.5) * Math.PI / HEIGHT;
			for (column in 0...WIDTH) {
				var a = ((column + 0.5) / WIDTH - 0.5) * 2 * Math.PI;
				var dir = [Math.cos(elevation) * Math.cos(a), Math.sin(elevation), Math.cos(elevation) * Math.sin(a)];
				// Sky above the horizon, ground below, over a narrow edge: the editor's room, whose irradiance is its
				// hemisphere ambient (SceneKit blurs the edge by roughness; model-viewer's prefiltering does here).
				var t = Math.max(0.0, Math.min(1.0, (dir[1] + 0.05) / 0.1)), up = t * t * (3 - 2 * t);
				var rgb = [for (channel in 0...3) look.ground[channel] + (look.sky[channel] - look.ground[channel]) * up];
				for (panel in panels) {
					var radius = panel.light.radius;
					if (radius <= 0)
						continue;
					var cosine = dir[0] * panel.toward[0] + dir[1] * panel.toward[1] + dir[2] * panel.toward[2];
					var angle = Math.acos(Math.max(-1.0, Math.min(1.0, cosine)));
					var radiance = panel.light.intensity / (radius * radius) * Math.exp(-angle * angle / (radius * radius));
					for (channel in 0...3)
						rgb[channel] += panel.light.color[channel] * radiance;
				}
				rgbe(out, offset, rgb);
				offset += 4;
			}
		}
		return out;
	}

	static function rgbe(out:Bytes, offset:Int, rgb:Array<Float>):Void {
		var peak = Math.max(rgb[0], Math.max(rgb[1], rgb[2]));
		if (peak < 1e-32) {
			out.setInt32(offset, 0);
			return;
		}
		// peak = mantissa * 2^exponent with mantissa in [0.5, 1).
		var exponent = Math.ceil(Math.log(peak) / Math.log(2));
		var scale = Math.pow(2, exponent);
		if (peak / scale >= 1) {
			exponent++;
			scale *= 2;
		}
		var factor = 256 / scale;
		for (channel in 0...3)
			out.set(offset + channel, Std.int(Math.min(255, rgb[channel] * factor)));
		out.set(offset + 3, Std.int(exponent) + 128);
	}

	static function cross(a:Array<Float>, b:Array<Float>):Array<Float>
		return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

	static function normalize(v:Array<Float>):Array<Float> {
		var length = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
		return [v[0] / length, v[1] / length, v[2] / length];
	}
}
