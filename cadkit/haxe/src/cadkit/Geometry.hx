package cadkit;

import CadKit;

/** Constructors for HXI value types used by the high-level CadKit API. */
class Geometry {
	public static function vec3(x:Float, y:Float, z:Float):CadKit.Vec3 {
		var result = new CadKit.Vec3();
		result.set_x(x);
		result.set_y(y);
		result.set_z(z);
		return result;
	}

	public static function meshOptions(linearDeflection:Float, angularDeflection:Float):CadKit.MeshOptions {
		var result = new CadKit.MeshOptions();
		result.set_linearDeflection(linearDeflection);
		result.set_angularDeflection(angularDeflection);
		return result;
	}
}
