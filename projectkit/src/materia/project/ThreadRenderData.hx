package materia.project;

import materia.project.ThreadSpecData.ThreadSpecDataValues;

/** Derived face interval for renderers without a B-rep. Frame and length use the part's unit; spec uses millimetres. */
@:wire
typedef ThreadRenderData = {
	@:id(1) var faceName:String;
	@:id(2) var faceIndex:Int;
	@:id(3) var spec:ThreadSpecData;
	@:id(4) var origin:Array<Float>;
	@:id(5) var xDirection:Array<Float>;
	@:id(6) var direction:Array<Float>;
	@:id(7) var length:Float;
	@:id(8) var ringAtStart:Bool;
	@:id(9) var ringAtEnd:Bool;
}

/** Portable display packets are validated before an editor allocates resources. */
class ThreadRenderDataValues {
	public static function validate(thread:ThreadRenderData):Void {
		ThreadSpecDataValues.validate(thread.spec);
		if (thread.faceName == null || thread.faceName.length == 0 || thread.faceName.length > 4096
			|| thread.faceIndex < 0 || !Math.isFinite(thread.length) || thread.length <= 0)
			throw "Thread display needs a face name, index and positive length";
		for (vector in [thread.origin, thread.xDirection, thread.direction]) {
			if (vector == null || vector.length != 3) throw 'Thread ${thread.faceName} has an invalid frame';
			for (value in vector) if (!Math.isFinite(value)) throw 'Thread ${thread.faceName} frame is non-finite';
		}
		var x = thread.xDirection, z = thread.direction;
		if (Math.abs(x[0] * x[0] + x[1] * x[1] + x[2] * x[2] - 1) > 1e-6
			|| Math.abs(z[0] * z[0] + z[1] * z[1] + z[2] * z[2] - 1) > 1e-6
			|| Math.abs(x[0] * z[0] + x[1] * z[1] + x[2] * z[2]) > 1e-6)
			throw 'Thread ${thread.faceName} frame axes are not orthonormal';
	}
}
