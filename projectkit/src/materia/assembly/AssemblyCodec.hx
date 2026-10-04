package materia.assembly;

import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Validation shared by assembly frames and identifiers. */
class AssemblyCodec {
	public static function validateFrame(frame:AssemblyFrame):Void {
		if (frame == null || !Math.isFinite(frame.x) || !Math.isFinite(frame.y) ||
			!Math.isFinite(frame.z) || !Math.isFinite(frame.qx) || !Math.isFinite(frame.qy) ||
			!Math.isFinite(frame.qz) || !Math.isFinite(frame.qw) ||
			Math.abs(frame.x) > 1e9 || Math.abs(frame.y) > 1e9 || Math.abs(frame.z) > 1e9 ||
			Math.abs(frame.qx * frame.qx + frame.qy * frame.qy + frame.qz * frame.qz + frame.qw * frame.qw - 1) > 1e-4)
			throw "Assembly frame must have finite position and unit rotation";
	}

	/**
		Whether text holds a NUL character. Checked by code: a `"\x00"` literal cannot be a HashLink string
		constant (and compiled to the text "x00" before haxeon decoded `\x` escapes).
	*/
	public static function containsNul(value:String):Bool {
		for (i in 0...value.length)
			if (StringTools.fastCodeAt(value, i) == 0) return true;
		return false;
	}
}
