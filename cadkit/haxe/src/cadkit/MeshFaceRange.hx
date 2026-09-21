package cadkit;

/** Immutable index-stream range belonging to one CAD face. */
class MeshFaceRange {
	public final faceIndex:Int;
	public final firstIndex:Int;
	public final indexCount:Int;

	public function new(faceIndex:Int, firstIndex:Int, indexCount:Int) {
		this.faceIndex = faceIndex;
		this.firstIndex = firstIndex;
		this.indexCount = indexCount;
	}

	public function endIndex():Int {
		return firstIndex + indexCount;
	}
}
