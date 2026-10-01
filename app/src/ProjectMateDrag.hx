package app;

import cadkit.modeling.AssemblyMateDrag;
import kinematicskit.Vector3;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;

/** The document session's `SceneAssemblyDrag` for a mated part: an `AssemblyMateDrag` previewed in the scene, committed as one edit. */
class ProjectMateDrag implements SceneAssemblyDrag {
	final session:ProjectDocumentSession;
	final drag:AssemblyMateDrag;
	final before:AssemblyStateRecord;
	final metresPerUnit:Float;
	var currentTarget:Array<Float>;
	var lastMessage = "Following";
	var onTarget = true;
	var moved = false;
	var finished = false;

	public function new(session:ProjectDocumentSession, drag:AssemblyMateDrag, before:AssemblyStateRecord, metresPerUnit:Float) {
		this.session = session;
		this.drag = drag;
		this.before = before;
		this.metresPerUnit = metresPerUnit;
		currentTarget = grabbedPoint();
	}

	public function update(x:Float, y:Float, z:Float):Void {
		if (finished) return;
		if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z)) return;
		currentTarget = [x, y, z];
		var result = drag.drag(new Vector3(x / metresPerUnit, y / metresPerUnit, z / metresPerUnit));
		onTarget = result.following;
		lastMessage = result.message;
		moved = true;
		session.previewAssemblyPoses(drag.previewPose);
	}

	public function grabbedPoint():Array<Float> {
		var point = drag.grabbedPoint();
		return [point.x * metresPerUnit, point.y * metresPerUnit, point.z * metresPerUnit];
	}

	public function target():Array<Float> return currentTarget.copy();

	public function following():Bool return onTarget;

	public function message():String return lastMessage;

	public function commit():Bool {
		if (finished) return false;
		finished = true;
		return session.finishAssemblyDrag("Drag " + drag.occurrence, before, moved ? drag.commit() : null);
	}

	public function cancel():Void {
		if (finished) return;
		finished = true;
		session.finishAssemblyDrag("Drag " + drag.occurrence, before, null);
	}
}
