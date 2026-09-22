package bimkit;

import cadkit.parametric.DocumentChange;

class WallRoleChange implements DocumentChange {
	private final model:BimDocument;
	private final role:WallRole;
	private final before:Bool;
	private final after:Bool;

	public function new(model:BimDocument, role:WallRole, before:Bool, after:Bool) {
		this.model = model;
		this.role = role;
		this.before = before;
		this.after = after;
	}

	public function undo():Void
		model.restoreWallRole(role, before);

	public function redo():Void
		model.restoreWallRole(role, after);
}
