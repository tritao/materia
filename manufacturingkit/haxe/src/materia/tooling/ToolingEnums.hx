package materia.tooling;

enum abstract ToolingLengthUnit(String) to String {
	var Millimetres = "mm";
}

enum abstract CoatedFace(String) to String {
	var PositiveZ = "positive-z";
	var NegativeZ = "negative-z";
	var Both = "both";
	var Uncoated = "none";
}

enum abstract StockDirection(String) to String {
	var Free = "free";
	var WidthAligned = "width-aligned";
}

enum abstract InsertRole(String) to String {
	var TableMount = "table-mount";
	var LiftingMount = "lifting-mount";
	var ThreadedAnchor = "threaded-anchor";
	var GenericMount = "generic-mount";
	var GuideSeat = "guide-seat";
	var BearingSeat = "bearing-seat";
	var ColumnMount = "column-mount";
	var LevelingSeat = "leveling-seat";
	var HatchMount = "hatch-mount";
}

enum abstract InsertFinish(String) to String {
	var GroundAfterCasting = "ground-after-casting";
	var MachinedAfterCasting = "machined-after-casting";
	var ReplicatedAfterCasting = "replicated-after-casting";
	var SourceAsCast = "as-cast";
}

enum abstract MachiningSide(String) to String {
	var PositiveZ = "positive-z";
	var NegativeZ = "negative-z";
}

enum abstract ContactFace(String) to String {
	var None = "none";
	var PositiveZ = "positive-z";
	var NegativeZ = "negative-z";
}

/** Typed role vocabulary shared by source schemas and neutral validation. */
class InsertRoles {
	public static function values():Array<InsertRole>
		return [
			InsertRole.TableMount,
			InsertRole.LiftingMount,
			InsertRole.ThreadedAnchor,
			InsertRole.GenericMount,
			InsertRole.GuideSeat,
			InsertRole.BearingSeat,
			InsertRole.ColumnMount,
			InsertRole.LevelingSeat,
			InsertRole.HatchMount
		];

	public static function tokens():Array<String>
		return [for (role in values()) (role : String)];
}
