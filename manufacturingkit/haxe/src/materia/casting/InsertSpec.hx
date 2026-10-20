package materia.casting;

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
	var DriveMount = "drive-mount";
	var EnclosureMount = "enclosure-mount";
}

enum abstract InsertFinish(String) to String {
	var GroundAfterCasting = "ground-after-casting";
	var MachinedAfterCasting = "machined-after-casting";
	var ReplicatedAfterCasting = "replicated-after-casting";
	var SourceAsCast = "as-cast";
}

class InsertFinishes {
	public static function values():Array<InsertFinish>
		return [InsertFinish.GroundAfterCasting, InsertFinish.MachinedAfterCasting, InsertFinish.ReplicatedAfterCasting, InsertFinish.SourceAsCast];

	public static function tokens():Array<String>
		return [for (finish in values()) (finish : String)];
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
			InsertRole.HatchMount,
			InsertRole.DriveMount,
			InsertRole.EnclosureMount
		];

	public static function tokens():Array<String>
		return [for (role in values()) (role : String)];
}
