package machinekit.assembly;

/** A change to one stated transmission allowance. */
enum AllowanceEdit {
	Leave;
	Clear;
	State(value:Float);
}
