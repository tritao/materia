package machinekit.pressbrake.planning;

/** Physical ownership survives decomposition; diagnoses never parse a piece identity. */
enum PressCollisionOwner {
	SheetWall(wall:Int);
	SheetBend(bend:Int);
	Punch(station:String, segment:Int);
	Die(station:String, segment:Int);
	MachineMember(member:String);
}
