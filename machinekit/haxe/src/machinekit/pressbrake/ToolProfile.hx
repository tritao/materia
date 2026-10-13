package machinekit.pressbrake;

/** A physical section installed along the press bed. A four-V selection retains its whole block. */
enum ToolProfile {
	Punch(profile:PunchProfile);
	SingleV(profile:DieProfile);
	FourV(profile:MultiVDieProfile, activeFace:Int);
}
