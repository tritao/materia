package machinekit.pressbrake.planning;

/** Motion of an assembly-derived machine volume during the planned stroke. */
enum PressMemberMotion {
	Fixed;
	Ram;
	BackgaugeX;
	BackgaugeXR;
}
