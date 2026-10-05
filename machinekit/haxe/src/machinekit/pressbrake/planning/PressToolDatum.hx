package machinekit.pressbrake.planning;

/** Representative installed contact members for one homogeneous tooling station. */
typedef PressToolDatum = {
	final station:String;
	final punch:String;
	final die:String;
}
