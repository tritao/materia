package machinekit.pressbrake.planning;

/** Exact local boxes for an authored member whose opening cannot be replaced by a filled envelope. */
typedef PressMemberVolume = {
	final member:String;
	final boxes:Array<PressEnvelope>;
}
