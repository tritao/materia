package machinekit.milling;

import machinekit.assembly.MachineAssembly;

/** BT30 cartridge driven by a timing belt from a servo beside it (`MillSpindleHead`). */
class MillBeltSpindle implements MillSpindlePackage {
	/** Compact VMC head block: clamps the cartridge and carries the motor carrier feet. */
	public static inline var HEAD_HEIGHT:Float = 170;
	public static inline var MAXIMUM_RPM:Float = 8000;
	public final motorAdjustment:Null<Float>;
	public final drive:Null<MillSpindleDrivePart>;

	/** `drive` defaults to the installed Leadshine selection for the layout's head height. */
	public function new(?motorAdjustment:Float, ?drive:MillSpindleDrivePart) {
		this.motorAdjustment = motorAdjustment;
		this.drive = drive;
	}

	public function seat():Float return Bt30Cartridge.SEAT;

	public function diameter():Float return Bt30Cartridge.DIAMETER;

	public function headHeight():Float return HEAD_HEIGHT;

	public function maximumRpm():Float return MAXIMUM_RPM;

	public function assembly(l:MillZAxisLayout):MachineAssembly
		return new MillSpindleHead(l, motorAdjustment, drive == null ? MillSpindleDrivePart.installed(l.headHeight) : drive);
}
