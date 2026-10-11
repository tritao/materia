package machinekit.standard;

import cadkit.threads.ThreadSpec;
import machinekit.standard.FastenerTightening.TighteningTableRow;

/** Facts shared by driven fasteners; geometry and placement belong to the component. */
interface ScrewFastener {
	public function screwDrive():ScrewDrive;
	public function threadSpec():ThreadSpec;
	public function fastenerClass():PropertyClass;
	public function shankLength():Float;
	public function threadedLength():Float;
	public function tightening():TighteningTableRow;
	/** Bit contact in component millimetres, with +Z into the shank. */
	public function driveFrame():materia.assembly.AssemblyRecord.AssemblyFrame;
}
