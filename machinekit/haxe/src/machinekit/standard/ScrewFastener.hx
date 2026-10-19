package machinekit.standard;

import cadkit.threads.ThreadSpec;
import machinekit.standard.FastenerTightening.TighteningTableRow;

/** Facts shared by driven fasteners; geometry and placement belong to the component. */
interface ScrewFastener {
	/** Bearing face, with +Z into the shank, independent of head style. */
	public function bearingFrame():materia.assembly.AssemblyRecord.AssemblyFrame;
	/** Conservative radial head envelope and minimum circular bearing footprint. */
	public function headClearanceRadius():Float;
	public function headBearingRadius():Float;
	public function screwDrive():ScrewDrive;
	public function threadSpec():ThreadSpec;
	public function fastenerClass():PropertyClass;
	public function shankLength():Float;
	public function threadedLength():Float;
	public function tightening():TighteningTableRow;
	/** Bit contact in component millimetres, with +Z into the shank. */
	public function driveFrame():materia.assembly.AssemblyRecord.AssemblyFrame;
}
