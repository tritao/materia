package machinekit.tooling;

import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.HexNut;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

/** Nominal geometric engagement from the actual selected components. This does
 * not establish tightening, strip strength, load capacity or physical evidence.
 */
class ToolingFastenerFit {
	/** Exact coaxial interval overlap, using the screw's full threaded length
	 * measured from its tip. Nut CAD runs from local Z=0 to Z=height; screw CAD
	 * runs toward -Z from its head bearing face. Reversed nut placement is valid.
	 */
	public static function nutOverlap(screw:SocketHeadCapScrew,screwPose:AssemblyFrame,
		nut:HexNut,nutPose:AssemblyFrame):Float {
		materia.assembly.AssemblyCodec.validateFrame(screwPose);
		materia.assembly.AssemblyCodec.validateFrame(nutPose);
		if (!screw.thread.matches(nut.thread) || screw.thread.internal || !nut.thread.internal)
			throw "Tooling screw/nut threads do not match";
		var relative=F.compose(F.inverse(screwPose),nutPose);
		var axis=F.transformVector(relative,0,0,1);
		// Numerical coincidence, not an allowance for eccentric or tilted threads.
		if (Math.abs(relative.x)>1e-8 || Math.abs(relative.y)>1e-8
			|| Math.abs(axis.x)>1e-10 || Math.abs(axis.y)>1e-10)
			throw "Tooling screw/nut axes are not coaxial";
		var end=relative.z+axis.z*nut.height;
		var lower=Math.min(relative.z,end),upper=Math.max(relative.z,end);
		var threadStart=-screw.length,threadEnd=threadStart+screw.threadLength;
		return Math.max(0,Math.min(upper,threadEnd)-Math.max(lower,threadStart));
	}

	/** Require the entire nominal nut thickness to be on full screw thread.
	 * This deliberately does not use the insert-steel engagement policy: the
	 * selected standard nut is a different component and its strength is separate.
	 */
	public static function requireNut(screw:SocketHeadCapScrew,screwPose:AssemblyFrame,
		nut:HexNut,nutPose:AssemblyFrame,id:String):Void {
		var overlap=nutOverlap(screw,screwPose,nut,nutPose);
		if (overlap+1e-8<nut.height)
			throw "Tooling nut is not fully on screw thread: "+id+" ("+overlap+" of "+nut.height+" mm)";
	}
}
