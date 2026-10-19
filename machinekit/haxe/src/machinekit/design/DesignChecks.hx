package machinekit.design;

import cadkit.modeling.Part;
import machinekit.assembly.Diagnostics;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Geometric/process limits supplied by the designer, in mm; not strength qualification.
 * Callers own every shape. Access envelopes must include approach/withdrawal, not only the seated tool.
 */
class DesignChecks {
	public static function minimum(out:Diagnostics, subject:String, measured:Float, required:Float):Void {
		if (!Math.isFinite(measured) || !Math.isFinite(required) || required <= 0) throw "Invalid design limit";
		if (measured + 1e-6 < required) out.error("design.minimum-material",subject,
			'$subject: measured $measured mm, requires at least $required mm');
	}

	public static function blindFastener(out:Diagnostics, subject:String, length:Float, grip:Float,
		threadDepth:Float, cylindricalDrillDepth:Float, drillTip:Float, thickness:Float,
		minEngagement:Float, minTipGap:Float, minBack:Float):Void {
		for (v in [length,grip,threadDepth,cylindricalDrillDepth,drillTip,thickness,minEngagement,minTipGap,minBack])
			if (!Math.isFinite(v) || v <= 0) throw "Invalid blind fastener stack";
		minimum(out,subject+" thread engagement",length-grip,minEngagement);
		minimum(out,subject+" screw tip gap",threadDepth-(length-grip),minTipGap);
		minimum(out,subject+" tap runout",cylindricalDrillDepth-threadDepth,0.5);
		minimum(out,subject+" back wall",thickness-cylindricalDrillDepth-drillTip,minBack);
	}

	/** Axes may point in opposite directions; transverse position must agree. */
	public static function coaxial(out:Diagnostics, subject:String, a:AssemblyFrame, b:AssemblyFrame, tolerance:Float = 0.01):Void {
		var u=F.transformVector(a,0,0,1), v=F.transformVector(b,0,0,1);
		var dot=u.x*v.x+u.y*v.y+u.z*v.z;
		var dx=b.x-a.x,dy=b.y-a.y,dz=b.z-a.z, axial=dx*u.x+dy*u.y+dz*u.z;
		var error=Math.sqrt(Math.max(0,dx*dx+dy*dy+dz*dz-axial*axial));
		if(error>tolerance || Math.abs(dot)<0.999999) out.error("design.fastener-axis",subject,
			'$subject: transverse offset $error mm, axis dot product $dot');
	}

	/** Required ligament ring/shell must be wholly contained in the actual cut solid.
	 * This catches nearby holes and pockets as well as the outer boundary.
	 */
	public static function material(out:Diagnostics, subject:String, required:Part, host:Part):Void {
		var common=required.intersect(host);
		var missing=required.volume()-common.volume(); common.close();
		if (missing > 1e-5) out.error("design.material-envelope",subject,
			'$subject: required material envelope missing $missing mm^3');
	}

	/** Include clearance in the envelope itself. No exemptions for touching a tool shaft. */
	public static function access(out:Diagnostics, subject:String, envelope:Part, obstacles:Array<{id:String,part:Part}>):Void {
		for (obstacle in obstacles) {
			var common=envelope.intersect(obstacle.part), volume=common.volume(); common.close();
			if(volume > 1e-5) out.error("design.access-blocked",subject+"/"+obstacle.id,
				'$subject blocked by ${obstacle.id}: $volume mm^3 of required clearance');
		}
	}
}
