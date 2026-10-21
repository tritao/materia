package materia.casting;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef NominalThreadedFastener = {
	var shankLength:Float;
	var threadedLength:Float;
	var bearingFrame:AssemblyFrame;
}
typedef NominalThreadFinding = {var code:String; var subject:String; var passed:Bool; var message:String;}

/** Shared nominal interval checks for actual source screws and selected fixture
 * stock. Callers establish matching thread specifications; this function checks
 * placement, full-thread overlap, smooth shank and the source tip-gap limits.
 * It does not establish physical strength, torque or manufacturing evidence.
 */
class ThreadedFastenerFit {
	public static function checkIntervals(target:InsertHole,frame:AssemblyFrame,
		fastener:NominalThreadedFastener,subject:String):Array<NominalThreadFinding> {
		var result:Array<NominalThreadFinding> = [],thread=target.thread,usable=target.threadDepth;
		materia.assembly.AssemblyCodec.validateFrame(frame);
		materia.assembly.AssemblyCodec.validateFrame(fastener.bearingFrame);
		if (thread==null || usable==null) throw "Thread fit requires a resolved source thread";
		if (!Math.isFinite(fastener.shankLength) || !Math.isFinite(fastener.threadedLength)
			|| fastener.shankLength<=0 || fastener.threadedLength<=0 || fastener.threadedLength>fastener.shankLength)
			throw "Invalid nominal screw dimensions";
		if (!Math.isFinite(usable) || usable<=0 || !Math.isFinite(target.depth) || usable>target.depth)
			throw "Invalid source thread depth";
		var rules=InsertHole.InsertHoleCodec.threadLimits(target);
		var relative=F.compose(F.inverse(fastener.bearingFrame),frame),axis=F.transformVector(relative,0,0,1);
		var radial=Math.sqrt(relative.x*relative.x+relative.y*relative.y);
		function add(code:String,passed:Bool,message:String):Void
			result.push({code:code,subject:subject,passed:passed,message:message});
		// Numerical coincidence of nominal source axes, not a machining tolerance.
		add("thread-axis",radial<=1e-8 && Math.abs(axis.x)<=1e-10 && Math.abs(axis.y)<=1e-10,
			"Source screw and tapped-hole axes must coincide; radial offset "+radial+" mm.");
		var through=target.extent==InsertHole.HoleExtent.Through;
		var reverse=axis.z<0,fullThrough=through && usable>=target.depth-1e-8;
		add("thread-entry",!reverse || fullThrough,"Reverse entry requires a source hole tapped through its full depth.");
		var counterbore=target.counterbore;
		var start=counterbore==null ? 0.0 : counterbore.depth;
		var first=relative.z+axis.z*start,last=relative.z+axis.z*usable;
		var low=Math.min(first,last),high=Math.max(first,last);
		var screwStart=fastener.shankLength-fastener.threadedLength;
		var overlap=usable<=start ? 0.0 : Math.max(0,Math.min(high,fastener.shankLength)-Math.max(low,screwStart));
		add("thread-engagement",overlap+1e-8>=rules.minEngagement,
			"Full-thread overlap "+overlap+" mm; source policy requires "+rules.minEngagement+" mm.");
		var plainOverlap=Math.max(0,Math.min(high,screwStart)-Math.max(low,0));
		add("thread-shank",plainOverlap<=1e-8,"Smooth shank must not enter the source full-thread region; overlap "+plainOverlap+" mm.");
		if (!through || !fullThrough) {
			var limit=target.relief==null ? usable : target.depth;
			var penetration=(fastener.shankLength-relative.z)*axis.z;
			var gap=limit-penetration;
			add("thread-tip-gap",!reverse && gap+1e-8>=rules.minTipGap,
				"Screw tip clearance "+gap+" mm; source policy requires "+rules.minTipGap+" mm.");
		}
		return result;
	}
}
