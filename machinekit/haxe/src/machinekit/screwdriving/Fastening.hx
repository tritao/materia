package machinekit.screwdriving;

import cadkit.modeling.Plane;
import cadkit.threads.ThreadSpec;
import machinekit.standard.FastenerTightening.TighteningTableRow;

/** Joint-qualified limits, in radians from snug; CAD dimensions cannot supply them. */
typedef TighteningAngleWindow = {var minimum:Float; var maximum:Float;}

/** A face of a particular occurrence; reusable component names alone are not identities. */
typedef FasteningFace = {var member:String; var faceName:String;}

/** Derived seated connection. Frames and lengths are in assembly millimetres. */
class Fastening {
	public final screw:FasteningFace;
	public final clamped:FasteningFace;
	public final threaded:FasteningFace;
	public final frame:Plane;
	public final engagementLength:Float;
	public final minimumEngagement:Float;
	public final thread:ThreadSpec;
	public final tightening:TighteningTableRow;
	public final angleWindow:Null<TighteningAngleWindow>;

	public function new(screw:FasteningFace, clamped:FasteningFace, threaded:FasteningFace,
		frame:Plane, engagementLength:Float, minimumEngagement:Float, thread:ThreadSpec,
		tightening:TighteningTableRow, angleWindow:Null<TighteningAngleWindow>) {
		this.screw = screw; this.clamped = clamped; this.threaded = threaded;
		this.frame = frame; this.engagementLength = engagementLength;
		this.minimumEngagement = minimumEngagement; this.thread = thread;
		this.tightening = tightening;
		this.angleWindow = angleWindow == null ? null : {minimum: angleWindow.minimum, maximum: angleWindow.maximum};
	}

	/** Length prefixes make occurrence/face combinations unambiguous even when names contain separators. */
	public function name():String {
		var values = [screw.member, screw.faceName, clamped.member, clamped.faceName, threaded.member, threaded.faceName];
		return "fastening:" + [for (value in values) value.length + ":" + value].join("|");
	}
}
