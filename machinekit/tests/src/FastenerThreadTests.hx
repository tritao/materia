import cadkit.threads.Threads;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.HexBolt;
import machinekit.standard.HexNut;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LeadScrewNut;

/** Component rebuilds attach nominal threads without changing their B-rep envelopes. */
class FastenerThreadTests {
	static function near(a:Float, b:Float):Void { if (Math.abs(a - b) > 1e-5) throw 'thread expected $b, got $a'; }
	public static function main():Void run();
	public static function run():Void {
		var screw = SocketHeadCapScrew.metric("M6", 40);
		var part = screw.geometry();
		var thread = Threads.of(part)[0];
		near(thread.length, screw.threadLength);
		near(thread.frame.origin.z, -(40 - screw.threadLength));
		near(thread.frame.normal.z, -1);
		part.close();
		var tool = screw.tapHole(20);
		var tap = Threads.of(tool)[0];
		near(tap.length, 20); near(tap.frame.origin.z, 0);
		if (!tap.attribute.spec.internal || !tap.attribute.spec.matches(screw.thread)) throw "tap thread differs";
		tool.close();
		var bolt = HexBolt.metric("M6", 30).geometry();
		near(Threads.of(bolt)[0].length, 30); bolt.close();
		var nut = HexNut.metric("M6").geometry();
		near(Threads.of(nut)[0].length, 5.2); nut.close();
		var lead = new LeadScrew(new LeadScrewThread(machinekit.motion.LeadScrewThread.LeadScrewThreadFamily.MetricTrapezoidal, 8, 2), 100);
		var shaft = lead.geometry(); near(Threads.of(shaft)[0].length, 100); shaft.close();
		Sys.println("Fastener face threads passed");
	}
}
