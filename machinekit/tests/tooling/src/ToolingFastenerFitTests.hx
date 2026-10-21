import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.HexNut;
import machinekit.standard.PropertyClass;
import machinekit.tooling.ToolingFastenerFit as Fit;
import materia.assembly.AssemblyFrames as F;

class ToolingFastenerFitTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {
		assertions++;
		if (!ok) throw message;
	}
	static function rejects(fn:Void->Void,expected:String):Void {
		var message="";
		try fn() catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf(expected)>=0,'Expected "$expected", got "$message"');
	}
	static function near(actual:Float,expected:Float,message:String):Void
		check(Math.abs(actual-expected)<1e-7,message+": "+actual+" vs "+expected);

	static function fixtureCounterbore():Void {
		var source=SinglePocketSetup.foundation().insertLocations()[0].part.insertSpec();
		var hole=materia.casting.InsertHole.InsertHoleCodec.snapshot(source.holes)[0];
		hole.counterbore={diameter:9,depth:2};
		var pose=F.compose(F.translation(0,0,-18),F.turnY(Math.PI));
		var a=new machinekit.assembly.MachineAssembly();
		var minimum=materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole).minEngagement;
		rejects(() -> machinekit.tooling.ToolingHardwareCad.fixtureScrew(a,"short-fixture",hole,18+minimum,pose,F.identity()),"thread-engagement");
		check(a.components().length==0,"invalid actual fixture screws add no partial assembly components");
		machinekit.tooling.ToolingHardwareCad.fixtureScrew(a,"source-fixture",hole,
			18+materia.tooling.casting.InsertFixturePlanner.minimumInsertion(hole),pose,F.identity());
		check(a.components().length==2,"actual standard screw and washer include the source counterbore insertion depth");
		var unresolved=materia.casting.InsertHole.InsertHoleCodec.snapshot([hole])[0];unresolved.threadDepth=null;
		rejects(() -> machinekit.tooling.ToolingHardwareCad.fixtureScrew(new machinekit.assembly.MachineAssembly(),
			"unresolved",unresolved,30,pose,F.identity()),"resolved source thread");
		var invalid=F.identity();invalid.qw=0;
		rejects(() -> machinekit.tooling.ToolingHardwareCad.fixtureScrew(new machinekit.assembly.MachineAssembly(),
			"invalid-frame",hole,30,pose,invalid),"finite position and unit rotation");
	}
	public static function run():Void {
		var screw=SocketHeadCapScrew.metric("M6",50,Steel88),nut=HexNut.metric("M6");
		var origin=F.identity(),pose=F.translation(0,0,-40);
		near(screw.threadLength,24,"actual catalogue threaded length is shorter than the long screw");
		near(Fit.nutOverlap(screw,origin,nut,pose),nut.height,"whole nut inside threaded interval");
		Fit.requireNut(screw,origin,nut,pose,"synthetic-nut");
		pose.z=-28;
		near(Fit.nutOverlap(screw,origin,nut,pose),2,"partial engagement at the smooth-shank transition");
		rejects(() -> Fit.requireNut(screw,origin,nut,pose,"partial-shank"),"partial-shank");
		pose.z=-20;
		near(Fit.nutOverlap(screw,origin,nut,pose),0,"smooth shank contributes no thread engagement");
		pose.z=-52;
		near(Fit.nutOverlap(screw,origin,nut,pose),nut.height-2,"nut protruding beyond the tip loses engagement");
		rejects(() -> Fit.requireNut(screw,origin,nut,pose,"past-tip"),"not fully on screw thread");
		pose.z=-60;
		near(Fit.nutOverlap(screw,origin,nut,pose),0,"disjoint threaded intervals have zero overlap");
		pose=F.compose(F.translation(0,0,-40+nut.height),F.turnY(Math.PI));
		near(Fit.nutOverlap(screw,origin,nut,pose),nut.height,"reversed nut has the same finite overlap");
		var common=F.compose(F.translation(200,-100,300),F.turnY(0.6));
		near(Fit.nutOverlap(screw,common,nut,F.compose(common,pose)),nut.height,"common rigid placement preserves engagement");
		pose=F.translation(0.01,0,-40);
		rejects(() -> Fit.nutOverlap(screw,origin,nut,pose),"not coaxial");
		pose=F.compose(F.translation(0,0,-40),F.turnY(0.001));
		rejects(() -> Fit.nutOverlap(screw,origin,nut,pose),"not coaxial");
		pose=F.translation(0,0,-40);
		rejects(() -> Fit.nutOverlap(screw,origin,HexNut.metric("M8"),pose),"threads do not match");
		var fine=new cadkit.threads.ThreadSpec("M6x0.75",6,0.75,1,materia.project.ThreadSpecData.ThreadHand.RightHand,true,"6H",materia.project.ThreadSpecData.ThreadProfile.Metric);
		var differentPitch=HexNut.custom(nut.spec,fine);
		rejects(() -> Fit.nutOverlap(screw,origin,differentPitch,pose),"threads do not match");
		var shortScrew=SocketHeadCapScrew.metric("M6",12,Steel88);
		near(Fit.nutOverlap(shortScrew,origin,nut,F.translation(0,0,-8)),nut.height,"short screws clip threaded length at actual screw length");
		pose=F.identity();pose.qw=0;
		rejects(() -> Fit.nutOverlap(screw,origin,nut,pose),"finite position and unit rotation");
		fixtureCounterbore();
		Sys.println('ToolingFastenerFitTests: $assertions assertions passed');
	}
}
