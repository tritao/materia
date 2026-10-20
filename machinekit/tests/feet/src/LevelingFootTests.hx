import cadkit.modeling.Part;
import cadkit.modeling.Polygon;
import machinekit.structural.LevelingFoot;
import machinekit.structural.LevelingFootSpec;
import machinekit.structural.LevelingFootSpec.LevelingFootCodec;
import machinekit.component.Solids;
import machinekit.component.MachineKitComponents;
import machinekit.assembly.MachineAssembly;
import machinekit.standard.FastenerThreads;

/** Migration controls use the retired solids independently of the new recipe builder. */
class LevelingFootTests {
	static var assertions = 0;
	static function check(ok:Bool, message:String):Void {
		assertions++;
		if (!ok) throw message;
	}
	static function rejects(fn:Void -> Void, message:String):Void {
		var caught = "";
		try fn() catch (error:Dynamic) caught = Std.string(error);
		check(caught.indexOf(message) >= 0, 'Expected "$message", got "$caught"');
	}
	static function equivalent(label:String, spec:LevelingFootSpec, expected:Part):Void {
		var part = LevelingFoot.of(spec), actual = part.geometry();
		try {
			check(actual.valid(), label + " valid");
			check(actual.solidCount() == 1, label + " one connected nominal body");
			var threads = cadkit.threads.Threads.of(actual);
			switch spec {
				case Foot(_, _, thread, _, top, mount, _, _):
					check(threads.length > 0, label + " source thread reaches CAD");
					for (t in threads) check(t.attribute.spec.matches(thread) && !t.attribute.spec.internal,
						label + " external thread matches source");
					check(part.connector("mount").frame.z == mount && part.connector("top").frame.z == top,
						label + " mounting datum distinct from stem top");
				case Screw(thread, _, _):
					check(threads.length == 1 && threads[0].attribute.spec.matches(thread)
						&& !threads[0].attribute.spec.internal, label + " screw thread identity");
				case Nut(_, _, thread):
					check(threads.length == 1 && threads[0].attribute.spec.matches(thread)
						&& threads[0].attribute.spec.internal, label + " internal nut thread identity");
				case Pad(_, _), Plate(_, _, _, _):
					check(threads.length == 0, label + " clearance hardware unthreaded");
			}
			var missing = expected.subtract(actual), extra = actual.subtract(expected);
			var difference = missing.volume() + extra.volume();
			missing.close(); extra.close();
			check(difference < 1e-5, label + " zero symmetric difference: " + difference);
			var saved = LevelingFootCodec.encode(spec);
			check(LevelingFootCodec.encode(LevelingFootCodec.decode(saved)) == saved, label + " strict round trip");
			var assembly = new MachineAssembly();
			assembly.addComponent("part", part);
			var copy = MachineAssembly.decode(assembly.encode(), MachineKitComponents.standard());
			check(copy.components()[0].component.values().token("part") == saved, label + " assembly source preserved");
			check(part.ratingsBasis == Assumed, label + " no supplier qualification inferred");
		} catch (error:Dynamic) {
			actual.close(); expected.close(); throw error;
		}
		actual.close(); expected.close();
	}
	static function installedMillFeet():Void {
		var layout = new machinekit.milling.MillFrameLayout({yRailSpacing:220, yRailStart:-270,
			yRailLength:410, columnFront:180, columnHeight:600, zRailSpacing:180, zRailStart:40, zRailLength:400},
			{footPockets:true, columnFootWidth:440});
		var frame = new machinekit.milling.MillFrame(layout);
		frame.validate();
		var poses = frame.solvedPoses();
		var feet = frame.components().filter(row -> StringTools.startsWith(row.id,"levelingFoot"));
		check(feet.length == 4,"mill installs four shared feet");
		for (row in feet) {
			var pose:materia.assembly.AssemblyRecord.AssemblyFrame = poses.get(row.id);
			var floor = materia.assembly.AssemblyFrames.compose(pose,row.component.connector("floor").frame);
			var mount = materia.assembly.AssemblyFrames.compose(pose,row.component.connector("mount").frame);
			check(Math.abs(floor.z+50)<1e-6,"installed mill floor remains -50 mm");
			check(Math.abs(mount.z-layout.casting.footAccessFloor)<1e-6,"foot mount follows its source steel seat");
			var local = row.component.geometry();
			var actual = machinekit.assembly.PosedParts.moved(local,pose); local.close();
			var old = Solids.union([Part.cylinderSpan(35,-50,-30,pose.x,pose.y),
				Part.cylinderSpan(8,-30,65,pose.x,pose.y),Part.cylinderSpan(14,40,50,pose.x,pose.y)]);
			try {
				var a=actual.subtract(old), b=old.subtract(actual);
				var difference=a.volume()+b.volume();a.close();b.close();
				check(difference<1e-5,"installed foot zero symmetric difference "+row.id);
			} catch(error:Dynamic) {actual.close();old.close();throw error;}
			actual.close();old.close();
		}
	}
	public static function main():Void {
		var m16 = FastenerThreads.metric(16, 2), m24 = FastenerThreads.metric(24, 3);
		equivalent("mill foot in floor frame", Foot(70,20,m16,20,115,90,Nominal,RoundCollar(28,10,90)),
			Solids.union([Part.cylinderSpan(35,0,20),Part.cylinderSpan(8,20,115),Part.cylinderSpan(14,90,100)]));
		equivalent("hobber foot", Foot(100,15,m24,15,232,207,Nominal,NoCollar),
			Solids.union([Part.cylinderSpan(50,0,15),Part.cylinderSpan(12,15,232),Part.cylinderSpan(12,39,47)]));
		equivalent("router foot", Foot(100,20,FastenerThreads.metric(20,2.5),20,120,100,Nominal,NoCollar),
			Solids.union([Part.cylinderSpan(50,0,20),Part.cylinderSpan(10,20,120)]));
		for (s in [machinekit.structural.TSlotExtrusion.forProfile("HFS5-2020"),
			machinekit.structural.TSlotExtrusion.forProfile("HFS5-4040")]) {
			var hardware = machinekit.structural.TSlotTableHardware.forProfile(s), thread = hardware.footSpec();
			var t = FastenerThreads.metric(thread.diameter,thread.pitch);
			equivalent("table foot " + s.designation, Foot(s.size,8,t,7,40+hardware.footEngagement,40,ThreadRoot,NoCollar),
				Solids.union([Part.cylinderSpan(s.size/2,0,8),Part.cylinderSpan(t.externalMinorDiameter()/2,7,40+hardware.footEngagement)]));
		}
		equivalent("removable plate", Plate(110,110,15,26),
			Solids.cut(Part.box(110,110,15),[Part.cylinderSpan(13,-1,16)]));
		equivalent("support nut", Nut(Hexagon(36),19,m24),
			Solids.cut(Part.prism(Polygon.regular(6,36),0,19),[Part.cylinderSpan(12,-1,20)]));
		equivalent("round nut", Nut(Round(28),10,m16),
			Solids.cut(Part.cylinderSpan(14,0,10),[Part.cylinderSpan(8,-1,11)]));
		equivalent("separate pad", Pad(70,20),Part.cylinderSpan(35,0,20));
		equivalent("separate nominal screw", Screw(m16,95,Nominal),Part.cylinderSpan(8,0,95));
		equivalent("separate root screw", Screw(m16,95,ThreadRoot),Part.cylinderSpan(m16.externalMinorDiameter()/2,0,95));
		rejects(() -> LevelingFootCodec.decode('{"version":0,"part":{}}'),"schema version");
		rejects(() -> LevelingFootCodec.decode('{"version":1,"part":{"kind":"pad","diameter":70,"height":20,"extra":1}}'),"fields");
		rejects(() -> LevelingFootCodec.decode('{"version":1,"part":{"kind":"pad","diameter":"70","height":20}}'),"finite number");
		rejects(() -> LevelingFootCodec.encode(Plate(20,20,5,21)),"bearing land");
		rejects(() -> LevelingFootCodec.encode(Foot(70,20,m16,21,115,90,Nominal,NoCollar)),"meet the pad");
		rejects(() -> LevelingFootCodec.encode(Foot(70,20,m16,20,115,116,Nominal,NoCollar)),"mounting datum");
		rejects(() -> LevelingFootCodec.encode(Screw(m16.mating(),95,Nominal)),"external thread");
		rejects(() -> LevelingFootCodec.encode(Nut(Hexagon(12),19,m24)),"around its thread");
		rejects(() -> LevelingFootCodec.decode('{"version":"1","part":{}}'),"finite number");
		rejects(() -> LevelingFootCodec.encode(Screw(FastenerThreads.metric(1,2),95,Nominal)),"root diameter");
		rejects(() -> new LevelingFoot(new machinekit.component.ComponentValues().setNumber("recessDepth",160)),
			"Unknown component input");
		var registry = MachineKitComponents.standard();
		check(!registry.has("machinekit.structural.levelling-foot"),"old router recipe removed");
		check(!registry.has("machinekit.structural.leveling-seat"),"old seat recipe removed");
		check(!registry.has("machinekit.milling.leveling-foot"),"old mill recipe removed");
		installedMillFeet();
		Sys.println('LevelingFootTests: $assertions assertions passed');
	}
}
