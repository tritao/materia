import cadkit.threads.ThreadSpec;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.HexBolt;
import machinekit.standard.PropertyClass;
import machinekit.standard.PropertyClass.PropertyClasses;
import machinekit.standard.FastenerTightening;
import machinekit.standard.ScrewFastener;
import machinekit.component.ComponentValues;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LeadScrewThread.LeadScrewThreadFamily;
import machinekit.motion.LeadScrewThread.LeadScrewHand;

/** Drives, mechanical classes and cited tightening data are stated independently of B-rep geometry. */
class FastenerDataTests {
	static var assertions = 0;

	public static function main():Void run();

	public static function run():Void {
		assertions = 0;
		for (size in SocketHeadCapScrew.catalog().designations()) {
			var screw = SocketHeadCapScrew.metric(size, 40);
			checkFastener(screw, size);
			switch screw.drive {
				case HexSocket(mm): near(mm, screw.spec.socketSize, "socket size from screw");
				default: throw "Cap screw needs a hex socket";
			}
			for (token in PropertyClasses.tokens()) {
				var type = SocketHeadCapScrew.recipeType();
				var values = new ComponentValues().setToken("size", size).setToken("propertyClass", token);
				var first:SocketHeadCapScrew = cast type.create(values);
				var restored:SocketHeadCapScrew = cast type.create(first.values());
				check(first.propertyClass == PropertyClasses.parse(token), "recipe states property class");
				check(first.bom.material == PropertyClasses.material(first.propertyClass), "class supplies material");
				check(first.bom.partNumber == restored.bom.partNumber, "class survives recipe round trip");
				check(first.tightening().torqueNm > 0, "cited row for each offered class");
			}
		}
		for (size in HexBolt.catalog().designations()) {
			var bolt = HexBolt.metric(size, 30);
			checkFastener(bolt, size);
			switch bolt.drive {
				case HexHead(mm): near(mm, bolt.spec.acrossFlats, "wrench size from bolt");
				default: throw "Bolt needs a hex head";
			}
		}
		var steel = SocketHeadCapScrew.metric("M6", 25).tightening();
		near(steel.torqueNm, 16.5, "published M6 class 12.9 assembly torque, Nm");
		near(steel.preloadN, 15650, "published M6 class 12.9 preload, N");
		near(steel.friction, 0.125, "steel row friction");
		var stainless = HexBolt.metric("M6", 25, A270).tightening();
		near(stainless.torqueNm, 5.9, "published A2-70 torque");
		near(stainless.preloadN, 6850, "stainless kN converted to N");
		near(stainless.friction, 0.1, "stainless row friction");
		for (key in FastenerTightening.catalog().designations())
			check(StringTools.startsWith(FastenerTightening.catalog().metadata(key).source, "https://"), "row has published source");
		var fine = machinekit.standard.FastenerThreads.metric(6, 0.75);
		var metric = new ThreadSpec("M6", 6, 1);
		check(metric.toleranceClass == "6g" && metric.mating().toleranceClass == "6H", "metric mating tolerances");
		check(metric.matches(metric.mating()) && !metric.matches(fine), "nominal mating dimensions and fine pitch");
		check(fine.designation == "M6x0.75", "fine designation retains pitch");
		var absent = false;
		try { var row = FastenerTightening.forThread(fine, Steel129); } catch (error:Dynamic)
			absent = Std.string(error).indexOf("No published tightening row") >= 0;
		check(absent, "absent fine row reports instead of inventing a torque");
		var oldInputRejected = false;
		try { var wrong = SocketHeadCapScrew.recipeType().create(new ComponentValues().setToken("material", "steel C45")); }
		catch (error:Dynamic) oldInputRejected = Std.string(error).indexOf("Unknown input") >= 0;
		check(oldInputRejected, "obsolete free-text material recipe input is rejected");
		var inconsistentMaterialRejected = false;
		try SocketHeadCapScrew.metric("M6", 25).setMaterial("aluminium 6061")
		catch (error:Dynamic) inconsistentMaterialRejected = Std.string(error).indexOf("determined by property class") >= 0;
		check(inconsistentMaterialRejected, "mechanical class cannot contradict physical material");
		var saved = new machinekit.assembly.MachineAssembly();
		saved.addComponent("screw", SocketHeadCapScrew.metric("M6", 25, Steel109));
		saved.addComponent("bolt", HexBolt.metric("M6", 25, A270));
		var loaded = machinekit.assembly.MachineAssembly.decode(saved.encode());
		for (entry in loaded.components()) {
			var fastener:ScrewFastener = cast entry.component;
			check(fastener.fastenerClass() == (entry.id == "screw" ? Steel109 : A270), "class survives machine format");
		}
		var drive = SocketHeadCapScrew.metric("M6", 25).drive;
		var document = new cadkit.parametric.Document();
		var root = machinekit.document.MachineAssemblyDocuments.defineAssembly(document, saved);
		var reopened = cadkit.parametric.DocumentCodec.decode(cadkit.parametric.DocumentCodec.encode(document));
		var rebuilt = machinekit.document.MachineAssemblyDocuments.rebuildAssembly(reopened.element(root.id));
		for (entry in rebuilt.components()) {
			var fastener:ScrewFastener = cast entry.component;
			check(fastener.fastenerClass() == (entry.id == "screw" ? Steel109 : A270), "class survives CadKit document");
		}
		reopened.close();
		document.close();
		var decodedDrive:machinekit.standard.ScrewDrive = haxeon.wire.JsonWire.decode(haxeon.wire.JsonWire.encode(drive));
		switch decodedDrive {
			case HexSocket(mm): near(mm, 5, "drive size survives wire format");
			default: throw "Drive changed in wire format";
		}
		for (family in [MetricTrapezoidal, Acme, Ball]) for (hand in [RightHand, LeftHand]) {
			// The synthetic ball race states its root; nominal thread facts do not depend on it.
			var raceRoot:Null<Float> = family == Ball ? 10.0 : null;
			var lead = new LeadScrewThread(family, 12, 2, 4, hand, raceRoot);
			var spec = lead.threadSpec();
			near(spec.pitch * spec.starts, lead.lead, "shared lead");
			check(spec.hand == (hand == RightHand ? materia.project.ThreadSpecData.ThreadHand.RightHand
				: materia.project.ThreadSpecData.ThreadHand.LeftHand), "shared hand");
			check(spec.internal == false, "lead screw is external");
		}
		trace('Fastener data: $assertions assertions passed');
	}

	static function checkFastener(fastener:ScrewFastener, size:String):Void {
		var thread = fastener.threadSpec();
		check(thread.designation == size && !thread.internal, "catalog external thread");
		check(fastener.screwDrive() != null, "catalog drive");
		check(fastener.tightening().torqueNm > 0 && fastener.tightening().preloadN > 0, "catalog default tightening");
	}
	static function near(a:Float, b:Float, message:String):Void check(Math.abs(a - b) < 1e-6, message);
	static function check(value:Bool, message:String):Void {
		assertions++;
		if (!value) throw message;
	}
}
