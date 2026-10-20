import materia.tooling.casting.BoxMoldPlanner;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CoreSupport.CoreSupportMode;
import materia.tooling.casting.CoreSupport.CoreSupportKind;
import materia.tooling.casting.CoreSupports;
import materia.tooling.casting.BoxMoldCodec;
import machinekit.assembly.PosedParts;
import machinekit.tooling.BoxMoldCad;
import machinekit.tooling.ToolingBlock;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import materia.assembly.AssemblyFrames as F;

class BoxMoldTests {
	static var assertions = 0;

	static function check(ok:Bool, label:String):Void {
		assertions++;
		if (!ok)
			throw label;
	}

	static function rejects(fn:Void->Void, label:String):Void {
		var caught = false;
		try
			fn()
		catch (e:Dynamic) {
			caught = true;
			Sys.println("Expected rejection: " + Std.string(e));
		}
		check(caught, label);
	}

	public static function run():Void {
		for (t in [18.0, 17.8])
			for (rotated in [false, true])
				for (bed in [false, true]) {
					var d = SinglePocketSetup.document(t, rotated, bed),
						p = BoxMoldPlanner.plan(d),
						report = BoxMoldCad.review(p);
					var encoded = BoxMoldCodec.encode(p);
					check(BoxMoldCodec.encode(BoxMoldCodec.decode(encoded)) == encoded, "mold canonical recipe restoration");
					var local = SinglePocketSetup.foundation(bed).body.geometry(),
						original = PosedParts.moved(local, d.setup.sourceToMold);
					local.close();
					var actual = BoxMoldCad.reconstructed(p),
						missing = original.subtractAll([actual]),
						excess = actual.subtractAll([original]);
					check(missing.volume() + excess.volume() < 1e-5, "actual foundation CAD matches mold reconstruction");
					missing.close();
					excess.close();
					original.close();
					actual.close();
					check(report.symmetricDifferenceMm3 < 1e-5, "casting symmetric difference");
					check(p.panels.members.length == 5 && p.cores.length == 1 && p.fixtures.length == 1, "complete box kit");
					check(Math.abs(p.cores[0].cavity.height - 60) < 1e-6, "cut tool clipped to source envelope");
					var c = p.cores[0];
					check(Math.abs(c.foam.width + 2 * c.coveringThickness - c.cavity.width) < 1e-8, "covering is included in cavity size");
					check(c.cutting.wireDirection.y == 1 && c.cutting.block.depth == c.foam.depth, "ruled block wire definition");
					check(p.panels.members[0].panel.stock.measuredThickness == t, "measured thickness preserved");
					check(Math.abs(p.cavity.width - (rotated ? 180 : 240)) < 1e-6, "setup rotates cavity and all tooling");
					check(report.assembly.assemblyOrder[0] == "bed", "bed installs before supported members");
					check(report.assembly.removalOrder.length == (bed ? 7 : 9) && report.demolding.removalOrder.length == (bed ? 7 : 10),
						"support geometry participates in graphs");
					check(p.cores[0].support.kind == (bed ? CoreSupportKind.BedSocket : CoreSupportKind.Bridge), "support selected from opening");
					check(p.supportPanels.length == (bed ? 0 : 4), "support templates realized");
					if (bed) {
						var foot = p.cores[0].support.foot;
						check(foot != null && foot.foam.z < p.cavity.z, "locating foot embedded below bed without extending cavity");
						check(report.demolding.removalOrder.indexOf("bed") < report.demolding.removalOrder.indexOf(p.cores[0].id),
							"bed released before foot withdrawal");
					} else {
						check(report.assemblyGroups.length == 1 && report.assemblyGroups[0].members.indexOf("core-pocket") >= 0,
							"prebonded hanging unit installed together");
						check(report.rigidGroups.length == 1
							&& report.demolding.removalOrder.indexOf("core-pocket-bridge-carrier") < report.demolding.removalOrder.indexOf("core-pocket"),
							"bonded fixture removed as one unit before core");
						check(report.demolding.removalOrder.indexOf("core-pocket") < report.demolding.removalOrder.indexOf("cured-casting"),
							"suspended core removed before lifting casting");
					}
					var saved = BoxMoldCad.assembly(p).encode(),
						restored = MachineAssembly.decode(saved);
					restored.validate();
					check(restored.encode() == saved, "saved recipes restore complete assembly");
					check(report.validation.length > 0 && !report.fabrication.ready, "physical qualification pending");
				}
		var metric = SinglePocketSetup.document();
		metric.input.inserts[0].holes = materia.casting.InsertHoleGeometry.through(6.8,metric.input.inserts[0].height);
		metric.setup.sourceFingerprint = materia.tooling.ToolingCodec.inputFingerprint(metric.input);
		metric.recipe.hardware.insertBoreDiameter = 6.8;
		metric.recipe.hardware.insertScrewDiameter = 8;
		metric.recipe.hardware.insertClearanceDiameter = 9;
		metric.recipe.hardware.insertThreadSpec = "M8";
		metric.recipe.hardware.supportScrewDiameter = 8;
		metric.recipe.hardware.supportClearanceDiameter = 9;
		var selected = BoxMoldPlanner.plan(metric);
		var hardware = BoxMoldCad.assembly(selected);
		var screws = 0;
		for (row in hardware.components())
			if (Std.isOfType(row.component, machinekit.standard.SocketHeadCapScrew)) {
				var screw:machinekit.standard.SocketHeadCapScrew = cast row.component;
				check(screw.diameter == 8, "CAD hardware follows recipe M8 selection");
				screws++;
			}
		check(screws == 3, "insert screw and both bridge screws use selected diameter");
		var p = BoxMoldPlanner.plan(SinglePocketSetup.document());
		p.penetrations = p.penetrations.filter(x -> x.operationId != "insert-fixing");
		rejects(() -> BoxMoldCad.penetrations(p), "unclassified hole fails");
		p = BoxMoldPlanner.plan(SinglePocketSetup.document());
		p.penetrations.filter(x -> x.operationId == "insert-fixing")[0].sealMargin = 19;
		rejects(() -> BoxMoldCad.penetrations(p), "seal margin fails");
		p = BoxMoldPlanner.plan(SinglePocketSetup.document());
		p.panels.members.filter(m -> m.panel.id == "south")[0].withdrawal = {x: 0, y: 1, z: 0};
		p.panels.members.filter(m -> m.panel.id == "north")[0].withdrawal = {x: 0, y: -1, z: 0};
		rejects(() -> BoxMoldCad.review(p), "trapped opposite walls rejected");
		var d = SinglePocketSetup.document(),
			options = CoreSupports.defaults(d.recipe);
		options.mode = CoreSupportMode.BedSocket;
		rejects(() -> BoxMoldPlanner.plan(d, options), "bed support cannot bridge a solid casting floor");
		d = SinglePocketSetup.document(18, false, true);
		options = CoreSupports.defaults(d.recipe);
		options.mode = CoreSupportMode.Bridge;
		rejects(() -> BoxMoldPlanner.plan(d, options), "bridge cannot pass through a closed casting top");
		options = CoreSupports.defaults(d.recipe);
		options.embedDepth = 18;
		rejects(() -> BoxMoldPlanner.plan(d, options), "socket cannot consume bed stock");
		p = BoxMoldPlanner.plan(SinglePocketSetup.document());
		p.cores[0].cavity.width -= 2;
		p.cores[0].foam.width -= 2;
		p.cores[0].cutting.block.width -= 2;
		rejects(() -> BoxMoldCad.review(p), "missing core geometry rejected");
		p = BoxMoldPlanner.plan(SinglePocketSetup.document());
		var wall = p.panels.members.filter(m -> m.panel.id == "south")[0];
		wall.pose.y -= 0.5;
		var joint = p.panels.joints.filter(j -> j.movingId == "south")[0];
		joint.fixedFrame = F.compose(F.inverse(p.panels.members[0].pose), wall.pose);
		rejects(() -> BoxMoldCad.review(p), "open wall seam rejected by solid coverage");
		var block = new ToolingBlock(new ComponentValues().setNumber("width", 20)
			.setNumber("depth", 30)
			.setNumber("height", 40)
			.setToken("material", "EPS tooling"));
		var native = block.geometry();
		check(Math.abs(native.volume() - 24000) < 1e-6, "core block CAD volume");
		native.close();
		Sys.println('BoxMoldTests: $assertions assertions passed');
	}
}
