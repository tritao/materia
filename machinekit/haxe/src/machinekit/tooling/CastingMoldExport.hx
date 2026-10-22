package machinekit.tooling;

import haxe.Json;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldCodec;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.casting.CastingProfileGeometry;
import materia.tooling.casting.CoreSupports;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import materia.project.SceneArtifact;
import cadkit.drawing.Svg;
import cadkit.drawing.Dxf;
import cadkit.drawing.Drawing2D;
import cadkit.drawing.DrawingEntity;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;

/** Review outputs only. Existing CadKit writers render the exact panel operations. */
class CastingMoldExport {
	static function step(path:String, part:cadkit.modeling.Part):Void {
		try
			part.shape.exportStep(path)
		catch (e:Dynamic) {
			part.close();
			throw e;
		}
		part.close();
	}

	static function text(path:String, value:String):Void
		sys.io.File.saveContent(path, value);

	static function data(path:String, value:Dynamic):Void
		text(path, Json.stringify(value, null, "\t") + "\n");

	public static function write(p:CastingMoldPlan, dir:String, review:machinekit.tooling.CastingMoldCad.CastingMoldReview):Void {
		if (review.fingerprint != CastingMoldCodec.fingerprint(p))
			throw "Stale casting mold review";
		sys.FileSystem.createDirectory(dir);
		var encoded = CastingMoldCodec.encode(p);
		CastingMoldCad.decode(encoded);
		text(dir + "/mold-plan.json", encoded);
		data(dir + "/review.json", {
			phase: 4,
			fingerprint: CastingMoldCodec.fingerprint(p),
			geometry: review,
			changeRequests: p.changeRequests,
			limitations: "REVIEW ONLY. Datum fixtures require qualified attachments; steel bores, temporary masks and fixture pins are modeled in the swept graphs. Release accessible fasteners and adhesive as applicable; withdraw masks and fixture pins in the checked order. Compaction, anchorage, precision replication/grouting, loads and handling remain pending."
		});
		data(dir + "/feature-inventory.json", p.features);
		data(dir + "/demolding-steps.json", review.demoldingSteps);
		data(dir + "/handling.json", review.handling);
		data(dir + "/diagnostics.json", ToolingDiagnostics.fromValidation(p.validation).items);
		data(dir + "/design-change-requests.json", p.changeRequests);
		data(dir + "/process-options.json", p.options);
		for (m in p.panels.members.concat(p.supportPanels)) {
			var base = PanelDrawing.drawing(m.panel),
				entities = base.entities();
			entities.push(Label(Text, new Point2(0, -22), 3, "REVIEW ONLY: geometry, pending physical qualification"));
			var drawing = new Drawing2D(entities);
			text(dir + "/" + m.panel.id + ".svg", Svg.write(drawing));
			text(dir + "/" + m.panel.id + ".dxf", Dxf.write(drawing));
			text(dir + "/" + m.panel.id + ".panel.json", PanelCodec.encode(m.panel));
			var part = ToolingPanel.fromPlan(m.panel).geometry();
			step(dir + "/" + m.panel.id + ".step", part);
		}
		for (c in p.cores)
			for (index in 0...c.foamLayers.length) {
				var b = c.foamLayers[index],
					id = materia.tooling.ToolingIds.derive("", c.id, "-layer-" + index),
					cutting = CoreSupports.cutting(b, p.document.recipe.coreWireAxis);
				data(dir + "/" + id + ".cutting.json", {
					featureId: c.featureId,
					coreId: c.id,
					cutting: cutting,
					adhesiveThickness: c.adhesiveThickness,
					coveringThickness: c.coveringThickness
				});
				var local = cutting.block, part = ToolingVolume.geometryOf({
					id: id,
					material: EPSStock,
					cuts: [],
					regions: [],
					profiles: [],
					pieces: [local]
				});
				step(dir + "/" + id + ".step", part);

				var direction = cutting.wireDirection;
				var profileWidth = direction.x > 0.5 ? b.depth : b.width;
				var profileHeight = direction.z > 0.5 ? b.depth : b.height;
				var span = direction.x > 0.5 ? b.width : direction.y > 0.5 ? b.depth : b.height;
				var axis = direction.x > 0.5 ? "+X" : direction.y > 0.5 ? "+Y" : "+Z";
				var drawing = new Drawing2D([
					Curve(Outline, Segment.line(new Point2(0, 0), new Point2(profileWidth, 0))),
					Curve(Outline, Segment.line(new Point2(profileWidth, 0), new Point2(profileWidth, profileHeight))),
					Curve(Outline, Segment.line(new Point2(profileWidth, profileHeight), new Point2(0, profileHeight))),
					Curve(Outline, Segment.line(new Point2(0, profileHeight), new Point2(0, 0))),
					Label(Text, new Point2(0, profileHeight + 8), 3, id + " wire " + axis + "; span " + span + " mm; no kerf compensation"),
					Label(Text, new Point2(0, -8), 3, "REVIEW ONLY: EPS grade and adhesive unqualified")
				]);
				text(dir + "/" + id + ".svg", Svg.write(drawing));
				text(dir + "/" + id + ".dxf", Dxf.write(drawing));
			}
		for (f in p.formers) {
			var part = ToolingVolume.geometryOf(f.volume);
			step(dir + "/" + f.volume.id + ".step", part);
			text(dir + "/" + f.volume.id + ".volume.json", materia.tooling.casting.MoldVolumeCodec.encode(f.volume));
		}
		for (f in p.fixtures) {
			var part = ToolingVolume.geometryOf(f.datum);
			step(dir + "/" + f.datum.id + ".step", part);
			text(dir + "/" + f.datum.id + ".volume.json", materia.tooling.casting.MoldVolumeCodec.encode(f.datum));
		}
		var casting = CastingMoldCad.reconstructed(p);
		step(dir + "/reconstructed-casting.step", casting);
		var assembled = CastingMoldCad.assembly(p),
			saved = assembled.encode();
		var restored = MachineAssembly.decode(saved);
		restored.validate();
		if (restored.encode() != saved)
			throw "Assembly recipe did not restore exactly";
		text(dir + "/mold.assembly.json", saved);
		scene(dir, "mold", assembled, p, review);
		var coreList = [
			for (c in p.cores)
				c.id + ": " + c.foamLayers.length + " layers, carrier " + c.carrierId + ", shared " + c.bridgeId
		];
		text(dir
			+ "/procedure.md",
			"# Casting mold review — "
			+ p.document.input.id
			+ "\n\nREVIEW ONLY. Not approved for fabrication.\n\n"
			+ "1. Qualify stock, EPS, adhesive/covering/release compatibility, compaction, closure and brace loads before pouring.\n"
			+
			"2. Resolve all blocking validation records and design-change requests, including tooling fixture qualification, precision allowances and lifting/flip points. Datum fixtures place inserts nominally; they do not establish final precision.\n"
			+
			"3. Cut/laminate EPS with the specified interlayer glue lines. Permanently bond carriers to each shared bridge; prebond the row's cores to that fixture.\n\n"
			+ coreList.join("\n")
			+ "\n\n"
			+ "4. Geometric installation order: "
			+ review.assembly.assemblyOrder.join(" -> ")
			+
			". Fixture pins and bore masks are included in that order; fit loose clamp bolts afterwards. Clamp support rails and outer walls with qualified external closures.\n"
			+
			"5. Qualify lateral location and secure every insert to its bed-referenced nominal support. Resolve pending temporary threads/clamps and mask/protect bores. Seams and router-relief sealing profiles need a qualified process.\n"
			+
			"6. After cure, release accessible loose fixings and EPS/carrier adhesive as applicable. Pins and bore masks remain in the checked removal order. Geometric removal order: "
			+ review.demolding.removalOrder.join(" -> ")
			+ ". These are full straight-translation sweeps; mechanical release and EPS breakup are manual prerequisites.\n"
			+
			"7. Lift/flip only with qualified points/equipment. Replicate the designated rail/bearing seats against qualified masters; grout the column joint using its approved gap, jacking and cure-before-torque process. These allowances and processes are pending, and source geometry was not altered.\n");
	}

	static function scene(dir:String, id:String, a:MachineAssembly, plan:CastingMoldPlan, review:CastingMoldCad.CastingMoldReview):Void {
		var sceneData=AssemblyPreview.scene(a,id);
		sceneData.presentation=ToolingPresentation.casting(plan,sceneData,review);
		sys.io.File.saveBytes(dir+"/"+id+".mtrg",SceneArtifact.encode(sceneData));
		var rows:Array<Dynamic> = [], poses = a.solvedPoses();
		for (row in a.components()) {
			var mesh = AssemblyPreview.part(row.id, row.id, row.component.geometry(), row.component.materialId),
				stem = id + "-" + row.id;
			sys.io.File.saveBytes(dir + "/" + stem + ".vertices", mesh.vertices);
			sys.io.File.saveBytes(dir + "/" + stem + ".indices", mesh.indices);
			rows.push({id: stem, pose: poses.get(row.id), color: [mesh.red, mesh.green, mesh.blue]});
		}
		data(dir + "/" + id + "-meshes.json", rows);
	}
}
