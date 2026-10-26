package machinekit.tooling;

import materia.tooling.casting.CastingMoldPlan.MoldFixtures;
import materia.tooling.casting.MoldCores;
import materia.tooling.casting.CastingMoldPlanner;

import haxe.Json;
import materia.tooling.casting.CastingMoldPlan;
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
		if (review.fingerprint != CastingMoldPlanner.fingerprint(p))
			throw "Stale casting mold review";
		sys.FileSystem.createDirectory(dir);
		data(dir + "/review.json", {
			phase: 4,
			fingerprint: CastingMoldPlanner.fingerprint(p),
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
			for (job in MoldCores.cuttingJobs(c)) {
				var cutting = job.cutting,id = job.id;
				data(dir + "/" + id + ".cutting.json",MoldCores.cuttingJobRecord(c,cutting));
				var part = ToolingVolume.fromCoreCutting(id,cutting).geometry();
				step(dir + "/" + id + ".step", part);

				var volume=ToolingVolume.fromCoreCutting(id,cutting).volume;
				var drawing=ToolingSectionDrawing.byDirection(volume,cutting.wireDirection).drawing;
				text(dir + "/" + id + ".svg", Svg.write(drawing));
				text(dir + "/" + id + ".dxf", Dxf.write(drawing));
			}
		for (seal in p.seals) step(dir+"/"+seal.volume.id+".step",ToolingVolume.geometryOf(seal.volume));
		for (f in p.formers) {
			var part = ToolingVolume.geometryOf(f.volume);
			step(dir + "/" + f.volume.id + ".step", part);
		}
		for (volume in MoldFixtures.allVolumes(p)) {
			var part=ToolingVolume.geometryOf(volume);
			step(dir+"/"+volume.id+".step",part);
		}
		var casting = CastingMoldCad.reconstructed(p);
		try casting.shape.exportStep(dir + "/reconstructed-casting.step") catch (error:Dynamic) {
			casting.close();
			throw error;
		}
		var castingMesh = AssemblyPreview.part(CastingMoldView.REFERENCE_ID, "Finished casting", casting,
			materia.project.MaterialLibrary.fromSpec("mineral cast"));
		var assembled = CastingMoldCad.assembly(p),
			saved = assembled.encode();
		var restored = MachineAssembly.decode(saved);
		restored.validate();
		if (restored.encode() != saved)
			throw "Assembly recipe did not restore exactly";
		text(dir + "/mold.assembly.json", saved);
		scene(dir, "mold", assembled, p, review, castingMesh);
		var coreList = [for (c in p.cores) switch c.support.kind {
			case RoutedSocket: c.id+": "+c.layers.length+" layers, routed socket "+c.support.socketOperationId;
			case RegisteredFlange: c.id+": "+c.layers.length+" layers, externally clamped registered flange at "+MoldCores.receiverId(c);
			case Bridge: c.id+": "+c.layers.length+" layers, carrier "+MoldCores.carrierId(c)+", shared "+MoldCores.bridgeId(c);
				default:throw "Unknown EPS support kind: "+c.id;
		}];
		for(core in p.cores)if(core.support.assemblyByLayer)coreList.push(core.id
			+": install the supplied stock layers around fixtures in the checked order, then bond and finish their covering; keep with the casting for lift and break out through "
			+MoldCores.removal(core)+" afterwards. Release bed/carrier adhesive before tooling withdrawal; qualify these manual processes.");
		text(dir
			+ "/procedure.md",
			"# Casting mold review — "
			+ p.document.input.id
			+ "\n\nREVIEW ONLY. Not approved for fabrication.\n\n"
			+ "1. Prepare all tooling with the recipe release system before fitting any steel insert: seal exposed board faces, cure, coat, buff and dry, then optional PVA and drying. Keep insert and anchor bonding faces uncoated and mask adjacent contacts. Qualify stock, EPS, adhesive/covering/release compatibility, compaction, closure and brace loads before pouring.\n"
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

	/** Viewport-only generation shares the exported scene, without writing a kit. */
	public static function artifact(plan:CastingMoldPlan):haxe.io.Bytes {
		var casting = CastingMoldCad.reconstructed(plan);
		var mesh = AssemblyPreview.part(CastingMoldView.REFERENCE_ID, "Finished casting", casting,
			materia.project.MaterialLibrary.fromSpec("mineral cast"));
		return SceneArtifact.encode(sceneData("mold", CastingMoldCad.assembly(plan), plan, null, mesh));
	}

	static function sceneData(id:String, a:MachineAssembly, plan:CastingMoldPlan,
			review:Null<CastingMoldCad.CastingMoldReview>, castingMesh:materia.project.SceneArtifact.SceneArtifactPart):materia.project.SceneArtifact.SceneArtifactData {
		var sceneData=AssemblyPreview.scene(a,id);
		CastingMoldView.geometry(sceneData, castingMesh);
		if (review == null) {
			if (sceneData.assemblyDefinition == null) throw "Mold scene requires an assembly";
			sceneData.presentation = CastingMoldCad.presentation(plan,sceneData.assemblyDefinition);
			sceneData.recipeDiagnostics = ["Viewport preview; full geometric review and fabrication verification have not been run."];
		} else sceneData.presentation=ToolingPresentation.casting(plan,sceneData,review);
		var embedded = [for (insert in plan.document.input.inserts)
			plan.construction == SinglePocket ? insert.id : materia.tooling.ToolingIds.derive("insert-", insert.id)];
		for (anchor in CastingMoldCad.anchorComponents(plan)) embedded.push(anchor.id);
		var liftingIds = [for (insert in plan.document.input.inserts)
			if (insert.role == materia.casting.InsertSpec.InsertRole.LiftingMount)
				plan.construction == SinglePocket ? insert.id : materia.tooling.ToolingIds.derive("insert-", insert.id)];
		var moldToService = materia.assembly.AssemblyFrames.compose(plan.options.serviceOrientation,
			materia.assembly.AssemblyFrames.inverse(plan.document.setup.sourceToMold));
		CastingMoldView.presentation(sceneData, embedded, moldToService, liftingIds);
		return sceneData;
	}

	static function scene(dir:String, id:String, a:MachineAssembly, plan:CastingMoldPlan,
			review:CastingMoldCad.CastingMoldReview, castingMesh:materia.project.SceneArtifact.SceneArtifactPart):Void {
		var record = sceneData(id, a, plan, review, castingMesh);
		sys.io.File.saveBytes(dir+"/"+id+".mtrg",SceneArtifact.encode(record));
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
