import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.PosedParts;
import machinekit.component.Bom;
import machinekit.structural.InsertBodies;
import machinekit.tooling.SocketFixtureCad;
import machinekit.tooling.ToolingSourceCasting;
import machinekit.tooling.SweptRemoval;
import materia.tooling.casting.InsertFixturePlanner;
import materia.tooling.casting.InsertFixturePlanner.InsertPlatePlan;
import materia.tooling.ToolingValidation.ToolingReadiness;
import materia.tooling.ToolingCodec;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelAssemblyPlan;
import materia.assembly.AssemblyFrames as F;
import materia.project.SceneArtifact;

/** The coupon uses the same source inserts, neutral carrier and exact graph as machine tooling. */
class HobberSocketFixture {
	static var assertions = 0;
	static function check(ok:Bool,message:String):Void {
		assertions++;
		if (!ok) throw message;
	}
	static function rejects(action:Void->Void,expected:String):Void {
		var message = "";
		try action() catch (error:Dynamic) message = Std.string(error);
		check(message.indexOf(expected) >= 0,'Expected "$expected", got "$message"');
	}
	static function threadPair(plan:InsertPlatePlan,a:String,b:String):Bool {
		return plan.sites.filter(site -> a == site.id && b == "insert-"+site.insertId
			|| b == site.id && a == "insert-"+site.insertId).length == 1;
	}

	static function main():Void {
		var root = Sys.getCwd();
		while (!sys.FileSystem.exists(root+"/machinekit/examples/flexspline-hobber/HobberSocketFixture.hx")) {
			var parent = haxe.io.Path.directory(root);
			if (parent == root || parent.length == 0) throw "Cannot find socket coupon source";
			root = parent;
		}
		var out = root+"/machinekit/examples/flexspline-hobber/build/socket-fixture";
		sys.FileSystem.createDirectory(out);
		var plan = HobberSocketFixtureSetup.plan(), document = HobberSocketFixtureSetup.document();
		check(plan.sites.length == 2 && plan.masks.length == 0,"Both coupon sockets use their source fixing threads without masks");
		check(plan.penetrations.filter(pen -> pen.kind == InsertSealed && pen.sourceHoleId != null && pen.sealingFace != null).length == plan.sites.length,
			"Every insert-sealed carrier penetration traces to its source hole and sealing face");
		for (length in [20.0,50.0])
			rejects(() -> InsertFixturePlanner.plate(document,HobberSocketFixtureSetup.options(document,[length])),
				"No stock fixture screw satisfies");
		check(plan.validation.filter(row -> row.rule == "tooling.fixture.accuracy.socket-fixture").length == 1,
			"Fixture accuracy remains an explicit pending check");
		var validation = plan.validation.concat([
			ToolingReadiness.pending("tooling.fixture.wall-closure",["coupon-walls"],"Wall closure, resin pressure and leak qualification remain pending"),
			ToolingReadiness.pending("tooling.fixture.process-retention",["socket0","socket1"],"Trial cure, release and mineral-cast anchorage with actual materials")
		]);
		check(!ToolingReadiness.evaluate(validation).ready,"Geometry success cannot qualify this physical process");
		PanelAssemblyPlan.validate(plan.assembly);
		for (member in plan.assembly.members)
			check(PanelCodec.encode(PanelCodec.decode(PanelCodec.encode(member.panel))) == PanelCodec.encode(member.panel),"Panel record roundtrip");
		var assembly = SocketFixtureCad.assembly(plan,true), parts = assembly.components(), poses = assembly.solvedPoses();
		check(assembly.validate().length == 0,"Fixture assembly validates");
		check(MachineAssembly.decode(assembly.encode()).components().length == parts.length,"Fixture assembly recipes roundtrip");
		var removal = SocketFixtureCad.check(plan), cavity = SocketFixtureCad.cavity(plan);
		check(removal.removal.removalOrder.length == parts.length-plan.document.input.inserts.length-1,
			"Every removable component belongs to the computed graph");
		var cache = new PosedParts(), exactPairs = 0;
		var retainedScrewVolume=0.0;
		var castingRow = parts.filter(row -> row.id == "cured-casting")[0];
		var castShape = cache.posed(castingRow.component,poses.get(castingRow.id));
		try {
			for (i in 0...parts.length) for (j in i+1...parts.length) {
				var a = parts[i], b = parts[j];
				if (threadPair(plan,a.id,b.id)) continue; // Declared thread engagement, released before translation.
				if (PosedParts.apart(cache.box(a.component,poses.get(a.id)),cache.box(b.component,poses.get(b.id)),0)) continue;
				exactPairs++;
				var first = cache.posed(a.component,poses.get(a.id)), second = cache.posed(b.component,poses.get(b.id));
				first.shape.setThreadedFaces([]); second.shape.setThreadedFaces([]);
				try check(SweptRemoval.intersectionVolume(first,second) < 0.001,'Initial overlap ${a.id}/${b.id}')
				catch (error:Dynamic) {first.close(); second.close(); throw error;}
				first.close(); second.close();
			}
			var excludedVolume = 0.0;
			for (site in plan.sites) {
				var source = plan.document.input.inserts.filter(insert -> insert.id == site.insertId)[0];
				var hole = source.holes.filter(hole -> hole.id == site.sourceHoleId)[0];
				var mouth = F.compose(poses.get("insert-"+source.id),hole.frame);
				check(Math.abs(mouth.x-site.entry.x)+Math.abs(mouth.y-site.entry.y)+Math.abs(mouth.z-site.entry.z) < 1e-7,
					"Source hole and fixture fixing are coaxial on the contact plane");
				check(hole.extent == Blind && hole.threadDepth == 6 && hole.depth == 25 && site.insertion >= site.engagement
					&& site.insertion+materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole).minTipGap <= hole.depth,
					"Stock fixing screw preserves the short thread, deeper bore and closed-end budget");
				var exterior = InsertBodies.part(source.body);
				var at = PosedParts.moved(exterior,F.compose(plan.document.setup.sourceToMold,source.pose)); exterior.close();
				try {
					check(SweptRemoval.intersectionVolume(castShape,at) < 0.001,"Casting excludes the full exterior and protected blind bore");
					excludedVolume += at.volume();
				} catch (error:Dynamic) {at.close(); throw error;}
				at.close();
				var screw:machinekit.standard.SocketHeadCapScrew = cast parts.filter(row -> row.id == site.id)[0].component;
				var t = plan.document.recipe.panel.measuredThickness;
				// Straight hex-key approach uses the selected screw's full across-corners profile.
				var key = Part.prism(cadkit.modeling.Polygon.regular(6,screw.spec.socketSize),
					cavity.z-2*t-45,cavity.z-t-site.washerThickness-screw.spec.headHeight-0.1);
				var driver:Part;
				try driver = PosedParts.moved(key,F.translation(site.entry.x,site.entry.y,0))
				catch (error:Dynamic) {key.close(); throw error;}
				key.close();
				try for (id in [plan.assembly.members[0].panel.id,plan.assembly.members[1].panel.id,"cured-casting"]) {
					var row = parts.filter(row -> row.id == id)[0], body = cache.posed(row.component,poses.get(id));
					try check(SweptRemoval.intersectionVolume(driver,body) < 0.001,"Driver approach blocked by "+id)
					catch (error:Dynamic) {body.close(); throw error;}
					body.close();
				} catch (error:Dynamic) {driver.close(); throw error;}
				driver.close();
			}
			check(Math.abs(castShape.volume()+excludedVolume-cavity.width*cavity.depth*cavity.height) < 0.01,
				"Exact source exterior subtraction leaves no rectangular pocket around a crimped tail");
			var upper = plan.assembly.members[1], row = parts.filter(part -> part.id == upper.panel.id)[0];
			var plate = cache.posed(row.component,poses.get(row.id)), sweep = SweptRemoval.sweep(plate,new Vector(0,0,-removal.removal.travelMm));
			plate.close();
			var heldVolume = 0.0;
			try for (site in plan.sites) {
				var screw = parts.filter(part -> part.id == site.id)[0], held = cache.posed(screw.component,poses.get(screw.id));
				held.shape.setThreadedFaces([]);
				try heldVolume += SweptRemoval.intersectionVolume(sweep,held)
				catch (error:Dynamic) {held.close(); throw error;}
				held.close();
				check(removal.releaseBeforeTranslation.indexOf(site.id) >= 0,"Source fixing screw must be released before withdrawal");
			} catch (error:Dynamic) {sweep.close(); throw error;}
			sweep.close();
			retainedScrewVolume=heldVolume;
			check(heldVolume > 1,"Negative control: retained positioning screw heads block plate withdrawal");
			for (id in ["south-wall","north-wall","west-wall","east-wall"])
				check(removal.removal.removalOrder.indexOf(id) >= 0,"Wall withdrawal belongs to the exact graph");
		} catch (error:Dynamic) {castShape.close(); cache.close(); throw error;}
		castShape.close(); cache.close();
		var reusable = new Bom(), permanent = new Bom();
		for (row in parts) {
			if (plan.document.input.inserts.filter(insert -> row.id == "insert-"+insert.id).length == 1) permanent.addComponent(row.component);
			else if (row.id != "cured-casting") reusable.addComponent(row.component);
		}
		var mix = ToolingSourceCasting.fromInput(plan.document.input).massProperties().mass;
		var steps:Array<{id:String,kind:String,members:Array<String>,after:Array<String>}> = [
			{id:"cure",kind:"process-evidence-pending",members:["cured-casting"],after:[]},
			{id:"release-threads",kind:"unscrew",members:removal.releaseBeforeTranslation,after:["cure"]}
		];
		var previous = "release-threads";
		for (id in removal.removal.removalOrder) {
			steps.push({id:"withdraw-"+id,kind:"translate",members:[id],after:[previous]});
			previous = "withdraw-"+id;
		}
		sys.io.File.saveContent(out+"/fixture-plan.json",ToolingCodec.canonical(plan)+"\n");
		sys.io.File.saveContent(out+"/demolding-steps.json",haxe.Json.stringify({steps:steps,geometry:removal.removal},null,"  "));
		sys.io.File.saveContent(out+"/bom.json",haxe.Json.stringify({permanentSockets:permanent.lines(),mixKg:mix,reusableTooling:reusable.lines()},null,"  "));
		sys.io.File.saveContent(out+"/fixture.assembly.json",assembly.encode());
		for (member in plan.assembly.members) {
			var drawing = machinekit.tooling.PanelDrawing.drawing(member.panel);
			sys.io.File.saveContent(out+"/"+member.panel.id+".panel.json",PanelCodec.encode(member.panel));
			sys.io.File.saveContent(out+"/"+member.panel.id+".svg",cadkit.drawing.Svg.write(drawing));
			sys.io.File.saveContent(out+"/"+member.panel.id+".dxf",cadkit.drawing.Dxf.write(drawing));
		}
		var scene = AssemblyPreview.scene(assembly,"blind-socket-fixture");
		var views=SocketFixtureCad.presentation(plan,scene.assemblyDefinition);scene.presentation=views;
		// CAD face winding must agree with its outward normals before the renderer culls back faces.
		for(part in scene.parts)for(triangle in 0...Std.int(part.indexCount/3)) {
			var ids=[for(corner in 0...3)part.indices.getInt32((triangle*3+corner)*4)];
			var a=[for(axis in 0...3)part.vertices.getDouble(ids[0]*24+axis*8)];
			var b=[for(axis in 0...3)part.vertices.getDouble(ids[1]*24+axis*8)-a[axis]];
			var c=[for(axis in 0...3)part.vertices.getDouble(ids[2]*24+axis*8)-a[axis]];
			var normal=[b[1]*c[2]-b[2]*c[1],b[2]*c[0]-b[0]*c[2],b[0]*c[1]-b[1]*c[0]];
			for(id in ids) {
				var agreement=0.0;for(axis in 0...3)agreement+=normal[axis]*part.normals.getDouble(id*24+axis*8);
				if(agreement < -1e-12)throw "CAD triangle winding disagrees with its normal: "+part.id;
			}
		}
		check(true,"Fixture triangle winding agrees with outward shading normals");
		for(part in scene.parts)materia.project.MeshClosure.validate(part);
		check(true,"Fixture declared solids have closed consistently wound mesh boundaries");
		check(views.defaultView=="assembled"&&views.views.length==3,"Fixture exposes the assembled default and two inspection views");
		check(views.explode.length==assembly.components().length,"All fixture parts have presentation offsets");
		var engineering=assembly.encode(),bom=haxe.Json.stringify(assembly.billOfMaterials().lines());
		var cursor=materia.presentation.PresentationEvaluator.reset(views);cursor.explode=1;
		var shown=materia.presentation.PresentationEvaluator.evaluate(scene.assemblyDefinition,assembly.solvedPoses(),views,cursor,"");
		for(member in assembly.components()) {
			var actual=shown.poses.get(member.id),source=assembly.solvedPoses().get(member.id);
			if(actual==null||source==null)throw "Missing fixture display pose";
			var dx=0.0,dy=0.0,dz=0.0;
			if(member.id==plan.assembly.members[0].panel.id)dz=-75;
			else if(member.id==plan.assembly.members[1].panel.id)dz=-35;
			else if(member.id=="cured-casting"){dx=SocketFixtureCad.cavity(plan).width+150;dz=80;}
			else if(member.id=="south-wall")dy=-100;
			else if(member.id=="north-wall")dy=100;
			else if(member.id=="west-wall")dx=-100;
			else if(member.id=="east-wall")dx=100;
			else if(plan.document.input.inserts.filter(i->member.id==materia.tooling.ToolingIds.derive("insert-",i.id)).length==1)dz=60;
			else dz=-70;
			check(Math.abs(actual.x-source.x-dx)<1e-8&&Math.abs(actual.y-source.y-dy)<1e-8&&Math.abs(actual.z-source.z-dz)<1e-8,
				"Slider endpoint preserves the former review displacement: "+member.id);
		}
		check(assembly.encode()==engineering&&haxe.Json.stringify(assembly.billOfMaterials().lines())==bom,
			"Explode evaluation leaves the engineering recipe and exported BOM unchanged");
		scene.recipeDiagnostics = ["REVIEW ONLY: nominal catalogue exterior; fixture accuracy, sealing, wall closure and casting process unqualified."];
		scene.presentation=machinekit.tooling.ToolingPresentation.socket(plan,scene,removal,retainedScrewVolume);
		var presentation=scene.presentation;if(presentation==null)throw "Missing tooling procedures";
		check(presentation.procedures[0].steps.map(step->step.id).join("|")==steps.map(step->step.id).join("|"),
			"Demoulding procedure exactly follows demolding-steps.json");
		var procedureCursor=materia.presentation.PresentationEvaluator.reset(presentation);procedureCursor.procedure="screws-left-fitted";
		var fingerprint=materia.presentation.PresentationGeometry.fingerprint(scene);
		var failed=materia.presentation.PresentationEvaluator.evaluate(scene.assemblyDefinition,poses,presentation,procedureCursor,fingerprint);
		check(failed.evidence=="Failed"&&failed.visibility.get(plan.assembly.members[1].panel.id)==Shown,
			"Retained screw heads show a visible Failed plate withdrawal");
		var thicker=HobberSocketFixtureSetup.document();thicker.recipe.panel.measuredThickness+=0.5;thicker.recipe.revision++;
		var changedPlan=InsertFixturePlanner.plate(thicker,HobberSocketFixtureSetup.options(thicker,[25,30,35]));
		var changedAssembly=SocketFixtureCad.assembly(changedPlan,true),changedScene=AssemblyPreview.scene(changedAssembly,"blind-socket-fixture");
		var changedFingerprint=materia.presentation.PresentationGeometry.fingerprint(changedScene);
		check(changedFingerprint!=fingerprint,"Changing plate thickness changes the geometry fingerprint");
		var stale=materia.presentation.PresentationEvaluator.evaluate(changedScene.assemblyDefinition,changedAssembly.solvedPoses(),presentation,procedureCursor,changedFingerprint);
		check(stale.evidence=="Stale","Changed plate thickness makes cached evidence Stale");
		var bytes = SceneArtifact.encode(scene);
		check(SceneArtifact.decode(bytes).parts.length == scene.parts.length,"Scene save/reopen");
		sys.io.File.saveBytes(out+"/socket-fixture-assembled.mtrg",bytes);

		sys.io.File.saveContent(out+"/review.json",haxe.Json.stringify({status:"Geometry review; not fabrication release",assertions:assertions,
			initialExactPairs:exactPairs,nominalMixMassKg:mix,releaseBeforeWithdrawalProved:true,validation:validation,
			fabrication:ToolingReadiness.evaluate(validation),blocking:removal.removal.blocking},null,"  "));
		Sys.println('Socket fixture: $assertions assertions, $exactPairs exact initial pairs; source-derived carrier and removal order exported.');
	}
}
