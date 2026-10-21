import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import machinekit.structural.InsertHoles;
import machinekit.design.DesignInterfaces;
import machinekit.tooling.ToolingInsertCad;
import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertManufacturing;
import materia.casting.InsertManufacturing.InsertManufacturingCodec;
import materia.casting.InsertStages.InsertStage;
import materia.casting.InsertSpec.InsertBodyCodec;

class InsertStageTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {assertions++;if (!ok) throw message;}
	static function near(a:Float,b:Float,message:String):Void check(Math.abs(a-b)<1e-6,message);
	static function rejects(fn:Void->Void,expected:String):Void {
		var error="";try fn() catch (e:Dynamic) error=Std.string(e);
		check(error.indexOf(expected)>=0,'Expected "$expected", got "$error"');
	}
	public static function main():Void run();

	public static function run():Void {
		var pattern=InsertHoles.tappedBlind(InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(6).mating()),20);
		var holes=InsertHoles.records(pattern),first=holes[0];
		first.id="pre";first.frame.x=-12;
		var second=materia.casting.InsertHole.InsertHoleCodec.snapshot([first])[0];
		second.id="post";second.frame.x=12;holes.push(second);
		var material=materia.casting.InsertMaterial.InsertMaterials.specification(materia.casting.InsertMaterial.C45Bright);
		var manufacturing=InsertManufacturingCodec.draft(material);
		manufacturing.holeRoutes=[{holeId:"pre",route:InsertHoleRoute.BeforeCasting},
			{holeId:"post",route:InsertHoleRoute.TransferAfterCasting}];
		var values=CastInsert.boxValues(50,40,20).setToken("role",materia.casting.InsertSpec.InsertRole.GenericMount)
			.setToken("holes",InsertHoles.fromNeutral(holes)).setToken("origin",haxe.Json.stringify(
				materia.casting.InsertOrigin.InsertOriginCodec.record(materia.casting.InsertOrigin.Fabricated(manufacturing))));
		var source=new CastInsert(values),spec=source.insertSpec();
		var input:materia.tooling.casting.CastingToolingInput.CastingToolingInsert={id:"staged-insert",body:spec.body,holes:spec.holes,
			origin:spec.origin,anchorage:spec.anchorage,interfaces:spec.interfaces,material:source.insertMaterial(),pose:F.identity(),
			role:materia.casting.InsertSpec.InsertRole.GenericMount,label:"stage test",positionTolerance:0.01,
			finish:materia.casting.InsertSpec.InsertFinish.AsSupplied};
		var raw=ToolingInsertCad.component(input),service=ToolingInsertCad.serviceComponent(input);
		check(raw.buildStage()==InsertStage.BeforeCasting && service.buildStage()==InsertStage.Service,"geometry stage is explicit");
		var rawFacet=DesignInterfaces.of(raw),serviceFacet=DesignInterfaces.of(service);
		if (rawFacet==null) throw "Missing pre-cast physical interfaces";
		if (serviceFacet==null) throw "Missing service interfaces";
		check(rawFacet.holes.length==1 && rawFacet.holes[0].id=="pre",
			"pre-cast physical interface facet contains only its actual hole");
		check(serviceFacet.holes.length==2 && raw.insertSpec().holes.length==2,
			"full source design survives the pre-cast view without phantom physical interfaces");
		var drawing=machinekit.tooling.InsertDrawing.review(input).drawing,entries=0,serviceNote=false;
		for (entity in drawing.entities()) switch entity {
			case Curve(layer,segment):
				if (cadkit.drawing.Drawing2D.layerName(layer)=="HOLE_ENTRY") {
					entries++;near(segment.arcCenter().x,-12,"raw drawing entry follows the pre-cast hole position");
				}
			case Label(_,_,_,text):if (text.indexOf("post datum offsets XYZ=37,20,20 ")==0) serviceNote=true;
		}
		check(entries==2 && serviceNote,"raw drawings omit the future bore while retaining its service schedule");
		var assembly=new machinekit.assembly.MachineAssembly();assembly.addComponent("blank",raw);
		var restored=machinekit.assembly.MachineAssembly.decode(assembly.encode());
		var restoredPart:CastInsert=cast restored.components()[0].component;
		var restoredFacet=DesignInterfaces.of(restoredPart);
		if (restoredFacet==null) throw "Missing restored pre-cast interfaces";
		check(restoredPart.buildStage()==InsertStage.BeforeCasting && restoredFacet.holes.length==1,
			"portable assembly reconstruction retains the physical manufacturing stage");
		var stockValues=values.copy().setToken("body",InsertBodyCodec.encode(Primitive(Box(50,40,21))))
			.setToken("interfaces",haxe.Json.stringify(materia.casting.InterfaceRequirement.InterfaceRequirements.records([{
				id:"finished-top",geometry:materia.casting.InterfaceRequirement.PrecisionGeometry.Plane(F.translation(0,0,20)),
				route:materia.casting.InterfaceRequirement.InterfaceRoute.Ground,allowanceMm:1,finishingProcessErrorMm:null,
				permittedFinishes:[materia.casting.InterfaceRequirement.InterfaceFinish.Ground],positionToleranceMm:0.01,
				datumReference:"primary",relationshipIds:[]}])));
		var finished=new CastInsert(stockValues);
		var unfinished=new CastInsert(stockValues.copy().setToken("buildStage",InsertStage.BeforeCasting));
		near(finished.connector("top").frame.z,20,"service top connector lies on the finished steel");
		near(unfinished.connector("top").frame.z,21,"pre-cast top connector lies on the raw stock");
		var rawStockHoles=InsertHoles.copy(DesignInterfaces.of(unfinished));
		near(rawStockHoles[0].frame.z,21,"pre-cast drilling starts on the raw stock surface");
		near(rawStockHoles[0].frame.z-rawStockHoles[0].depth,first.frame.z-first.depth,
			"extending drilling through stock preserves the finished blind bottom");
		near(rawStockHoles[0].frame.z-(cast rawStockHoles[0].threadDepth:Float),first.frame.z-(cast first.threadDepth:Float),
			"extending tapping through stock preserves usable service thread");
		var stockInput:materia.tooling.casting.CastingToolingInput.CastingToolingInsert={id:input.id,body:finished.insertSpec().body,
			holes:finished.insertSpec().holes,origin:finished.insertSpec().origin,anchorage:finished.insertSpec().anchorage,interfaces:finished.insertSpec().interfaces,
			material:input.material,pose:input.pose,role:input.role,label:input.label,positionTolerance:input.positionTolerance,finish:input.finish};
		var rawSchedule=false;
		for (entity in machinekit.tooling.InsertDrawing.review(stockInput).drawing.entities()) switch entity {
			case Label(_,_,_,text):if (text.indexOf("pre PRE-CAST machining entry XYZ=-12,0,21 / drill depth="+rawStockHoles[0].depth)==0) rawSchedule=true;
			default:
		}
		check(rawSchedule,"raw manufacturing drawing gives the extended machining entry and depth, not only service dimensions");
		var owned:Array<Part> = [];
		try {
			var rawStock=unfinished.geometry();owned.push(rawStock);
			var finishedSteel=finished.geometry();owned.push(finishedSteel);
			var baseline=source.geometry();owned.push(baseline);
			var finishedExtra=finishedSteel.subtractAll([baseline]);owned.push(finishedExtra);
			var baselineExtra=baseline.subtractAll([finishedSteel]);owned.push(baselineExtra);
			near(finishedExtra.volume()+baselineExtra.volume(),0,
				"finished steel and finished blind holes match the exact no-stock source by symmetric difference");
			check(rawStock.volume()>finishedSteel.volume(),"raw physical steel retains stock until finishing");
			var a=raw.geometry();owned.push(a);
			var b=service.geometry();owned.push(b);
			var original=source.geometry();owned.push(original);
			check(a.valid() && b.valid() && a.volume()>b.volume(),"post-cast drilling removes material only in service geometry");
			var difference=original.subtractAll([b]);owned.push(difference);
			var reverse=b.subtractAll([original]);owned.push(reverse);
			near(difference.volume()+reverse.volume(),0,"service geometry equals the complete source CAD by symmetric difference");
			var saved=restoredPart.geometry();owned.push(saved);
			var savedDifference=saved.subtractAll([a]);owned.push(savedDifference);
			var savedReverse=a.subtractAll([saved]);owned.push(savedReverse);
			near(savedDifference.volume()+savedReverse.volume(),0,"restored pre-cast CAD equals original by symmetric difference");
			var cylinder=Part.cylinder(1,1);owned.push(cylinder);
			var postWitness=cylinder.translated(new Vector(12,0,18));owned.push(postWitness);
			var preWitness=cylinder.translated(new Vector(-12,0,18));owned.push(preWitness);
			var postRaw=a.intersect(postWitness);owned.push(postRaw);
			var postService=b.intersect(postWitness);owned.push(postService);
			var preRaw=a.intersect(preWitness);owned.push(preRaw);
			near(postRaw.volume(),postWitness.volume(),"future bore is solid steel during casting at its actual location");
			near(postService.volume(),0,"future bore exists after drilling in service");
			near(preRaw.volume(),0,"declared pre-cast bore already exists at its own location");
		} catch (error:Dynamic) {for (part in owned) part.close();throw error;}
		for (part in owned) part.close();
		var foundation=new machinekit.structural.MineralFoundation(new ComponentValues().setNumber("width",100)
			.setNumber("depth",80).setNumber("height",100),[{id:"seat",part:raw,pose:F.translation(0,0,80),
				openings:[materia.casting.CastingPatterns.face("body.z-max")]}]);
		rejects(() -> machinekit.tooling.FoundationToolingAdapter.snapshot(foundation,"wrong-stage",1),
			"Foundation design requires a service-stage insert");
		Sys.println('InsertStageTests: $assertions assertions passed');
	}
}
