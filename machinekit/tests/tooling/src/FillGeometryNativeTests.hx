import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Solids;
import machinekit.tooling.CastingFillAnalysis;
import machinekit.tooling.CastingFillGeometry;
import machinekit.tooling.ToolingBlock;
import materia.tooling.process.FillAnalysis;
import materia.tooling.casting.MoldPlan.MoldBox;

class FillGeometryNativeTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {assertions++;if (!ok) throw message;}
	static function near(a:Float,b:Float,message:String):Void check(Math.abs(a-b)<1e-5,message+": "+a+" vs "+b);
	static function settings():FillSourceSettings return {
		sourceFingerprint:"source",processFingerprint:"process",cores:[],densityKgM3:2400,gravityMps2:9.80665,
		pourRateMm3PerSecond:100,maximumAirEscapeWidthMm:null,extraLevels:[1,3,5,9],fillObstructions:[],
		panels:[{id:"wall",bottomMm:2}]
	};
	static function box(x:Float,z:Float,width:Float,height:Float):MoldBox
		return {x:x,y:0,z:z,width:width,depth:2,height:height};
	static function source(boxes:Array<MoldBox>):Part
		return Solids.building([],owned -> {
			for (box in boxes) owned.push(ToolingBlock.placedBox(box));
			return Part.fuseAll(owned);
		});
	public static function run():Void {
		var distantA=ToolingBlock.placedBox({x:10000,y:10000,z:10000,width:10,depth:10,height:10});
		var distantB=ToolingBlock.placedBox({x:20000,y:20000,z:20000,width:10,depth:10,height:10});
		check(CastingFillGeometry.componentKey(distantA)!=CastingFillGeometry.componentKey(distantB),
			"large source coordinates cannot alias through integer-rounded geometry keys");
		distantA.close();distantB.close();
		var cells=[box(0,0,2,10),box(2,0,2,2),box(4,0,4,6)],part=source(cells),s=settings();
		var native=CastingFillAnalysis.analyze("side-neck",part,10,s);
		var exact=FillAnalysis.analyze("side-neck","source","process",cells,[],[],10,2400,9.80665,100,null,s.extraLevels,[],s.panels);
		near(native.geometricResinVolumeMm3,96,"native region has the independent CAD volume");
		near(native.totalMixVolumeMm3,64,"side-sealed air is excluded from liquid quantity");
		check(native.trappedPockets.length==1,"native event solids detect the side-sealed pocket");
		near(native.trappedPockets[0].sealedAtMm,2,"side-neck closes at its exact event plane");
		near(native.trappedPockets[0].volumeMm3,32,"native limiting trapped volume agrees with the independent box path");
		for (level in exact.levels) {
			var matches=native.levels.filter(row -> Math.abs(row.heightMm-level.heightMm)<1e-6);
			check(matches.length==1,"native output retains the requested level");
			near(matches[0].mixVolumeMm3,level.mixVolumeMm3,"native and box liquid curves agree");
			near(matches[0].panelHeads[0].headPa,level.panelHeads[0].headPa,"native and box per-panel heads agree");
		}
		var pocket=native.trappedPockets[0],geometry=new CastingFillGeometry("side-neck",part);
		switch pocket.geometry {
			case CadAirAbove(height,key):
				var regions=geometry.airComponents(height),matches=regions.filter(region -> CastingFillGeometry.componentKey(region)==key);
				check(matches.length==1,"native pocket expression resolves uniquely in the bound source");
				near(matches[0].volume(),pocket.volumeMm3,"resolved expression carries the true pocket geometry");
				for (region in regions) region.close();
			case Rectilinear(_,_):throw "Native pocket must not pretend to be an array of boxes";
		}
		var reordered=cells.copy();reordered.reverse();
		var permutedSource=source(reordered),permuted=CastingFillAnalysis.analyze("side-neck",permutedSource,10,settings());
		check(materia.tooling.ToolingCodec.canonical(permuted.trappedPockets)==materia.tooling.ToolingCodec.canonical(native.trappedPockets),
			"source Boolean member order cannot change pocket identity or geometry references");
		permutedSource.close();
		var narrow=settings();narrow.maximumAirEscapeWidthMm=1;
		var wide=settings();wide.maximumAirEscapeWidthMm=3;
		check(CastingFillAnalysis.analyze("side-neck",part,10,narrow).validation.filter(row ->
			row.rule=="process.fill.air-escape-width" && row.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail).length==1,
			"native ceiling width screen rejects a wider roof");
		check(CastingFillAnalysis.analyze("side-neck",part,10,wide).validation.filter(row ->
			row.rule=="process.fill.air-escape-width" && row.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass).length==1,
			"native ceiling width screen accepts the declared conservative bound");
		var dense=settings();dense.extraLevels=[for (index in 0...21) index/2.0];
		var denseReport=CastingFillAnalysis.analyze("side-neck",part,10,dense);
		check(materia.tooling.ToolingCodec.canonical(denseReport.trappedPockets)==materia.tooling.ToolingCodec.canonical(native.trappedPockets),
			"additional output levels cannot change exact pocket detection or geometry identity");
		for (level in denseReport.levels) {
			var h=level.heightMm,expected=4*Math.min(h,10)+4*Math.min(h,2)+8*Math.min(h,6)-8*Math.max(0,Math.min(h,6)-2);
			near(level.mixVolumeMm3,expected,"dense native liquid curve excludes only the sealed air region");
		}
		geometry.close();part.close();
		var local=Part.prism([new Vector(0,0),new Vector(40,0),new Vector(40,20),new Vector(10,40),new Vector(0,40)],0,30);
		var pose=materia.assembly.AssemblyFrames.fromRotationMatrix(0,0,0,[1,0,0,0,0,-1,0,1,0]);
		var slope=machinekit.assembly.PosedParts.moved(local,pose);local.close();
		s=settings();s.extraLevels=[0,10,20,30,40];
		var sloped=CastingFillAnalysis.analyze("sloped-ceiling",slope,40,s);
		near(sloped.geometricResinVolumeMm3,39000,"sloped fill agrees with actual CAD volume");
		near(sloped.openFillAreaMm2,300,"sloped fill has the true top footprint");
		check(sloped.trappedPockets.length==0,"sloped ceiling stays connected to its upper opening");
		for (level in sloped.levels) {
			var h=level.heightMm,expected=h<=20 ? h*1200 : 24000+30*(40*(h-20)-0.75*(h-20)*(h-20));
			near(level.mixVolumeMm3,expected,"full sloped fill report agrees with analytic level integral");
		}
		var cover=ToolingBlock.placedBox({x:0,y:-30,z:45,width:5,depth:30,height:2});
		var partial=CastingFillAnalysis.analyze("sloped-ceiling",slope,40,s,[],[{id:"cover",part:cover}]);
		near(partial.openFillAreaMm2,150,"tooling above the fill plane blocks its exact projection");
		var fullCover=ToolingBlock.placedBox({x:0,y:-30,z:45,width:10,depth:30,height:2});
		var message="";
		try CastingFillAnalysis.analyze("sloped-ceiling",slope,40,s,[],[{id:"cover",part:fullCover}])
		catch (error:Dynamic) message=Std.string(error);
		check(message.indexOf("Fill face has zero open area")>=0,"overhead tooling that closes the last opening is rejected");
		fullCover.close();
		cover.close();slope.close();
		Sys.println('FillGeometryNativeTests: $assertions assertions passed');
	}
}
