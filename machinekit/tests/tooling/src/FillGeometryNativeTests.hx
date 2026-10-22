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
	/** The washer is against the anchor's head, not clamped against the insert.
	 * Its clearance bore is open below; the real head closes its upper end. */
	static function anchorWasherPocket():Void {
		var scope=new cadkit.modeling.Scope();
		try {
			var screw=machinekit.standard.SocketHeadCapScrew.metric("M3",12,machinekit.standard.PropertyClass.Steel88);
			var washer=machinekit.standard.FlatWasher.metric("M3");
			var boltLocal=scope.own(screw.geometry()),washerLocal=scope.own(washer.geometry());
			var bolt=scope.own(boltLocal.translated(new Vector(0,0,8)));
			var washerPart=scope.own(machinekit.assembly.PosedParts.moved(washerLocal,
				{x:0,y:0,z:8,qx:1,qy:0,qz:0,qw:0}));
			bolt.shape.setThreadedFaces([]);
			var body=scope.own(Part.box(20,20,14));
			var resin=scope.own(body.subtractAll([bolt,washerPart]));
			var nearMiss=scope.own(Part.cylinderSpan(1.49,7.5,8));
			var boundaries:Array<machinekit.tooling.CastingFillAnalysis.NativeFillPart> = [
				{id:"anchor-A-bolt",sourceIds:["seat-A","back-anchor"],part:bolt},
				{id:"anchor-A-washer",sourceIds:["seat-A","back-anchor"],part:washerPart},
				{id:"near-miss",part:nearMiss}
			];
			var s=settings(),report=CastingFillAnalysis.analyze("washer-fixture",resin,14,s,boundaries);
			check(report.trappedPockets.length==1,"a resin-accessible under-head washer clearance remains a real trap");
			var pocket=report.trappedPockets[0];
			near(pocket.volumeMm3,Math.PI*(1.6*1.6-1.5*1.5)*0.5,"trap is the exact M3 washer/shank clearance annulus");
			near(pocket.sealedAtMm,7.5,"washer mouth seals at its actual lower face");
			near(pocket.proposedVentPoint.z,8,"screw head is the actual pocket ceiling");
			check(pocket.sourceIds.join(",")=="anchor-A-bolt,anchor-A-washer,back-anchor,seat-A",
				"touching boundary faces name both hardware members, the source anchor and insert; a near miss contributes nothing");
			var geometry=new CastingFillGeometry("washer-fixture",resin);
			try {
				var before=geometry.airComponents(7.49);
				check(before.length==1,"washer bore actually connects to the resin domain before the mouth seals");
				for (part in before) part.close();
				switch pocket.geometry {
					case CadAirAbove(height,key):
						var regions=geometry.airComponents(height);
						var outer=scope.own(Part.cylinderSpan(1.6,7.5,8)),inner=scope.own(Part.cylinderSpan(1.5,7.5,8));
						var annulus=scope.own(outer.subtract(inner));
						var matches=0;
						for (region in regions) {
							if (CastingFillGeometry.componentKey(region)==key) {
								matches++;
								var missing=scope.own(region.subtract(annulus)),extra=scope.own(annulus.subtract(region));
								near(missing.volume()+extra.volume(),0,"componentKey resolves to exactly the washer annulus by symmetric difference");
							}
							region.close();
						}
						check(matches==1,"recorded hardware pocket has one exact source region");
					case Rectilinear(_,_):throw "Expected native washer pocket";
				}
				geometry.close();
			} catch (error:Dynamic) {geometry.close();throw error;}
			boundaries.reverse();
			var reordered=CastingFillAnalysis.analyze("washer-fixture",resin,14,s,boundaries);
			check(materia.tooling.ToolingCodec.canonical(reordered.trappedPockets)==materia.tooling.ToolingCodec.canonical(report.trappedPockets),
				"hardware boundary order cannot change pocket IDs or attribution");
			var flipped=scope.own(machinekit.assembly.PosedParts.moved(resin,
				{x:0,y:0,z:14,qx:1,qy:0,qz:0,qw:0}));
			var inverted=CastingFillAnalysis.analyze("washer-fixture",flipped,14,s);
			check(inverted.trappedPockets.length==1,"inverting the real screw vents the washer but exposes its downward-open hex socket");
			near(inverted.trappedPockets[0].volumeMm3,Math.sqrt(3)/2*screw.spec.socketSize*screw.spec.socketSize*screw.spec.socketDepth,
				"only the actual hex socket traps air after inversion; the annulus is genuinely vented");
			scope.close();
		} catch (error:Dynamic) {scope.close();throw error;}
	}
	public static function run():Void {
		anchorWasherPocket();
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
