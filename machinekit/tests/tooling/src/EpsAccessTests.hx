import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingRegion;
import materia.tooling.casting.MoldRegions;
import materia.tooling.casting.MoldCoreLayers;
import materia.tooling.casting.MoldCores;
import machinekit.tooling.NativeCastingMoldKernel;

/** Access EPS retains its independent fixture notch and its registered flange. */
class EpsAccessTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {
		assertions++;
		if (!value) throw message;
	}
	static function box(x:Float,y:Float,z:Float,w:Float,d:Float,h:Float):CastingRegion
		return MoldRegions.box({x:x,y:y,z:z,width:w,depth:d,height:h});

	static function fixture(stepped:Bool):Void {
		var kernel=new NativeCastingMoldKernel();
		materia.tooling.casting.CastingMoldPlanner.geometry=kernel;
		var recipe=SinglePocketSetup.document().recipe;
		recipe.core.width=1000;recipe.core.depth=500;recipe.core.height=100;
		var region:CastingRegion,frame:materia.assembly.AssemblyRecord.AssemblyFrame;
		if (stepped) {
			region=Union([box(0,0,0,100,100,10),box(20,20,10,60,60,20),box(0,0,30,100,100,10)]);
			frame=F.identity();
		} else {
			region=Difference(Union([box(0,0,0,80,90,100),box(-30,-10,-10,30,110,120)]),
				[box(-40,20,0,100,50,18)]);
			frame=F.fromRotationMatrix(0,0,0,[0,0,1,1,0,0,0,1,0]);
		}
		var core=MoldCoreLayers.registeredFlange(region,"access","core-access",recipe,frame,"west");
		check(core.support.kind==materia.tooling.casting.CoreSupport.CoreSupportKind.RegisteredFlange,
			"The access core uses one EPS type with a registered flange support");
		check(core.layers.length>1 && core.layers.length<=32,"Wire stock is split at exact profile changes and stock limits");
		check(MoldRegions.volume(Difference(core.foamEnvelope,[core.cavity]),core.id)<1e-6,
			"Lined foam stays inside the full registered access envelope");
		check(core.support.clampBearingAreaMm2>0,"Clamp bearing has a finite geometric limit, not a capacity claim");
		var pieces:Array<CastingRegion>=[];
		for (layer in core.layers) {
			check(kernel.prismaticSection(layer.solid,layer.wireDirection,1e-6,core.id)!=null,
				"Every stock piece is proved ruled in its declared wire direction");
			var piece=MoldCores.layerRegion(layer);
			check(MoldRegions.volume(Difference(piece,[core.foamEnvelope]),core.id)<1e-6,
				"Every wire-cut piece fits the globally lined envelope");
			for (previous in pieces) check(MoldRegions.volume(Intersection([piece,previous]),core.id)<1e-6,
				"EPS stock pieces have no overlapping interiors");
			pieces.push(piece);
		}
		check(MoldRegions.volume(Difference(core.foamEnvelope,[Union(pieces)]),core.id)>0,
			"Declared adhesive gaps remain distinct from the global covering");
		if (stepped) {
			var error="";
			try machinekit.tooling.FormerManufacturing.machining("undercut",region,F.identity(),recipe.panel,0,recipe)
			catch (caught:Dynamic) error=Std.string(caught);
			check(error.indexOf("Upper material blocks the cutter shaft")>=0,
				"Sharing the geometry bands does not remove the router's shaft-access rejection");
		}
	}
	static function main():Void {
		fixture(false);fixture(true);
		Sys.println('EpsAccessTests: $assertions assertions passed');
	}
}
