package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingRegion;
import materia.casting.CastingSection.CastingPrismSection;
import materia.casting.CastingSection.CastingSectionContour;
import materia.casting.CastingProfile.CastingSegment;
import materia.casting.CastingProfileGeometry as G;
import materia.casting.CastingProfileOffsets as Offset;
import materia.tooling.casting.CastingMoldPlan.CastingMoldCore;
import materia.tooling.casting.CoreSupport.CoreCutting;
import materia.tooling.ToolingRecipe;

/** Exact prism cores, with lining and explicit glue gaps between stock tiles. */
class MoldCoreLayers {
	static inline var EPS=1e-7;

	public static function supported(region:CastingRegion,id:String):Bool {
		var kernel=CastingMoldPlanner.geometry;
		return kernel!=null && kernel.prismaticSection(region,{x:0,y:0,z:1},EPS,id)!=null;
	}

	static function prism(profile:Array<CastingSegment>,pose:materia.assembly.AssemblyRecord.AssemblyFrame,depth:Float):CastingRegion
		return CastingRegion.Placed({solid:Prism(profile,depth),pose:pose});

	public static function build(region:CastingRegion,featureId:String,recipe:ToolingRecipe):Array<CastingMoldCore> {
		try return buildPrism(region,featureId,recipe) catch (error:Dynamic)
			throw "EPS core "+featureId+": "+Std.string(error);
	}

	static function buildPrism(region:CastingRegion,featureId:String,r:ToolingRecipe):Array<CastingMoldCore> {
		var kernel=CastingMoldPlanner.geometry;
		if (kernel==null) throw "Exact geometry kernel required";
		var section=kernel.prismaticSection(region,{x:0,y:0,z:1},EPS,featureId);
		if (section==null) throw "Unsupported non-prismatic core in fill direction";
		var outers=section.contours.filter(row -> row.depth%2==0),result:Array<CastingMoldCore> =[];
		var cover=r.linerThickness,glue=r.adhesiveLineThickness;
		var height=section.depth-cover-glue;
		if (height<=0) throw "Covering and adhesive consume foam height";
		for (index in 0...outers.length) {
			var outer=outers[index];
			var holes=section.contours.filter(row -> row.depth==outer.depth+1
				&& G.inside(outer.contour,row.contour[0].start.x,row.contour[0].start.y));
			var sourcePiece=CastingRegion.Difference(prism(outer.contour,section.pose,section.depth),
				[for (hole in holes) prism(hole.contour,section.pose,section.depth)]);
			var cavity=outers.length==1 ? region : CastingRegion.Intersection([region,sourcePiece]);
			var pose=F.compose(section.pose,F.translation(0,0,cover));
			var foam=CastingRegion.Difference(prism(Offset.inward(outer.contour,cover),pose,height),
				[for (hole in holes) prism(Offset.outward(hole.contour,cover),pose,height)]);
			if (MoldRegions.volume(foam,featureId)<=EPS) throw "Covering consumes foam section";
			if (MoldRegions.volume(CastingRegion.Difference(foam,[cavity]),featureId)>EPS)
				throw "Lined foam escapes its source cavity";
			var id=materia.tooling.ToolingIds.derive("core-",featureId,outers.length==1 ? "" : "-piece-"+index);
			result.push({id:id,featureId:featureId,cavity:cavity,foamEnvelope:foam,layers:tiles(foam,id,r),
				coveringThickness:cover,adhesiveThickness:glue,
				carrierId:materia.tooling.ToolingIds.derive("",id,"-carrier"),bridgeId:"pending",
				bondAreaMm2:0,bridgeBondAreaMm2:0});
		}
		return result;
	}

	/** A box trim of an aligned prism remains ruled along the same wire axis.
	 * The trim allocates manufacturing stock; it never redefines a design cut.
	 */
	static function tiles(foam:CastingRegion,id:String,r:ToolingRecipe):Array<CoreCutting> {
		var bounds=MoldCores.stockBounds(foam,id);
		var rectangle=materia.casting.CastingProfiles.rectangle(bounds.x,bounds.y,bounds.width,bounds.depth);
		var whole=CoreSupports.prismCutting(rectangle,bounds.height,F.translation(0,0,bounds.z),r.coreWireAxis);
		whole.solid=MoldRegions.moved(foam,F.inverse(whole.stockToMold));
		var sizes=[whole.block.width,whole.block.depth,whole.block.height];
		var available=[r.core.width,r.core.depth,r.core.height],counts:Array<Int> =[],steps:Array<Float> =[];
		var gap=r.adhesiveLineThickness,total=1;
		for (axis in 0...3) {
			var count=Std.int(Math.ceil((sizes[axis]+gap)/(available[axis]+gap)));
			if (count<1 || count>32 || total*count>32) throw "Too many EPS stock pieces: "+id;
			counts.push(count);total*=count;
			var step=(sizes[axis]-gap*(count-1))/count;
			if (step<=0 || step>available[axis]+EPS) throw "EPS glue gaps consume stock: "+id;
			steps.push(step);
		}
		if (total==1) return [whole];
		var result:Array<CoreCutting> =[];
		for (x in 0...counts[0]) for (y in 0...counts[1]) for (z in 0...counts[2]) {
			var cell={x:x*(steps[0]+gap),y:y*(steps[1]+gap),z:z*(steps[2]+gap),
				width:steps[0],depth:steps[1],height:steps[2]};
			var material=CastingRegion.Intersection([whole.solid,MoldRegions.box(cell)]);
			if (MoldRegions.volume(material,id)<=EPS) continue;
			var cutting=CoreSupports.cutting({x:0,y:0,z:0,width:cell.width,depth:cell.depth,height:cell.height},r.coreWireAxis);
			cutting.solid=MoldRegions.moved(material,F.translation(-cell.x,-cell.y,-cell.z));
			cutting.stockToMold=F.compose(whole.stockToMold,F.translation(cell.x,cell.y,cell.z));
			result.push(cutting);
		}
		if (result.length==0) throw "EPS stock contains no finished material: "+id;
		return result;
	}
}
