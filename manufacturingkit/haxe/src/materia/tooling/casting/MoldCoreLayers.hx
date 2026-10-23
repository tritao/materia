package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingRegion;
import materia.casting.CastingProfile.CastingSegment;
import materia.casting.CastingProfileGeometry as G;
import materia.casting.CastingProfileOffsets as Offset;
import materia.tooling.casting.CastingMoldPlan.CastingMoldCore;
import materia.tooling.casting.CoreSupport.CoreCutting;
import materia.tooling.ToolingRecipe;

/** Exact ruled cores, with lining and explicit glue gaps between stock pieces. */
class MoldCoreLayers {
	static inline var EPS=1e-7;

	public static function box(cavity:materia.casting.Box3,featureId:String,r:ToolingRecipe,
			kind:CoreSupport.CoreSupportKind,singleBlock:Bool=false):CastingMoldCore {
		var bridge=kind==CoreSupport.CoreSupportKind.Bridge,cover=r.linerThickness,g=r.adhesiveLineThickness;
		var foam:materia.casting.Box3={x:cavity.x+cover,y:cavity.y+cover,z:cavity.z+(bridge ? cover : 0),
			width:cavity.width-2*cover,depth:cavity.depth-2*cover,height:cavity.height-cover-(bridge ? g : 0)};
		if (foam.width<=0 || foam.depth<=0 || foam.height<=0 || foam.width>r.core.width || foam.depth>r.core.depth
			|| (singleBlock && foam.height>r.core.height)) throw "EPS block exceeds stock";
		var count=Std.int(Math.ceil((foam.height+g)/(r.core.height+g)));
		if (count<1 || count>(singleBlock ? 1 : 32)) throw "Too many EPS layers";
		var layerHeight=(foam.height-g*(count-1))/count;
		if (layerHeight<=0) throw "EPS adhesive consumes layer height";
		var layers:Array<CoreCutting>=[];
		for (index in 0...count) layers.push(CoreSupports.cutting({x:foam.x,y:foam.y,z:foam.z+index*(layerHeight+g),
			width:foam.width,depth:foam.depth,height:layerHeight},r.coreWireAxis));
		return {id:materia.tooling.ToolingIds.derive("core-",featureId),featureId:featureId,
			cavity:MoldRegions.box(cavity),foamEnvelope:MoldRegions.box(foam),layers:layers,coveringThickness:cover,
			support:CoreSupports.specification(kind,r)};
	}

	public static function supported(region:CastingRegion,id:String):Bool {
		var kernel=CastingMoldPlanner.geometry;
		if (kernel==null) return false;
		return MoldCoreDrafts.hasDraft(region,id) ? MoldCoreDrafts.section(region,id,EPS)!=null
			: kernel.prismaticSection(region,{x:0,y:0,z:1},EPS,id)!=null;
	}

	static function prism(profile:Array<CastingSegment>,pose:materia.assembly.AssemblyRecord.AssemblyFrame,depth:Float):CastingRegion
		return CastingRegion.Placed({solid:Prism(profile,depth),pose:pose});

	public static function build(region:CastingRegion,featureId:String,recipe:ToolingRecipe):Array<CastingMoldCore> {
		try return buildRuled(region,featureId,recipe) catch (error:Dynamic)
			throw "EPS core "+featureId+": "+Std.string(error);
	}

	static function buildRuled(region:CastingRegion,featureId:String,r:ToolingRecipe):Array<CastingMoldCore> {
		var kernel=CastingMoldPlanner.geometry;
		if (kernel==null) throw "Exact geometry kernel required";
		var section=MoldCoreDrafts.hasDraft(region,featureId) ? null : kernel.prismaticSection(region,{x:0,y:0,z:1},EPS,featureId);
		if (section==null) {
			var draft=MoldCoreDrafts.section(region,featureId,EPS);
			if (draft==null) throw "Unsupported non-ruled core in fill direction";
			var foam=MoldCoreDrafts.foam(draft,r.linerThickness,r.adhesiveLineThickness);
			return [core(region,foam,featureId,materia.tooling.ToolingIds.derive("core-",featureId),r,false)];
		}
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
			var id=materia.tooling.ToolingIds.derive("core-",featureId,outers.length==1 ? "" : "-piece-"+index);
			result.push(core(cavity,foam,featureId,id,r,true));
		}
		return result;
	}

	/** An EPS access former retains the already-proved neck and flange geometry.
	 * Erosion uses the profile metric times the withdrawal-axis interval. Across
	 * a step, intersect every neighbouring eroded section within covering reach;
	 * internal stock boundaries therefore receive glue, never spurious lining.
	 */
	public static function registeredFlange(region:CastingRegion,featureId:String,id:String,r:ToolingRecipe,
			frame:materia.assembly.AssemblyRecord.AssemblyFrame,panelId:String):CastingMoldCore {
		var kernel=CastingMoldPlanner.geometry;
		if (kernel==null) throw "Exact geometry kernel required for EPS access former: "+featureId;
		var bounds=kernel.bounds(MoldRegions.moved(region,F.inverse(frame)),featureId);
		frame=F.compose(frame,F.translation(bounds.x,bounds.y,bounds.z));
		var work=MoldRegions.moved(region,F.inverse(frame));
		var bands=kernel.prismaticBands(work,bounds.width,bounds.depth,bounds.height,featureId);
		var cover=r.linerThickness,gap=r.adhesiveLineThickness;
		var low=cover,high=bounds.height-cover;
		if (high<=low) throw "Covering consumes EPS access former: "+featureId;
		var levels=[low,high];
		for (band in bands) for (value in [band.bottomMm-cover,band.bottomMm+cover,band.topMm-cover,band.topMm+cover])
			if (value>low+EPS && value<high-EPS) levels.push(value);
		levels.sort(Reflect.compare);
		var unique:Array<Float>=[];
		for (value in levels) if (unique.length==0 || value-unique[unique.length-1]>EPS) unique.push(value);
		levels=unique;
		var cells:Array<{bottom:Float,top:Float,region:CastingRegion}>=[];
		for (index in 0...levels.length-1) {
			var bottom=levels[index],top=levels[index+1],middle=(bottom+top)/2;
			var active=bands.filter(band -> band.bottomMm<middle+cover+EPS && band.topMm>middle-cover-EPS);
			var sections:Array<CastingRegion>=[];
			for (band in active) {
				var outers:Array<CastingRegion>=[];
				for (outer in band.contours) if (outer.depth%2==0) {
					var holes=band.contours.filter(hole -> hole.depth==outer.depth+1
						&& G.inside(outer.contour,hole.contour[0].start.x,hole.contour[0].start.y));
					var pose=F.translation(0,0,bottom);
					outers.push(CastingRegion.Difference(prism(Offset.inward(outer.contour,cover),pose,top-bottom),
						[for (hole in holes) prism(Offset.outward(hole.contour,cover),pose,top-bottom)]));
				}
				sections.push(CastingRegion.Union(outers));
			}
			if (sections.length==0) throw "Missing exact EPS erosion section: "+featureId;
			var cell=CastingRegion.Intersection(sections);
			if (MoldRegions.volume(cell,featureId)>EPS) cells.push({bottom:bottom,top:top,region:cell});
		}
		if (cells.length==0) throw "Covering consumes EPS access former: "+featureId;
		var merged:Array<{bottom:Float,top:Float,region:CastingRegion}>=[];
		for (cell in cells) {
			var section=kernel.prismaticSection(cell.region,{x:0,y:0,z:1},EPS,featureId);
			if (section==null) throw "EPS erosion interval is not prismatic: "+featureId;
			var footprint=MoldRegions.section(section.contours,F.identity(),1);
			if (merged.length>0 && Math.abs(merged[merged.length-1].top-cell.bottom)<EPS) {
				var previous=merged[merged.length-1];
				var other=kernel.prismaticSection(previous.region,{x:0,y:0,z:1},EPS,featureId);
				if (other==null) throw "EPS stock interval is not prismatic: "+featureId;
				var old=MoldRegions.section(other.contours,F.identity(),1);
				var difference=MoldRegions.volume(CastingRegion.Difference(footprint,[old]),featureId)
					+MoldRegions.volume(CastingRegion.Difference(old,[footprint]),featureId);
				if (difference<=EPS) {
					previous.top=cell.top;
					previous.region=MoldRegions.section(other.contours,F.translation(0,0,previous.bottom),previous.top-previous.bottom);
					continue;
				}
			}
			merged.push(cell);
		}
		cells=merged;
		var foam=CastingRegion.Union([for (cell in cells) cell.region]),layers:Array<CoreCutting>=[];
		for (index in 0...cells.length) {
			var cell=cells[index],bottom=cell.bottom,top=cell.top;
			if (index>0 && Math.abs(cells[index-1].top-bottom)<EPS) bottom+=gap/2;
			if (index+1<cells.length && Math.abs(cells[index+1].bottom-top)<EPS) top-=gap/2;
			if (top<=bottom) throw "Adhesive consumes EPS stock band "+cell.bottom+".."+cell.top+" mm: "+featureId;
			var trimmed=CastingRegion.Intersection([cell.region,MoldRegions.box({x:0,y:0,z:bottom,
				width:bounds.width,depth:bounds.depth,height:top-bottom})]);
			for (cutting in tiles(trimmed,id,r,true)) {
				cutting.stockToMold=F.compose(frame,cutting.stockToMold);layers.push(cutting);
			}
		}
		if (layers.length>32) throw "Too many EPS stock pieces: "+featureId;
		if (MoldRegions.volume(CastingRegion.Difference(foam,[work]),featureId)>EPS)
			throw "EPS lining escapes its access former: "+featureId;
		var support=CoreSupports.specification(CoreSupport.CoreSupportKind.RegisteredFlange,r);
		var root=kernel.prismaticSection(cells[0].region,{x:0,y:0,z:1},EPS,featureId);
		if (root==null) throw "EPS flange has no certified bearing face: "+featureId;
		support.clampBearingAreaMm2=MoldCores.area(root.contours);
		support.panelIds=[panelId];support.padClosure=materia.tooling.panel.PanelJoints.PanelClosure.ExternalClamps;
		return {id:id,featureId:featureId,cavity:region,foamEnvelope:MoldRegions.moved(foam,frame),layers:layers,
			coveringThickness:cover,support:support};
	}

	static function core(cavity:CastingRegion,foam:CastingRegion,featureId:String,id:String,r:ToolingRecipe,
			allowProfileSplit:Bool):CastingMoldCore {
		if (MoldRegions.volume(foam,featureId)<=EPS) throw "Covering consumes foam section";
		if (MoldRegions.volume(CastingRegion.Difference(foam,[cavity]),featureId)>EPS)
			throw "Lined foam escapes its source cavity";
		return {id:id,featureId:featureId,cavity:cavity,foamEnvelope:foam,layers:tiles(foam,id,r,allowProfileSplit),
			coveringThickness:r.linerThickness,support:CoreSupports.specification(CoreSupport.CoreSupportKind.Bridge,r)};
	}

	/** Prisms allow trims on every stock plane. Drafted foam retains complete
	 * cross-sections and can split only between wire-normal planes. Stock
	 * allocation never redefines a source design cut.
	 */
	static function tiles(foam:CastingRegion,id:String,r:ToolingRecipe,allowProfileSplit:Bool):Array<CoreCutting> {
		var bounds=MoldCores.stockBounds(foam,id);
		var rectangle=materia.casting.CastingProfiles.rectangle(bounds.x,bounds.y,bounds.width,bounds.depth);
		var whole=CoreSupports.prismCutting(rectangle,bounds.height,F.translation(0,0,bounds.z),r.coreWireAxis);
		whole.solid=MoldRegions.moved(foam,F.inverse(whole.stockToMold));
		var sizes=[whole.block.width,whole.block.depth,whole.block.height];
		var available=[r.core.width,r.core.depth,r.core.height],counts:Array<Int> =[],steps:Array<Float> =[];
		var gap=r.adhesiveLineThickness,total=1;
		var wireAxis=switch r.coreWireAxis {case X:0;case Y:1;case Z:2;default:throw "Invalid core wire axis";};
		for (axis in 0...3) {
			var count=Std.int(Math.ceil((sizes[axis]+gap)/(available[axis]+gap)));
			if (!allowProfileSplit && axis!=wireAxis && count>1)
				throw "Unsupported drafted EPS cross-profile tiling for the declared wire axis: "+id;
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
