package machinekit.structural;

import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertSpec;
import materia.casting.InsertAnchorage;
import machinekit.standard.HexFlangeBolt;
import cadkit.modeling.Part;

enum InsertAnchorContext {
	FootprintOnly;
	Casting(body:Part,pose:materia.assembly.AssemblyRecord.AssemblyFrame);
}

/** One source design rule, shared by every insert in a machine recipe. All
 * lengths are derived from the chosen standard thread or the back footprint. */
typedef InsertAnchorPatternRule = {
	var id:String;
	var demandFactor:Float;
	var maximumCount:Int;
	var liftingLoadPaths:Int;
	var gravityMps2:Float;
	var minimumCount:Int;
	var maximumPitchInShortWidths:Float;
	var embedmentDiameters:Float;
	var coverDiameters:Float;
	var spacingDiameters:Float;
	var screwLengthsMm:Array<Float>;
}

class InsertAnchorPattern {
	/** Square land for a central clearance bore and corner anchors. Both bore
	 * walls survive: the corner bore must lie wholly outside the central wall. */
	public static function minimumClearancePadSide(diameter:Float,thickness:Float,
		material:materia.casting.InsertMaterial):Float {
		var central=machinekit.design.ThreadedHolePolicy.clearance(diameter);
		for (size in availableSizes()) {
			var hole:materia.casting.InsertHole;
			try hole=holeForStock(size,thickness,material) catch (_:Dynamic) continue;
			var nominal=HexFlangeBolt.catalog().get(size);
			var edge=Math.max(hole.limits.minWall+nominal.diameter/2,nominal.flangeDiameter/2);
			var separation=diameter/2+nominal.diameter/2+Math.max(central.minWall,hole.limits.minWall);
			return Math.max(diameter+2*central.minWall,2*(edge+separation/Math.sqrt(2)));
		}
		throw "No standard blind anchor thread fits the clearance-pad thickness";
	}

	/** Absolute geometric floor around a central opening, using the smallest
	 * standard flange bolt. The demand-admitted pattern can require more land. */
	public static function minimumBandDepth(openingMm:Float,thicknessMm:Float,material:materia.casting.InsertMaterial,
		rule:InsertAnchorPatternRule):Float {
		validateRule(rule);
		if (!Math.isFinite(openingMm) || openingMm<0) throw "Invalid anchor-band opening";
		for (size in availableSizes()) {
			var hole:materia.casting.InsertHole;
			try hole=holeForStock(size,thicknessMm,material) catch (_:Dynamic) continue;
			var diameter=HexFlangeBolt.catalog().get(size).diameter;
			var radius=HexFlangeBolt.catalog().get(size).flangeDiameter/2;
			var edge=Math.max(hole.limits.minWall+diameter/2,radius);
			return openingMm+2*(edge+radius+rule.coverDiameters*diameter);
		}
		throw "No standard blind anchor thread fits the source stock thickness";
	}

	/** Resin land for the demand-admitted back pattern. Use the same footprint
	 * rule as the final native selection, rather than the smallest catalogue
	 * thread. Final casting containment and cover remain mandatory. */
	public static function minimumOuterLand(id:String,source:CastInsert,rule:InsertAnchorPatternRule,
		liftingShareN:Float):{x:Float,y:Float} {
		var sized=derive(id,source,rule,FootprintOnly,liftingShareN).insertSpec();
		var raw=switch materia.casting.InsertFinishing.assess(sized.body,sized.interfaces) {
			case Resolved(result):result;
			case Invalid(reason),Unsupported(reason):throw reason;
		};
		var stock=materia.casting.InsertFinishing.box(raw);
		if (stock==null) throw "Anchor outer land requires rectangular source steel: "+id;
		var land={x:0.0,y:0.0};
		switch sized.anchorage {
			case Bolted(rules,anchors):for (anchor in anchors) {
				var hole=sized.holes.filter(h -> h.id==anchor.holeId)[0];
				var margin=anchor.screw.flangeDiameterMm/2+rules.minimumCoverMm;
				land.x=Math.max(land.x,Math.max(stock.x-hole.frame.x+margin,
					hole.frame.x+margin-stock.x-stock.width));
				land.y=Math.max(land.y,Math.max(stock.y-hole.frame.y+margin,
					hole.frame.y+margin-stock.y-stock.depth));
			}
			default:throw "Missing demand-sized anchor pattern: "+id;
		}
		return land;
	}

	static function availableSizes():Array<String> {
		var catalog=HexFlangeBolt.catalog();
		var sizes=catalog.designations();
		sizes.sort((a,b) -> Reflect.compare(catalog.get(a).diameter,catalog.get(b).diameter));
		return sizes;
	}

	static function holeForStock(size:String,thickness:Float,material:materia.casting.InsertMaterial):materia.casting.InsertHole {
		var nominal=HexFlangeBolt.catalog().get(size);
		var thread=machinekit.standard.FastenerThreads.coarse(nominal.diameter).mating();
		return InsertHoles.records(InsertHoles.tappedBlind(InsertHoles.metricThread(thread),thickness,material))[0];
	}

	static function validateRule(rule:InsertAnchorPatternRule):Void {
		if (rule==null || rule.minimumCount<2 || rule.maximumCount<rule.minimumCount || rule.maximumCount>256
			|| rule.id==null || StringTools.trim(rule.id).length==0 || rule.demandFactor<1
			|| rule.liftingLoadPaths<1 || rule.gravityMps2<=0 || !Math.isFinite(rule.demandFactor+rule.gravityMps2)
			|| rule.maximumPitchInShortWidths<=0
			|| rule.embedmentDiameters<=0 || rule.coverDiameters<=0 || rule.spacingDiameters<=0
			|| !Math.isFinite(rule.maximumPitchInShortWidths+rule.embedmentDiameters+rule.coverDiameters+rule.spacingDiameters)
			|| rule.screwLengthsMm==null || rule.screwLengthsMm.length==0)
			throw "Invalid anchor pattern rule";
		for (length in rule.screwLengthsMm)
			if (!Math.isFinite(length) || length<=0) throw "Invalid declared standard anchor screw length";
	}

	public static function derive(id:String, source:CastInsert, rule:InsertAnchorPatternRule,context:InsertAnchorContext,liftingShareN:Float):CastInsert {
		try return build(source,rule,context,liftingShareN)
		catch (error:Dynamic) throw "Insert " + id + ": anchor pattern: " + Std.string(error);
	}

	static function build(source:CastInsert,rule:InsertAnchorPatternRule,context:InsertAnchorContext,liftingShareN:Float):CastInsert {
		validateRule(rule);
		if (!Math.isFinite(liftingShareN) || liftingShareN<0) throw "Invalid source lifting demand";
		var spec=source.insertSpec();
		switch spec.anchorage {case Undeclared:default:throw "Anchor pattern requires unanchored source steel";}
		var steel=switch materia.casting.InsertFinishing.assess(spec.body,spec.interfaces) {
			case Resolved(result):result;
			case Invalid(reason),Unsupported(reason):throw reason;
		};
		var stock=materia.casting.InsertFinishing.box(steel);
		if (stock==null) throw "Back-face anchor rule requires rectangular steel";
		var demand=InsertAnchorHardware.demand(spec,rule.id,rule.demandFactor,liftingShareN,[]);
		var total=demand.liftingShareN;
		for (load in demand.servicePreloads) total+=load.preloadN;
		total*=demand.factor;
		var candidates:Array<{size:String,diameter:Float,thread:cadkit.threads.ThreadSpec,rules:InsertAnchorRules,
			points:Array<InsertAnchorHardware.InsertBackAnchorChoice>,lengthMm:Float,landMm:Float,key:String}> = [];
		var failures:Array<String> = [];
		for (size in availableSizes()) {
			var nominal=HexFlangeBolt.catalog().get(size),diameter=nominal.diameter;
			var hole:materia.casting.InsertHole;
			try hole=holeForStock(size,stock.height,source.insertMaterial().grade)
			catch (error:Dynamic) {failures.push(size+": "+Std.string(error));continue;}
			var insertion=materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole).minEngagement;
			var holeThread=hole.thread;
			if (holeThread==null) throw "Missing resolved source anchor thread";
			var thread=InsertHoles.threadSpec(holeThread);
			var lengths=rule.screwLengthsMm.filter(length -> length>insertion && HexFlangeBolt.metric(size,length,machinekit.standard.PropertyClass.Steel88).threadedLength()>=insertion);
			lengths.sort(Reflect.compare);
			if (lengths.length==0) {failures.push(size+": no declared screw length leaves the flange outside the insert");continue;}
			var edge=Math.max(hole.limits.minWall+diameter/2,nominal.flangeDiameter/2);
			// A strip may end on an exposed casting face. Search both the steel
			// policy margin and the flange-plus-cover margin independently on each
			// axis; source resin beside the other edge need not be discarded.
			var coveredEdge=Math.max(edge,nominal.flangeDiameter/2+rule.coverDiameters*diameter);
			var edges=coveredEdge>edge ? [edge,coveredEdge] : [edge];
			for (edgeX in edges) for (edgeY in edges) {
				var sx=stock.width-2*edgeX,sy=stock.depth-2*edgeY;
				if (sx<0 || sy<0) {failures.push(size+": policy edge wall does not fit the footprint");continue;}
				var spacing=rule.spacingDiameters*diameter,step=nominal.flangeDiameter+spacing;
				var nxMax=Std.int(Math.min(rule.maximumCount,Math.floor(sx/step)+1));
				var nyMax=Std.int(Math.min(rule.maximumCount,Math.floor(sy/step)+1));
				var pitch=rule.maximumPitchInShortWidths*Math.min(stock.width,stock.depth);
				var limit=machinekit.design.ThreadedHolePolicy.assemblyPreloadN(thread);
				var rules:InsertAnchorRules={demand:demand,minimumEmbedmentMm:rule.embedmentDiameters*diameter,
					minimumCoverMm:rule.coverDiameters*diameter,minimumSpacingMm:spacing,
					minimumBearingAreaMm2:Math.PI/4*(nominal.bearingOuterDiameter*nominal.bearingOuterDiameter
						-nominal.bearingInnerDiameter*nominal.bearingInnerDiameter)};
				for (nx in 1...nxMax+1) for (ny in 1...nyMax+1) {
					if (nx*ny>rule.maximumCount || nx*ny<rule.minimumCount || nx*ny*limit<total) continue;
					if (sx>pitch && nx<2 || sy>pitch && ny<2
						|| nx>1 && sx/(nx-1)>pitch || ny>1 && sy/(ny-1)>pitch) continue;
					// Also pack the same grid at minimum flange spacing so an
					// off-centre service hole does not force sites beside it.
					var spansX=nx>1 && (nx-1)*step<sx ? [sx,(nx-1)*step] : [nx==1 ? 0.0 : sx];
					var spansY=ny>1 && (ny-1)*step<sy ? [sy,(ny-1)*step] : [ny==1 ? 0.0 : sy];
					for (spanX in spansX) for (spanY in spansY) {
						var offsetsX=spanX<sx ? [(sx-spanX)/2,0.0,sx-spanX] : [0.0];
						var offsetsY=spanY<sy ? [(sy-spanY)/2,0.0,sy-spanY] : [0.0];
						for (ox in offsetsX) for (oy in offsetsY) {
							var points:Array<InsertAnchorHardware.InsertBackAnchorChoice> = [];
							for (ix in 0...nx) for (iy in 0...ny) {
								var x=stock.x+edgeX+ox+(nx==1 ? 0.0 : ix*spanX/(nx-1));
								var y=stock.y+edgeY+oy+(ny==1 ? 0.0 : iy*spanY/(ny-1));
								var clear=true;
								for (other in spec.holes) {
									var axis=F.transformVector(other.frame,0,0,1);
									if (Math.abs(axis.x)+Math.abs(axis.y)>1e-8)
										throw "Mounting-hole direction needs an explicit back-pattern clearance construction: "+other.id;
									var radius=(other.thread==null ? other.diameter : other.thread.majorDiameter)/2;
									if (other.counterbore!=null) radius=Math.max(radius,other.counterbore.diameter/2);
									if (other.relief!=null) radius=Math.max(radius,other.relief.diameter/2);
									var dx=x-other.frame.x,dy=y-other.frame.y;
									if (Math.sqrt(dx*dx+dy*dy)<radius+diameter/2+InsertAnchorHardware.requiredHoleLigament(hole,other)) clear=false;
								}
								if (clear) points.push({id:"back-anchor-"+ix+"-"+iy,xMm:x,yMm:y,lengthMm:lengths[0],insertionMm:insertion,
									locking:InsertAnchorLock.RetainingCompound,lockingReference:null});
							}
							// Removing sites beside mounting holes must not leave a long
							// unsupported interval that the unfiltered grid did not have.
							var supported=true;
							for (axis in 0...2) {
								var low=axis==0 ? stock.x : stock.y,span=axis==0 ? stock.width : stock.depth;
								if (span<=pitch || points.length==0) continue;
								var positions=[for (point in points) axis==0 ? point.xMm : point.yMm];
								positions.sort(Reflect.compare);
								if (positions[0]-low>pitch || low+span-positions[positions.length-1]>pitch) supported=false;
								for (index in 1...positions.length)
									if (positions[index]-positions[index-1]>pitch) supported=false;
							}
							if (supported && points.length>=rule.minimumCount && points.length*limit>=total) {
								var margin=nominal.flangeDiameter/2+rules.minimumCoverMm,landX=0.0,landY=0.0;
								for (point in points) {
									landX=Math.max(landX,Math.max(stock.x+margin-point.xMm,point.xMm+margin-stock.x-stock.width));
									landY=Math.max(landY,Math.max(stock.y+margin-point.yMm,point.yMm+margin-stock.y-stock.depth));
								}
								var key=haxe.Json.stringify(points);
								for (length in lengths) candidates.push({size:size,diameter:diameter,thread:thread,rules:rules,
									points:points,lengthMm:length,landMm:landX+landY,key:key});
							}
						}
					}
				}
			}
		}
		// Meet demand first, minimize holes, then select the smallest passing
		// standard thread. Casting cover is an admission check, never a fallback.
		candidates.sort((a,b) -> {
			var count=Reflect.compare(a.points.length,b.points.length);
			if (count!=0) return count;
			var diameter=Reflect.compare(a.diameter,b.diameter);
			if (diameter!=0) return diameter;
			var length=Reflect.compare(a.lengthMm,b.lengthMm);
			if (length!=0) return length;
			// Prefer the positions needing least extra resin land, preserving the
			// demand/count/size ordering. Lexical coordinates do not locate steel.
			var land=Reflect.compare(a.landMm,b.landMm);
			if (land!=0) return land;
			return Reflect.compare(a.key,b.key);
		});
		var tested=new Map<String,Bool>();
		for (candidate in candidates) {
			var accepted=true;
			var selected=[for (point in candidate.points) {id:point.id,xMm:point.xMm,yMm:point.yMm,
				lengthMm:candidate.lengthMm,insertionMm:point.insertionMm,locking:point.locking,lockingReference:point.lockingReference}];
			for (point in selected) {
				var key=candidate.size+":"+point.xMm+":"+point.yMm+":"+point.lengthMm;
				if (!tested.exists(key)) tested.set(key,contained(InsertAnchorHardware.backPattern(source,candidate.thread,candidate.rules,[point]),
					context,candidate.rules.minimumCoverMm,candidate.rules.minimumEmbedmentMm));
				if (tested.get(key)!=true) {accepted=false;break;}
			}
			if (!accepted) continue;
			var result=InsertAnchorHardware.backPattern(source,candidate.thread,candidate.rules,selected);
			InsertAnchorHardware.parts(result.insertSpec());return result;
		}
		throw "No ISO 4162 pattern meets design demand "+total+" N, hole policy, footprint and casting cover; "
			+candidates.length+" demand-admissible patterns checked. "+failures.join("; ");
	}

	/** Candidate rejection uses exact native occupancy, not a sampled pose or a
	 * box stand-in. No rule is relaxed when the casting leaves too few sites. */
	static function contained(source:CastInsert,context:InsertAnchorContext,cover:Float,minimumEmbedment:Float):Bool {
		return switch context {
			case FootprintOnly:true;
			case Casting(body,pose):
				var owned:Array<Part> = [],accepted=true;
				function keep(part:Part):Part {owned.push(part);return part;}
				try {
					var spec=source.insertSpec();
					var anchor=switch spec.anchorage {case Bolted(_,anchors):anchors[0];default:throw "Missing anchor candidate";};
					var hole=spec.holes.filter(value -> value.id==anchor.holeId)[0];
					var bearing=F.compose(pose,materia.casting.InsertAnchorage.InsertAnchorageCodec.bearingFrame(hole,anchor));
					if (InsertAnchorHardware.axialEmbedment(F.transformPoint(bearing,0,0,0),F.transformVector(bearing,0,0,1),body)
						+1e-7<minimumEmbedment) return false;
					var exterior=keep(machinekit.assembly.PosedParts.moved(keep(InsertBodies.part(spec.body)),pose));
					var hardware:Array<Part> = [];
					for (component in InsertAnchorHardware.parts(spec)) {
						var shape=keep(machinekit.assembly.PosedParts.moved(keep(component.part.geometry()),F.compose(pose,component.pose)));
						shape.shape.setThreadedFaces([]);hardware.push(shape);
					}
					var exposed=keep(keep(Part.fuseAll(hardware)).subtract(exterior));
					if (keep(exposed.subtract(body)).volume()>1e-6) {
						accepted=false;
					}
					var bounds=exposed.shape.bounds(),lo=bounds.get_min(),hi=bounds.get_max();
					if (accepted) for (index in 0...body.shape.subshapeCount(CadKit.ShapeKind.Face)) {
						var face=body.shape.subshape(CadKit.ShapeKind.Face,index);
						try {
							var other=face.bounds(),a=other.get_min(),b=other.get_max();
							var dx=Math.max(0,Math.max(lo.get_x()-b.get_x(),a.get_x()-hi.get_x()));
							var dy=Math.max(0,Math.max(lo.get_y()-b.get_y(),a.get_y()-hi.get_y()));
							var dz=Math.max(0,Math.max(lo.get_z()-b.get_z(),a.get_z()-hi.get_z()));
							// Enclosing bounds supply a conservative distance lower bound;
							// only near faces need the actual surface-distance solve.
							if (dx*dx+dy*dy+dz*dz<cover*cover && exposed.shape.distance(face)+1e-7<cover) {
								accepted=false;
							}
						}
						catch (error:Dynamic) {face.close();throw error;}
						face.close();if (!accepted) break;
					}
				} catch (error:Dynamic) {for (part in owned) part.close();throw error;}
				for (part in owned) part.close();
				accepted;
		};
	}
}
