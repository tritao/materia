package machinekit.tooling;

import machinekit.standard.HexFlangeBolt;
import machinekit.standard.PropertyClass;
import machinekit.component.MachineComponent;
import materia.casting.InsertAnchorage;
import materia.casting.InsertSpec;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import machinekit.structural.CastInsert;
import machinekit.structural.InsertHoles;
import machinekit.structural.InsertBodies;
import cadkit.modeling.Part;
import machinekit.assembly.PosedParts;
import machinekit.structural.CastingCavityCad;
import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertAnchorage.InsertAnchorageCodec as C;
import materia.tooling.casting.CastingToolingInput;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;

class InsertAnchorScreenDefect extends haxe.Exception {}

enum InsertAnchorContext {
	FootprintOnly;
	Casting(body:Part,pose:materia.assembly.AssemblyRecord.AssemblyFrame);
}


enum abstract InsertAnchorComponentKind(String) to String {
	var Bolt = "bolt";
}

typedef InsertAnchorComponent = {
	var anchorId:String;
	var kind:InsertAnchorComponentKind;
	var part:MachineComponent;
	var pose:AssemblyFrame;
}


/** Native checks of source hardware. Strength, locking evidence and tooling
 * interference remain separate; source shape bounds never substitute for solids. */
class InsertAnchorGeometry {
	/** Actual source hardware against every realized tooling member. The insert's
	 * own threaded engagement and its own flange bearing geometry are intentional. */
	public static function reviewMold(plan:materia.tooling.casting.CastingMoldPlan):Array<ToolingValidation> {
		var components=CastingMoldCad.anchorComponents(plan);
		if (components.length==0) return [];
		var assembly=CastingMoldCad.assembly(plan),poses=assembly.solvedPoses();
		var parts:Array<Part> = [],placed:Map<String,Part> = [],bounds:Map<String,Array<Float>> = [],checks:Array<ToolingValidation> = [];
		try {
			for (member in assembly.components()) {
				var local=member.component.geometry();parts.push(local);
				var shape=PosedParts.moved(local,poses.get(member.id));parts.push(shape);
				shape.shape.setThreadedFaces([]);placed.set(member.id,shape);
				var box=shape.shape.bounds(),low=box.get_min(),high=box.get_max();
				bounds.set(member.id,[low.get_x(),low.get_y(),low.get_z(),high.get_x(),high.get_y(),high.get_z()]);
			}
			var seen:Map<String,Bool> = [];
			for (hardware in components) {
				var key=hardware.insertId+"/"+hardware.anchorId;
				if (seen.exists(key)) continue;
				seen.set(key,true);
				var own=[for (part in components) if (part.insertId==hardware.insertId && part.anchorId==hardware.anchorId) part.id];
				var conflicts:Array<String> = [];
				for (member in assembly.components()) {
					if (own.indexOf(member.id)>=0 || member.id==materia.tooling.ToolingIds.derive("insert-",hardware.insertId)) continue;
					for (id in own) {
						var a=bounds.get(id),b=bounds.get(member.id),disjoint=false;
						for (axis in 0...3)
							if (Math.min(a[axis+3],b[axis+3])<=Math.max(a[axis],b[axis])) disjoint=true;
						if (disjoint) continue;
						var anchor=placed.get(id),other=placed.get(member.id);
						var common=anchor.intersect(other);
						var volume:Float;
						try volume=common.volume() catch (error:Dynamic) {common.close();throw error;}
						common.close();
						if (volume>1e-6 && conflicts.indexOf(member.id)<0) conflicts.push(member.id);
					}
				}
				conflicts.sort(Reflect.compare);
				var finding=check(hardware.insertId,hardware.anchorId,"tooling-interference",conflicts.length==0,
					conflicts.length==0 ? "Exact source anchor hardware clears all realized mold members."
					: "Source anchor intersects mold members: "+conflicts.join(", "));
				finding.affectedIds=finding.affectedIds.concat(conflicts);checks.push(finding);
			}
		} catch (error:Dynamic) {for (part in parts) part.close();throw error;}
		for (part in parts) part.close();
		checks.sort((a,b) -> Reflect.compare(a.rule,b.rule));return checks;
	}

	public static function review(input:CastingToolingInput,?bearingStrengthMPa:Float,?tensileStrengthMPa:Float):Array<ToolingValidation> {
		if (input.inserts.filter(insert -> switch insert.anchorage {case Bolted(_,_):true;default:false;}).length==0) return [];
		var parts:Array<Part> = [],shapes:Array<cadkit.Shape> = [],result:Array<ToolingValidation> = [];
		function own(part:Part):Part {parts.push(part);return part;}
		function ownShape(shape:cadkit.Shape):cadkit.Shape {shapes.push(shape);return shape;}
		try {
			var envelope=[for (prism in input.envelope) own(CastingCavityCad.prism(prism))];
			var body=own(Part.fuseAll(envelope)),cuts:Array<Part> = [];
			// Steel recesses are interfaces inside the composite casting, not its
			// exposed surface. Other source cavities retain their exact boundaries.
			var recesses=[for (feature in input.features)
				if (feature.kind==materia.casting.CastingFeature.CastingFeatureKind.InsertRecess) feature.id];
			for (cavity in materia.casting.CastingCavities.derive(input.envelope,input.features))
				if (recesses.indexOf(cavity.featureId)<0) cuts.push(own(CastingCavityCad.part(cavity)));
			var bulk=own(body.subtractAll(cuts));
			var resin=own(ToolingSourceCasting.fromInput(input).geometry());
			for (insert in input.inserts) switch insert.anchorage {
				case Undeclared,CatalogueRetention:
				case Bolted(rules,anchors):
					var capacityGeometry:Array<{anchorId:String,bearingAreaMm2:Float,embedmentMm:Float}> = [];
					var spec:materia.casting.InsertSpec={body:insert.body,holes:insert.holes,origin:insert.origin,
						anchorage:insert.anchorage,interfaces:insert.interfaces};
					var exterior=own(PosedParts.moved(own(machinekit.structural.InsertBodies.part(insert.body)),insert.pose));
					var hardware=InsertAnchorGeometry.components(spec);
					for (anchor in anchors) {
						var metal:Array<Part> = [],bolt:Null<Part> = null;
						var bearingPose=F.identity();
						for (component in hardware) if (component.anchorId==anchor.id) {
							var pose=F.compose(insert.pose,component.pose);
							var placed=own(PosedParts.moved(own(component.part.geometry()),pose));metal.push(placed);
							if (component.kind==InsertAnchorComponentKind.Bolt) {bolt=placed;
								bearingPose=F.compose(insert.pose,C.bearingFrame(spec.holes.filter(h -> h.id==anchor.holeId)[0],anchor));}
						}
						var exposed=own(own(Part.fuseAll(metal)).subtractAll([exterior]));
						var outside=own(exposed.subtractAll([bulk]));
						var contained=outside.volume()<=1e-6,cover=contained ? boundaryDistance(exposed,bulk) : 0.0;
						result.push(check(insert.id,anchor.id,"cover",contained && cover+1e-7>=rules.minimumCoverMm,
							contained ? "Minimum exposed-casting boundary cover "+cover+" mm; requires "+rules.minimumCoverMm+" mm."
							: "Anchor hardware outside its steel insert extends into a source cavity or outside the casting."));
						if (bolt==null) throw "Missing source flange anchor: "+insert.id+"/"+anchor.id;
						var placedBolt:Part=bolt;
						var point=F.transformPoint(bearingPose,0,0,0);
						var normal=F.transformVector(bearingPose,0,0,1);
						var embedment=InsertAnchorGeometry.axialEmbedment(point,normal,bulk);
						result.push(check(insert.id,anchor.id,"embedment",embedment+1e-7>=rules.minimumEmbedmentMm,
							"Axial depth from flange bearing plane to the exposed casting boundary toward the loaded insert face "
							+embedment+" mm; requires "+rules.minimumEmbedmentMm+" mm."));
						var origin=cadkit.Geometry.vec3(point.x,point.y,point.z);
						var direction=cadkit.Geometry.vec3(normal.x,normal.y,normal.z);
						var face=ownShape(placedBolt.shape.section(origin,direction));
						var resinFace=ownShape(resin.shape.section(origin,direction));
						var contact=ownShape(face.planarCommon(resinFace)),area=contact.area();
						result.push(check(insert.id,anchor.id,"bearing-area",area+1e-7>=rules.minimumBearingAreaMm2,
							"Actual flange face bearing on nominal source resin "+area+" mm2; requires "+rules.minimumBearingAreaMm2
							+" mm2. Tooling interference and physical capacity are not qualified by this source-only check."));
						capacityGeometry.push({anchorId:anchor.id,bearingAreaMm2:area,embedmentMm:embedment});
					}
					result=result.concat(materia.casting.InsertAnchorage.InsertAnchorageChecks.materialChecks(insert,
						capacityGeometry,
						bearingStrengthMPa==null ? materia.casting.CastingInput.Unresolved("Cured mix bearing strength")
							: materia.casting.CastingInput.Known(bearingStrengthMPa),
						tensileStrengthMPa==null ? materia.casting.CastingInput.Unresolved("Cured mix tensile strength")
							: materia.casting.CastingInput.Known(tensileStrengthMPa)));
			}
		} catch (error:Dynamic) {for (shape in shapes) shape.close();for (part in parts) part.close();throw error;}
		for (shape in shapes) shape.close();
		for (part in parts) part.close();
		result.sort((a,b) -> Reflect.compare(a.rule,b.rule));return result;
	}

	static function boundaryDistance(part:Part,bulk:Part):Float {
		var minimum=Math.POSITIVE_INFINITY;
		for (index in 0...bulk.shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face=bulk.shape.subshape(CadKit.ShapeKind.Face,index);
			try minimum=Math.min(minimum,part.shape.distance(face))
			catch (error:Dynamic) {face.close();throw error;}
			face.close();
		}
		if (!Math.isFinite(minimum)) throw "Anchor casting boundary distance could not be evaluated";
		return minimum;
	}

	static function check(insertId:String,anchorId:String,suffix:String,passed:Bool,message:String):ToolingValidation
		return {rule:"tooling.insert.anchorage."+insertId+"."+anchorId+".cad."+suffix,affectedIds:[insertId],
			status:passed ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail,
			severity:passed ? ToolingSeverity.Info : ToolingSeverity.Error,message:message,evidence:[]};



	/** Source authoring: add only the declared back-hole pattern. Existing bought-part
	 * hole routes remain unchanged, including transfer drilling after casting. */
	public static function backPattern(source:CastInsert,thread:cadkit.threads.ThreadSpec,
		rules:InsertAnchorRules,points:Array<InsertBackAnchorChoice>):CastInsert {
		var spec=source.insertSpec();
		switch spec.anchorage {case Undeclared:default:throw "Source anchorage is already declared";}
		var manufacturing=switch spec.origin {
			case Fabricated(plan):plan;
			case Catalogue(_):throw "A catalogue insert cannot acquire fabricated back holes";
		};
		var steel=switch materia.casting.InsertFinishing.assess(spec.body,spec.interfaces) {
			case Resolved(result):result;
			case Invalid(reason),Unsupported(reason):throw "Resolve source anchor stock before authoring back holes: "+reason;
		};
		var stock=materia.casting.InsertFinishing.box(steel);
		if (stock==null) throw "Unsupported non-box source back-hole pattern";
		var choice=InsertHoles.metricThread(thread),anchors:Array<InsertAnchorChoice> = [];
		for (point in points) {
			materia.casting.CastingFeatureCodec.identifier(point.id);
			if (!Math.isFinite(point.xMm+point.yMm)) throw "Anchor coordinates must be finite: "+point.id;
			if (spec.holes.filter(hole -> hole.id==point.id).length>0) throw "Duplicate source anchor hole: "+point.id;
			var hole=InsertHoles.records(InsertHoles.tappedBlind(choice,stock.height,source.insertMaterial().grade))[0];
			hole.id=point.id;hole.frame=F.translation(point.xMm,point.yMm,stock.z);
			materia.casting.InsertStages.validateServiceHole(hole,stock);
			spec.holes.push(hole);
			manufacturing.holeRoutes.push({holeId:hole.id,route:materia.casting.InsertManufacturing.InsertHoleRoute.BeforeCasting});
			anchors.push({id:point.id,holeId:hole.id,lengthMm:point.lengthMm,insertionMm:point.insertionMm,
				locking:point.locking});
		}
		var selected=choose(spec,rules,anchors),values=source.values().copy();
		values.setToken("holes",InsertHoles.fromNeutral(spec.holes))
			.setToken("origin",haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(spec.origin)))
			.setToken("anchorage",haxe.Json.stringify(C.record(selected)));
		return new CastInsert(values);
	}

	public static function demand(spec:InsertSpec,ruleId:String,factor:Float,liftingShareN:Float,anchorHoleIds:Array<String>):InsertAnchorDemand {
		var occupied=C.occupiedHoleIds(spec.anchorage).concat(anchorHoleIds);
		var loads:Array<{holeId:String,preloadN:Float}> = [];
		for (hole in spec.holes) {
			var thread=hole.thread;
			if (thread!=null && occupied.indexOf(hole.id)<0)
				loads.push({holeId:hole.id,preloadN:machinekit.design.ThreadedHolePolicy.assemblyPreloadN(InsertHoles.threadSpec(thread))});
		}
		loads.sort((a,b) -> Reflect.compare(a.holeId,b.holeId));
		return {ruleId:ruleId,factor:factor,servicePreloads:loads,liftingShareN:liftingShareN,
			servicePreloadReference:machinekit.design.ThreadedHolePolicy.ASSEMBLY_PRELOAD_REFERENCE};
	}

	public static function choose(spec:InsertSpec,rules:InsertAnchorRules,choices:Array<InsertAnchorChoice>):InsertAnchorage {
		var anchors:Array<InsertBoltAnchor> = [];
		for (choice in choices) {
			var holes=spec.holes.filter(hole -> hole.id==choice.holeId);
			if (holes.length!=1 || holes[0].thread==null) throw "Anchor choice requires a declared source tap: "+choice.id;
			var thread=holes[0].thread;
			if (thread==null) throw "Missing source anchor thread";
			var screw=HexFlangeBolt.metric(thread.designation,choice.lengthMm,Steel88);
			anchors.push({id:choice.id,holeId:choice.holeId,insertionMm:choice.insertionMm,
				screw:screwRecord(screw),locking:choice.locking});
		}
		var result=C.copy(Bolted(rules,anchors));C.references(result,spec.body,spec.holes,spec.origin,spec.interfaces);return result;
	}

	public static function components(spec:InsertSpec):Array<InsertAnchorComponent> {
		var source=InsertSpecCodec.snapshot(InsertSpecCodec.record(spec)),result:Array<InsertAnchorComponent> = [];
		checkHoleClearance(source);
		var holes=materia.casting.InsertStages.holes(source,materia.casting.InsertStages.InsertStage.BeforeCasting);
		switch source.anchorage {
			case Undeclared,CatalogueRetention:
			case Bolted(_,anchors):
				for (anchor in anchors) {
					var matches=holes.filter(hole -> hole.id==anchor.holeId);
					if (matches.length!=1) throw "Missing physical pre-cast anchor hole: "+anchor.id;
					var hole=matches[0],service=source.holes.filter(h -> h.id==anchor.holeId)[0],bearing=C.bearingFrame(service,anchor);
					var screw=HexFlangeBolt.metric(anchor.screw.thread.designation,anchor.screw.lengthMm,Steel88);
					if (!sameScrew(screwRecord(screw),anchor.screw))
						throw "Anchor hardware differs from selected standard components: "+anchor.id;
					var headInRaw=F.compose(F.inverse(hole.frame),bearing);
					if (headInRaw.z>1e-8)
						throw "Anchor flange intersects pre-cast steel: "+anchor.id;
					var failures=materia.casting.ThreadedFastenerFit.checkIntervals(service,service.frame,
						{shankLength:anchor.screw.lengthMm,threadedLength:anchor.screw.threadedLengthMm,bearingFrame:bearing},anchor.id)
						.concat(materia.casting.ThreadedFastenerFit.checkIntervals(hole,hole.frame,
							{shankLength:anchor.screw.lengthMm,threadedLength:anchor.screw.threadedLengthMm,bearingFrame:bearing},anchor.id))
						.filter(finding -> !finding.passed);
					if (failures.length>0) throw "Anchor source thread fit fails: "+anchor.id+": "+[for (f in failures) f.message].join("; ");
					result.push({anchorId:anchor.id,kind:InsertAnchorComponentKind.Bolt,part:screw,pose:F.compose(bearing,F.inverse(screw.bearingFrame()))});
				}
		}
		return result;
	}

	/** Both holes must retain their declared material wall. Screw diametral
	 * clearance alone cannot protect the steel between intersecting bore spans. */
	public static function requiredHoleLigament(a:materia.casting.InsertHole,b:materia.casting.InsertHole):Float
		return InsertAnchorSelection.requiredHoleLigament(a,b);

	/** Exact separation of the declared bore/major-thread design envelopes, in
	 * insert coordinates. Counterbores and reliefs retain their own axial spans.
	 * No mesh, endpoint-only distance or parallel-axis assumption is used. */
	static function checkHoleClearance(spec:InsertSpec):Void {
		var occupied=C.occupiedHoleIds(spec.anchorage);
		if (occupied.length==0) return;
		var shapes:Map<String,cadkit.modeling.Part> = [];
		function envelope(hole:materia.casting.InsertHole):cadkit.modeling.Part {
			var cached=shapes.get(hole.id);if (cached!=null) return cached;
			var shape=machinekit.component.Solids.building([],owned -> {
				function add(radius:Float,from:Float,to:Float):Void
					owned.push(machinekit.design.DesignGeometry.envelope({id:hole.id,frame:hole.frame,
						volume:CylinderVolume(radius,from,to)}));
				add((hole.thread==null ? hole.diameter : hole.thread.majorDiameter)/2,0,hole.depth);
				if (hole.counterbore!=null) add(hole.counterbore.diameter/2,0,hole.counterbore.depth);
				if (hole.relief!=null) add(hole.relief.diameter/2,hole.relief.start,hole.depth);
				return machinekit.component.Solids.union(owned);
			});
			shapes.set(hole.id,shape);return shape;
		}
		try {
			for (id in occupied) {
				var hole=spec.holes.filter(candidate -> candidate.id==id)[0];
				for (other in spec.holes) if (other.id!=id) {
					var minimum=requiredHoleLigament(hole,other);
					var first=envelope(hole),second=envelope(other);
					var firstBounds=first.shape.bounds(),secondBounds=second.shape.bounds();
					var a=firstBounds.get_min(),b=firstBounds.get_max();
					var c=secondBounds.get_min(),d=secondBounds.get_max();
					var dx=Math.max(0,Math.max(a.get_x()-d.get_x(),c.get_x()-b.get_x()));
					var dy=Math.max(0,Math.max(a.get_y()-d.get_y(),c.get_y()-b.get_y()));
					var dz=Math.max(0,Math.max(a.get_z()-d.get_z(),c.get_z()-b.get_z()));
					// Box distance is a lower bound: only definitely separated holes
					// bypass the exact solid distance, including rotated bore spans.
					if (Math.sqrt(dx*dx+dy*dy+dz*dz)>=minimum) continue;
					var measured=first.distance(second);
					if (measured+1e-7<minimum)
						throw "Anchor "+id+": source hole "+other.id+" clearance "+measured+" mm; requires "+minimum+" mm";
				}
			}
		} catch (error:Dynamic) {for (shape in shapes) shape.close();throw error;}
		for (shape in shapes) shape.close();
	}

	static function sameScrew(a:InsertAnchorScrew,b:InsertAnchorScrew):Bool {
		var x=a.thread,y=b.thread;
		return a.partNumber==b.partNumber && a.lengthMm==b.lengthMm && a.threadedLengthMm==b.threadedLengthMm
			&& a.headHeightMm==b.headHeightMm && a.headAcrossFlatsMm==b.headAcrossFlatsMm
			&& a.flangeDiameterMm==b.flangeDiameterMm && a.flangeThicknessMm==b.flangeThicknessMm
			&& a.bearingInnerDiameterMm==b.bearingInnerDiameterMm && a.bearingOuterDiameterMm==b.bearingOuterDiameterMm
			&& a.assemblyPreloadN==b.assemblyPreloadN && a.preloadReference==b.preloadReference
			&& x.designation==y.designation && x.majorDiameter==y.majorDiameter && x.pitch==y.pitch
			&& x.starts==y.starts && x.internal==y.internal && x.toleranceClass==y.toleranceClass
			&& x.hand==y.hand && x.profile==y.profile;
	}

	static function screwRecord(part:HexFlangeBolt):InsertAnchorScrew
		return {partNumber:part.bom.partNumber,thread:InsertHoles.threadRecord(part.threadSpec()),lengthMm:part.shankLength(),
			threadedLengthMm:part.threadedLength(),headHeightMm:part.spec.headHeight,headAcrossFlatsMm:part.spec.acrossFlats,
			flangeDiameterMm:part.spec.flangeDiameter,flangeThicknessMm:part.spec.flangeThickness,
			bearingInnerDiameterMm:part.spec.bearingInnerDiameter,bearingOuterDiameterMm:part.spec.bearingOuterDiameter,
			assemblyPreloadN:machinekit.design.ThreadedHolePolicy.assemblyPreloadN(part.threadSpec()),
			preloadReference:machinekit.design.ThreadedHolePolicy.ASSEMBLY_PRELOAD_REFERENCE};
	/** Exact ray exit to the exposed composite-casting boundary. Steel/resin
	 * interfaces do not substitute for that surface. Used by source sizing and
	 * its independent tooling review; source geometry is borrowed. */
	public static function axialEmbedment(point:materia.assembly.AssemblyDefinition.AssemblyVector,
		direction:materia.assembly.AssemblyDefinition.AssemblyVector,bulk:cadkit.modeling.Part):Float {
		var bounds=bulk.shape.bounds(),lo=bounds.get_min(),hi=bounds.get_max();
		var dx=hi.get_x()-lo.get_x(),dy=hi.get_y()-lo.get_y(),dz=hi.get_z()-lo.get_z();
		var length=2*Math.sqrt(dx*dx+dy*dy+dz*dz);
		var ray=cadkit.modeling.Curve.line(new cadkit.modeling.Vector(point.x,point.y,point.z),
			new cadkit.modeling.Vector(point.x+direction.x*length,point.y+direction.y*length,point.z+direction.z*length));
		var depth=Math.POSITIVE_INFINITY;
		var intersection:Null<cadkit.Shape> = null;
		try {
			intersection=bulk.shape.common(ray.shape);
			for (vertex in intersection.vertices().all()) {
				var p=vertex.position();
				var distance=(p.get_x()-point.x)*direction.x+(p.get_y()-point.y)*direction.y+(p.get_z()-point.z)*direction.z;
				vertex.close();
				// The ray starts inside the qualified bulk. Its own start vertex
				// is not a boundary event; the first other interval end is.
				if (distance>1e-7) depth=Math.min(depth,distance);
			}
		} catch (error:Dynamic) {if (intersection!=null) intersection.close();ray.close();throw error;}
		intersection.close();
		ray.close();
		if (!Math.isFinite(depth)) throw "Anchor axial embedment has no exposed casting boundary";
		return depth;
	}


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
		InsertAnchorSelection.validateRule(rule);
		if (!Math.isFinite(openingMm) || openingMm<0) throw "Invalid anchor-band opening";
		for (size in availableSizes()) {
			if (rule.allowedNominalDiametersMm.indexOf(HexFlangeBolt.catalog().get(size).diameter)<0) continue;
			var hole:materia.casting.InsertHole;
			try hole=holeForStock(size,thicknessMm,material) catch (_:Dynamic) continue;
			var diameter=HexFlangeBolt.catalog().get(size).diameter;
			var radius=HexFlangeBolt.catalog().get(size).flangeDiameter/2;
			var edge=Math.max(hole.limits.minWall+diameter/2,radius);
			return openingMm+2*(edge+radius+rule.coverDiameters*diameter);
		}
		throw "No standard blind anchor thread fits the source stock thickness";
	}

	/** Two lands beside an access pocket each carry a row of anchors. This
	 * demand-side floor precedes the unchanged full footprint/cover selector. */
	public static function minimumAnchorBandWidth(source:CastInsert,rule:InsertAnchorPatternRule):Float {
		if (rule.allowedNominalDiametersMm.length!=1) throw "Band width needs one selected anchor thread";
		var diameter=rule.allowedNominalDiametersMm[0];
		var thread=machinekit.standard.FastenerThreads.coarse(diameter).mating();
		var nominal=HexFlangeBolt.catalog().get(thread.designation);
		var policy=machinekit.design.ThreadedHolePolicy.resolve(thread,source.insertMaterial().grade);
		var demand=demand(source.insertSpec(),rule.id,rule.demandFactor,0,[]),load=0.0;
		for (service in demand.servicePreloads) load+=service.preloadN;
		var count=Math.max(rule.minimumCount,Math.ceil(load*demand.factor/
			machinekit.design.ThreadedHolePolicy.assemblyPreloadN(thread)));
		if (count>rule.maximumCount) throw "Anchor demand exceeds the declared count limit";
		var columns=Math.ceil(count/2),edge=Math.max(policy.minWall+diameter/2,nominal.flangeDiameter/2);
		return 2*edge+(columns-1)*(nominal.flangeDiameter+rule.spacingDiameters*diameter);
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


	public static function derive(id:String, source:CastInsert, rule:InsertAnchorPatternRule,context:InsertAnchorContext,liftingShareN:Float):CastInsert {
		try return build(source,rule,context,liftingShareN)
		catch (error:Dynamic) {
			if (Std.isOfType(error,InsertAnchorScreenDefect)) throw error;
			throw "Insert " + id + ": anchor pattern: " + Std.string(error);
		}
	}

	static function build(source:CastInsert,rule:InsertAnchorPatternRule,context:InsertAnchorContext,liftingShareN:Float):CastInsert {
		InsertAnchorSelection.validateRule(rule);
		if (!Math.isFinite(liftingShareN) || liftingShareN<0) throw "Invalid source lifting demand";
		var spec=source.insertSpec();
		switch spec.anchorage {case Undeclared:default:throw "Anchor pattern requires unanchored source steel";}
		var steel=switch materia.casting.InsertFinishing.assess(spec.body,spec.interfaces) {
			case Resolved(result):result;
			case Invalid(reason),Unsupported(reason):throw reason;
		};
		var stock=materia.casting.InsertFinishing.box(steel);
		if (stock==null) throw "Back-face anchor rule requires rectangular steel";
		var choices:Array<InsertAnchorStockChoice>=[],failures:Array<String>=[];
		for (size in availableSizes()) {
			if (rule.allowedNominalDiametersMm.indexOf(HexFlangeBolt.catalog().get(size).diameter)<0) continue;
			var nominal=HexFlangeBolt.catalog().get(size),hole:materia.casting.InsertHole;
			try hole=holeForStock(size,stock.height,source.insertMaterial().grade)
			catch (error:Dynamic) {failures.push(size+": "+Std.string(error));continue;}
			var insertion=materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole).minEngagement;
			var lengths=rule.screwLengthsMm.filter(length -> length>insertion
				&& HexFlangeBolt.metric(size,length,Steel88).threadedLength()>=insertion);
			lengths.sort(Reflect.compare);
			if (lengths.length==0) {failures.push(size+": no declared screw length leaves the flange outside the insert");continue;}
			if (hole.thread==null) throw "Missing resolved source anchor thread";
			choices.push({size:size,diameter:nominal.diameter,flangeDiameter:nominal.flangeDiameter,
				bearingInnerDiameter:nominal.bearingInnerDiameter,bearingOuterDiameter:nominal.bearingOuterDiameter,
				hole:hole,lengths:lengths,assemblyPreloadN:machinekit.design.ThreadedHolePolicy.assemblyPreloadN(InsertHoles.threadSpec(hole.thread))});
		}
		var boundary=switch context {case FootprintOnly:[];case Casting(body,pose):anchorBoundary(body,pose,stock,choices,rule);};
		var tested=new Map<String,Bool>(),screws=new Map<String,InsertAnchorScrew>();
		var candidate=InsertAnchorSelection.select(spec,rule,demand(spec,rule.id,rule.demandFactor,liftingShareN,[]),choices,failures,item -> {
			for (point in item.points) {
				var selected={id:point.id,xMm:point.xMm,yMm:point.yMm,lengthMm:item.lengthMm,
					insertionMm:point.insertionMm,locking:point.locking};
				var key=item.size+":"+point.xMm+":"+point.yMm+":"+item.lengthMm;
				if (!tested.exists(key)) {
					var screwKey=item.size+":"+item.lengthMm;
					if (!screws.exists(screwKey)) screws.set(screwKey,screwRecord(HexFlangeBolt.metric(item.size,item.lengthMm,Steel88)));
					var accepted=switch context {
						case FootprintOnly:true;
						case Casting(_,pose):InsertAnchorScreen.accepts(selected,screws.get(screwKey),stock.z,pose,boundary,
							item.rules.minimumCoverMm,item.rules.minimumEmbedmentMm);
					};
					tested.set(key,accepted);
				}
				if (tested.get(key)!=true) return false;
			}
			return true;
		});
		var selected=[for (point in candidate.points) {id:point.id,xMm:point.xMm,yMm:point.yMm,
			lengthMm:candidate.lengthMm,insertionMm:point.insertionMm,locking:point.locking}];
		var result=backPattern(source,InsertHoles.threadSpec(candidate.thread),candidate.rules,selected);
		if (!contained(result,context,candidate.rules.minimumCoverMm,candidate.rules.minimumEmbedmentMm))
			throw new InsertAnchorScreenDefect("Analytic anchor screen disagrees with native confirmation; candidate must not fall back");
		components(result.insertSpec());return result;
	}

	/** Exact query window, not a replacement casting envelope. Every possible
	 * anchor, its required cover and its embedment test segment lie inside it.
	 * Artificial window faces therefore cannot change an admission predicate.
	 * Cropping once avoids unrelated curved faces elsewhere in the casting;
	 * the chosen pattern is still confirmed against the complete original body.
	 */
	static function anchorBoundary(body:Part,pose:AssemblyFrame,stock:materia.casting.Box3,
			choices:Array<InsertAnchorStockChoice>,rule:InsertAnchorPatternRule):Array<AnchorFace> {
		if (choices.length==0) return [];
		var radius=0.0,cover=0.0,embedment=0.0,length=0.0,head=0.0;
		for (choice in choices) {
			radius=Math.max(radius,choice.flangeDiameter/2);
			cover=Math.max(cover,rule.coverDiameters*choice.diameter);
			embedment=Math.max(embedment,rule.embedmentDiameters*choice.diameter);
			head=Math.max(head,HexFlangeBolt.catalog().get(choice.size).headHeight);
			for (value in choice.lengths) length=Math.max(length,value);
		}
		var margin=radius+cover;
		var low=stock.z-length-head-margin;
		var high=stock.z+stock.height+embedment+margin;
		var scope=new cadkit.modeling.Scope();
		try {
			var local=scope.own(Part.box(stock.width+2*margin,stock.depth+2*margin,high-low));
			var window=scope.own(PosedParts.moved(local,F.compose(pose,
				F.translation(stock.x+stock.width/2,stock.y+stock.depth/2,low))));
			var region=scope.own(body.intersect(window));
			var result=planarBoundary(region);
			scope.close();
			return result;
		} catch (error:Dynamic) {
			scope.close();
			throw error;
		}
	}

	/** Copy exact trimmed planar boundaries once per selector invocation. The
	 * body already comes from the shared finished-cavity realization; no local
	 * clipping, tessellation or bounding-box cavity substitution occurs here. */
	public static function planarBoundary(body:Part):Array<AnchorFace> {
		var result:Array<AnchorFace>=[];
		for (i in 0...body.shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face=body.shape.subshape(CadKit.ShapeKind.Face,i);
			try {
				if (face.surfaceKind()!=CadKit.SurfaceKind.Plane)
					throw "Anchor analytic screen requires a declared planar source boundary";
				var n=face.faceNormal(),edges:Array<AnchorEdge>=[];
				for (j in 0...face.subshapeCount(CadKit.ShapeKind.Edge)) {
					var edge=face.subshape(CadKit.ShapeKind.Edge,j);
					try {
						if (edge.curveKind()!=CadKit.CurveKind.Line)
							throw "Anchor analytic screen requires exact straight boundary edges";
						var a=edge.positionAt(0),b=edge.positionAt(1);
						edges.push({a:{x:a.get_x(),y:a.get_y(),z:a.get_z()},b:{x:b.get_x(),y:b.get_y(),z:b.get_z()}});
					} catch (error:Dynamic) {edge.close();throw error;}
					edge.close();
				}
				if (edges.length<3) throw "Anchor source face has no finite planar trim";
				result.push({normal:{x:n.get_x(),y:n.get_y(),z:n.get_z()},edges:edges});
			} catch (error:Dynamic) {face.close();throw error;}
			face.close();
		}
		return result;
	}

	/** Chosen-pattern confirmation. Analytic admission never substitutes for
	 * native occupancy, all-anchor embedment or finite-face cover. */
	static function contained(source:CastInsert,context:InsertAnchorContext,cover:Float,minimumEmbedment:Float):Bool {
		return switch context {
			case FootprintOnly:true;
			case Casting(body,pose):
				var owned:Array<Part> = [],accepted=true;
				function keep(part:Part):Part {owned.push(part);return part;}
				try {
					var spec=source.insertSpec();
					var anchors=switch spec.anchorage {case Bolted(_,anchors):anchors;default:throw "Missing anchor candidate";};
					for (anchor in anchors) {
						var hole=spec.holes.filter(value -> value.id==anchor.holeId)[0];
						var bearing=F.compose(pose,C.bearingFrame(hole,anchor));
						if (axialEmbedment(F.transformPoint(bearing,0,0,0),F.transformVector(bearing,0,0,1),body)
							+1e-7<minimumEmbedment) return false;
					}
					var exterior=keep(machinekit.assembly.PosedParts.moved(keep(InsertBodies.part(spec.body)),pose));
					var hardware:Array<Part> = [];
					for (component in InsertAnchorGeometry.components(spec)) {
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


	public static function embeddedParts(rows:Array<EmbeddedInsertAnchors>):Array<Part> {
		var result:Array<Part> = [],cache=new PosedParts();
		try {
			for (row in rows) {
				var components:Array<InsertAnchorGeometry.InsertAnchorComponent>;
				try components=InsertAnchorGeometry.components(row.spec)
				catch (error:Dynamic) throw "Insert "+row.owner+": "+Std.string(error);
				for (hardware in components) {
					var occupancy=cache.posed(hardware.part,F.compose(row.pose,hardware.pose));
					result.push(occupancy);
					// This owned copy represents excluded resin volume, not an external
					// screw machining interface on the resulting casting. Keep the exact
					// B-rep; semantic threads remain on the real component in the BOM.
					occupancy.shape.setThreadedFaces([]);
			}
			}
		} catch (error:Dynamic) {for (part in result) part.close();cache.close();throw error;}
		cache.close();return result;
	}
}
