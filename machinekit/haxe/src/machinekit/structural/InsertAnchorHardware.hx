package machinekit.structural;

import machinekit.standard.HexFlangeBolt;
import machinekit.standard.PropertyClass;
import machinekit.component.MachineComponent;
import materia.casting.InsertAnchorage;
import materia.casting.InsertAnchorage.InsertAnchorageCodec as C;
import materia.casting.InsertSpec;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef InsertAnchorChoice = {
	var id:String;
	var holeId:String;
	var lengthMm:Float;
	var insertionMm:Float;
	var locking:InsertAnchorLock;
	var lockingReference:Null<String>;
}

typedef InsertBackAnchorChoice = {
	var id:String;
	var xMm:Float;
	var yMm:Float;
	var lengthMm:Float;
	var insertionMm:Float;
	var locking:InsertAnchorLock;
	var lockingReference:Null<String>;
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

/** Standard components are the dimension authority. Source choices reference
 * holes already declared by InsertHoles; no anchor hole is invented here. */
class InsertAnchorHardware {
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
				locking:point.locking,lockingReference:point.lockingReference});
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
				screw:screwRecord(screw),locking:choice.locking,lockingReference:choice.lockingReference});
		}
		var result=C.snapshot(Bolted(rules,anchors));C.references(result,spec.body,spec.holes,spec.origin,spec.interfaces);return result;
	}

	public static function parts(spec:InsertSpec):Array<InsertAnchorComponent> {
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
	public static function requiredHoleLigament(a:materia.casting.InsertHole,b:materia.casting.InsertHole):Float {
		return Math.max(Math.max(a.limits.minWall,b.limits.minWall),
			Math.max(a.limits.minClearance,b.limits.minClearance));
	}

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
}
