package machinekit.structural;

import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.FlatWasher;
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
	var Washer = "washer";
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

	public static function choose(spec:InsertSpec,rules:InsertAnchorRules,choices:Array<InsertAnchorChoice>):InsertAnchorage {
		var anchors:Array<InsertBoltAnchor> = [];
		for (choice in choices) {
			var holes=spec.holes.filter(hole -> hole.id==choice.holeId);
			if (holes.length!=1 || holes[0].thread==null) throw "Anchor choice requires a declared source tap: "+choice.id;
			var thread=holes[0].thread;
			if (thread==null) throw "Missing source anchor thread";
			var screw=SocketHeadCapScrew.metric(thread.designation,choice.lengthMm,Steel88),washer=FlatWasher.metric(thread.designation);
			anchors.push({id:choice.id,holeId:choice.holeId,insertionMm:choice.insertionMm,
				screw:screwRecord(screw),washer:washerRecord(washer),locking:choice.locking,lockingReference:choice.lockingReference});
		}
		var result=C.snapshot(Bolted(rules,anchors));C.references(result,spec.body,spec.holes,spec.origin,spec.interfaces);return result;
	}

	public static function parts(spec:InsertSpec):Array<InsertAnchorComponent> {
		var source=InsertSpecCodec.snapshot(spec),result:Array<InsertAnchorComponent> = [];
		checkHoleClearance(source);
		var holes=materia.casting.InsertStages.holes(source,materia.casting.InsertStages.InsertStage.BeforeCasting);
		switch source.anchorage {
			case Undeclared,CatalogueRetention:
			case Bolted(_,anchors):
				for (anchor in anchors) {
					var matches=holes.filter(hole -> hole.id==anchor.holeId);
					if (matches.length!=1) throw "Missing physical pre-cast anchor hole: "+anchor.id;
					var hole=matches[0],service=source.holes.filter(h -> h.id==anchor.holeId)[0],bearing=C.bearingFrame(service,anchor);
					var screw=SocketHeadCapScrew.metric(anchor.screw.thread.designation,anchor.screw.lengthMm,Steel88);
					var washer=FlatWasher.metric(anchor.screw.thread.designation);
					if (!sameScrew(screwRecord(screw),anchor.screw) || !sameWasher(washerRecord(washer),anchor.washer))
						throw "Anchor hardware differs from selected standard components: "+anchor.id;
					var headInRaw=F.compose(F.inverse(hole.frame),bearing);
					if (headInRaw.z+anchor.washer.thicknessMm>1e-8)
						throw "Anchor washer intersects pre-cast steel: "+anchor.id;
					var failures=materia.casting.ThreadedFastenerFit.checkIntervals(service,service.frame,
						{shankLength:anchor.screw.lengthMm,threadedLength:anchor.screw.threadedLengthMm,bearingFrame:bearing},anchor.id)
						.concat(materia.casting.ThreadedFastenerFit.checkIntervals(hole,hole.frame,
							{shankLength:anchor.screw.lengthMm,threadedLength:anchor.screw.threadedLengthMm,bearingFrame:bearing},anchor.id))
						.filter(finding -> !finding.passed);
					if (failures.length>0) throw "Anchor source thread fit fails: "+anchor.id+": "+[for (f in failures) f.message].join("; ");
					result.push({anchorId:anchor.id,kind:InsertAnchorComponentKind.Bolt,part:screw,pose:F.compose(bearing,F.inverse(screw.bearingFrame()))});
					result.push({anchorId:anchor.id,kind:InsertAnchorComponentKind.Washer,part:washer,pose:bearing});
				}
		}
		return result;
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
					var minimum=Math.max(hole.limits.minClearance,other.limits.minClearance);
					var measured=envelope(hole).distance(envelope(other));
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
			&& a.headDiameterMm==b.headDiameterMm && a.headHeightMm==b.headHeightMm
			&& a.socketSizeMm==b.socketSizeMm && a.socketDepthMm==b.socketDepthMm
			&& x.designation==y.designation && x.majorDiameter==y.majorDiameter && x.pitch==y.pitch
			&& x.starts==y.starts && x.internal==y.internal && x.toleranceClass==y.toleranceClass && x.hand==y.hand && x.profile==y.profile;
	}
	static function sameWasher(a:InsertAnchorWasher,b:InsertAnchorWasher):Bool
		return a.partNumber==b.partNumber && a.innerDiameterMm==b.innerDiameterMm
			&& a.outerDiameterMm==b.outerDiameterMm && a.thicknessMm==b.thicknessMm;

	static function screwRecord(part:SocketHeadCapScrew):InsertAnchorScrew
		return {partNumber:part.bom.partNumber,thread:InsertHoles.threadRecord(part.threadSpec()),lengthMm:part.shankLength(),
			threadedLengthMm:part.threadedLength(),headDiameterMm:part.spec.headDiameter,headHeightMm:part.spec.headHeight,
			socketSizeMm:part.spec.socketSize,socketDepthMm:part.spec.socketDepth};
	static function washerRecord(part:FlatWasher):InsertAnchorWasher
		return {partNumber:part.bom.partNumber,innerDiameterMm:part.innerDiameter,outerDiameterMm:part.outerDiameter,thicknessMm:part.thickness};
}
