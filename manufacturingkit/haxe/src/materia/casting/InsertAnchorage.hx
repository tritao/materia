package materia.casting;

import materia.tooling.casting.CastingToolingInput;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertHole;

/** ISO 4162 Type F class 8.8 hardware facts copied from the standard component.
 * The assembly preload is a steel-side sizing limit, never mix capacity. */
typedef InsertAnchorScrew = {
	var partNumber:String;
	var thread:InsertThread;
	var lengthMm:Float;
	var threadedLengthMm:Float;
	var headHeightMm:Float;
	var headAcrossFlatsMm:Float;
	var flangeDiameterMm:Float;
	var flangeThicknessMm:Float;
	var bearingInnerDiameterMm:Float;
	var bearingOuterDiameterMm:Float;
	var assemblyPreloadN:Float;
	var preloadReference:String;
}

typedef InsertAnchorDemand = {
	var ruleId:String;
	var factor:Float;
	var servicePreloads:Array<{holeId:String,preloadN:Float}>;
	var servicePreloadReference:String;
	var liftingShareN:Float;
}

enum InsertAnchorLock {
	Shoulder;
	RetainingCompound(reference:CastingInput.CastingTextInput);
}

/** Limits are source design rules, not unverified evidence or implicit defaults.
 * Embedment and cover refer to the surrounding casting's exposed boundary and
 * must be qualified with that geometry, not estimated from the insert thickness.
 */
typedef InsertAnchorRules = {
	var demand:InsertAnchorDemand;
	var minimumEmbedmentMm:Float;
	var minimumBearingAreaMm2:Float;
	var minimumSpacingMm:Float;
	var minimumCoverMm:Float;
}

typedef InsertBoltAnchor = {
	var id:String;
	var holeId:String;
	/** Tip insertion measured from the source finished-steel hole entry.
	 * Finishing stock never counts toward permanent thread engagement. */
	var insertionMm:Float;
	var screw:InsertAnchorScrew;
	var locking:InsertAnchorLock;
}

enum InsertAnchorage {
	Undeclared;
	Bolted(rules:InsertAnchorRules,anchors:Array<InsertBoltAnchor>);
	CatalogueRetention;
}

class InsertAnchorageCodec {
	public static function copy(body:InsertAnchorage):InsertAnchorage return snapshot(record(body));

	public static function snapshot(raw:Dynamic):InsertAnchorage {
		return switch CastingRecordCodec.string(raw,"kind") {
			case "undeclared":CastingFeatureCodec.fields(raw,["kind"]);Undeclared;
			case "catalogue-retention":CastingFeatureCodec.fields(raw,["kind"]);CatalogueRetention;
			case "bolted":
				CastingFeatureCodec.fields(raw,["kind","rules","anchors"]);
				var r=Reflect.field(raw,"rules");
				CastingFeatureCodec.fields(r,["demand","minimumEmbedmentMm","minimumBearingAreaMm2","minimumSpacingMm","minimumCoverMm"]);
				var d=Reflect.field(r,"demand");
				CastingFeatureCodec.fields(d,["ruleId","factor","servicePreloads","servicePreloadReference","liftingShareN"]);
				var loads:Array<{holeId:String,preloadN:Float}> = [],loadIds:Map<String,Bool> = [];
				for (load in CastingRecordCodec.array(d,"servicePreloads")) {
					CastingFeatureCodec.fields(load,["holeId","preloadN"]);
					var holeId=CastingRecordCodec.string(load,"holeId");CastingFeatureCodec.identifier(holeId);
					if (loadIds.exists(holeId)) throw "Duplicate anchor service preload: "+holeId;
					loadIds.set(holeId,true);loads.push({holeId:holeId,preloadN:positive(load,"preloadN")});
				}
				loads.sort((a,b) -> Reflect.compare(a.holeId,b.holeId));
				var lifting=CastingRecordCodec.number(d,"liftingShareN"),factor=positive(d,"factor");
				if (lifting<0 || factor<1) throw "Invalid anchor design demand";
				var demand:InsertAnchorDemand={ruleId:text(d,"ruleId"),factor:factor,servicePreloads:loads,
					servicePreloadReference:text(d,"servicePreloadReference"),liftingShareN:lifting};
				var rules:InsertAnchorRules={demand:demand,minimumEmbedmentMm:positive(r,"minimumEmbedmentMm"),
					minimumBearingAreaMm2:positive(r,"minimumBearingAreaMm2"),minimumSpacingMm:positive(r,"minimumSpacingMm"),
					minimumCoverMm:positive(r,"minimumCoverMm")};
				var anchors:Array<InsertBoltAnchor> = [],ids:Map<String,Bool> = [],holes:Map<String,Bool> = [];
				for (a in CastingRecordCodec.array(raw,"anchors")) {
					CastingFeatureCodec.fields(a,["id","holeId","insertionMm","screw","locking"]);
					var id=CastingRecordCodec.string(a,"id"),holeId=CastingRecordCodec.string(a,"holeId");
					CastingFeatureCodec.identifier(id);CastingFeatureCodec.identifier(holeId);
					if (ids.exists(id) || holes.exists(holeId)) throw "Duplicate anchor ID or source hole: "+id;
					ids.set(id,true);holes.set(holeId,true);
					var s=Reflect.field(a,"screw");
					CastingFeatureCodec.fields(s,["partNumber","thread","lengthMm","threadedLengthMm","headHeightMm","headAcrossFlatsMm",
						"flangeDiameterMm","flangeThicknessMm","bearingInnerDiameterMm","bearingOuterDiameterMm","assemblyPreloadN","preloadReference"]);
					var thread=InsertThreadCodec.snapshot(Reflect.field(s,"thread"));
					if (thread.internal || thread.profile!=InsertThreadProfile.Metric) throw "Anchor requires an external metric ISO 4162 class 8.8 bolt: "+id;
					var screw:InsertAnchorScrew={partNumber:text(s,"partNumber"),thread:thread,lengthMm:positive(s,"lengthMm"),
						threadedLengthMm:positive(s,"threadedLengthMm"),headHeightMm:positive(s,"headHeightMm"),
						headAcrossFlatsMm:positive(s,"headAcrossFlatsMm"),flangeDiameterMm:positive(s,"flangeDiameterMm"),
						flangeThicknessMm:positive(s,"flangeThicknessMm"),bearingInnerDiameterMm:positive(s,"bearingInnerDiameterMm"),
						bearingOuterDiameterMm:positive(s,"bearingOuterDiameterMm"),assemblyPreloadN:positive(s,"assemblyPreloadN"),
						preloadReference:text(s,"preloadReference")};
					if (screw.flangeThicknessMm>=screw.headHeightMm || screw.headAcrossFlatsMm<=thread.majorDiameter
						|| screw.threadedLengthMm>screw.lengthMm || screw.bearingInnerDiameterMm<thread.majorDiameter
						|| screw.bearingOuterDiameterMm<=screw.bearingInnerDiameterMm
						|| screw.flangeDiameterMm<screw.bearingOuterDiameterMm
						|| screw.flangeDiameterMm<screw.headAcrossFlatsMm/Math.cos(Math.PI/6)) throw "Inconsistent anchor hardware: "+id;
					var locking=readLock(Reflect.field(a,"locking"));
					var insertion=positive(a,"insertionMm");
					if (insertion>=screw.lengthMm) throw "Anchor flange must remain outside the insert entry: "+id;
					anchors.push({id:id,holeId:holeId,insertionMm:insertion,screw:screw,locking:locking});
				}
				if (anchors.length==0 || anchors.length>256) throw "Bolted anchorage requires 1..256 anchors";
				anchors.sort((a,b) -> Reflect.compare(a.id,b.id));Bolted(rules,anchors);
			default:throw "Unknown insert anchorage kind";
		};
	}

	public static function record(value:InsertAnchorage):Dynamic return switch value {
		case Undeclared:{kind:"undeclared"};
		case CatalogueRetention:{kind:"catalogue-retention"};
		case Bolted(rules,anchors):{kind:"bolted",rules:rules,anchors:[for (anchor in anchors) {
			id:anchor.id,holeId:anchor.holeId,insertionMm:anchor.insertionMm,screw:anchor.screw,locking:lockingRecord(anchor.locking)}]};
	};

	public static function lockingRecord(value:InsertAnchorLock):Dynamic return switch value {
		case Shoulder:{kind:"shoulder"};
		case RetainingCompound(reference):{kind:"retaining-compound",reference:CastingInput.CastingInputs.textRecord(reference)};
	};
	static function readLock(raw:Dynamic):InsertAnchorLock return switch CastingRecordCodec.string(raw,"kind") {
		case "shoulder":CastingFeatureCodec.fields(raw,["kind"]);Shoulder;
		case "retaining-compound":
			CastingFeatureCodec.fields(raw,["kind","reference"]);
			RetainingCompound(CastingInput.CastingInputs.text(Reflect.field(raw,"reference")));
		default:throw "Unknown anchor locking method";
	};

	public static function references(value:InsertAnchorage,body:InsertSpec.InsertBody,holes:Array<InsertHole>,origin:InsertOrigin,interfaces:Array<InterfaceRequirement>):Void {
		switch value {
			case Undeclared:
			case CatalogueRetention:
				switch origin {case Catalogue(_):default:throw "Catalogue retention requires a catalogue insert";}
				switch body {case CrimpedSocket(_,_):default:throw "Catalogue retention requires declared product retention geometry";}
			case Bolted(rules,anchors):
				var back:Null<Float> = null,stock:Null<Box3> = null;
				switch InsertFinishing.assess(body,interfaces) {
					case Resolved(steel):back=steel.pose.z;stock=InsertFinishing.box(steel);
					case Invalid(_),Unsupported(_):
				}
				for (anchor in anchors) {
					var matches=holes.filter(hole -> hole.id==anchor.holeId);
					if (matches.length!=1) throw "Unknown anchor source hole: "+anchor.holeId;
					var hole=matches[0],normal=F.transformVector(hole.frame,0,0,1);
					if (hole.extent!=HoleExtent.Blind || hole.thread==null) throw "Anchor requires a blind source tap: "+anchor.id;
					if ((back!=null && Math.abs(hole.frame.z-back)>1e-8)
						|| Math.abs(normal.x)>1e-10 || Math.abs(normal.y)>1e-10 || normal.z<1-1e-10)
						throw "Anchor source hole must enter the insert back face: "+anchor.id;
					if (stock!=null) try InsertStages.validateServiceHole(hole,stock)
					catch (error:Dynamic) throw "Anchor "+anchor.id+": "+Std.string(error);
					var internal=hole.thread;
					if (internal==null) throw "Missing anchor thread";
					var external=anchor.screw.thread;
					if (internal.majorDiameter!=external.majorDiameter || internal.pitch!=external.pitch
						|| internal.starts!=external.starts || internal.hand!=external.hand || internal.profile!=external.profile)
						throw "Anchor screw differs from its source thread: "+anchor.id;
					switch origin {
						case Fabricated(manufacturing):
							if (manufacturing.holeRoutes.filter(route -> route.holeId==hole.id
								&& route.route==InsertManufacturing.InsertHoleRoute.BeforeCasting).length!=1)
								throw "Anchor source hole must be drilled before casting: "+anchor.id;
						case Catalogue(_):
					}
				}
				var occupied=[for (anchor in anchors) anchor.holeId];
				for (load in rules.demand.servicePreloads) {
					var service=holes.filter(h -> h.id==load.holeId && h.thread!=null);
					if (service.length!=1 || occupied.indexOf(load.holeId)>=0) throw "Unknown anchor service preload hole: "+load.holeId;
				}
				for (hole in holes) if (hole.thread!=null && occupied.indexOf(hole.id)<0
					&& rules.demand.servicePreloads.filter(load -> load.holeId==hole.id).length!=1)
					throw "Missing anchor service preload: "+hole.id;
		}
	}

	public static function occupiedHoleIds(value:InsertAnchorage):Array<String> return switch value {
		case Bolted(_,anchors):[for (anchor in anchors) anchor.holeId];
		case Undeclared,CatalogueRetention:[];
	};

	public static function bearingFrame(hole:InsertHole,anchor:InsertBoltAnchor):AssemblyFrame
		return F.compose(hole.frame,F.translation(0,0,anchor.insertionMm-anchor.screw.lengthMm));

	public static function demandN(rules:InsertAnchorRules):Float {
		var result=rules.demand.liftingShareN;
		for (load in rules.demand.servicePreloads) result+=load.preloadN;
		return result*rules.demand.factor;
	}
	public static function bearingArea(screw:InsertAnchorScrew):Float
		return Math.PI/4*(screw.bearingOuterDiameterMm*screw.bearingOuterDiameterMm
			-screw.bearingInnerDiameterMm*screw.bearingInnerDiameterMm);

	static function positive(raw:Dynamic,name:String):Float {
		var n=CastingRecordCodec.number(raw,name);if (n<=0) throw "Invalid anchor dimension: "+name;return n;
	}
	static function text(raw:Dynamic,name:String):String {
		var s=CastingRecordCodec.string(raw,name);
		if (StringTools.trim(s).length==0 || materia.assembly.AssemblyCodec.containsNul(s)) throw "Invalid anchor reference: "+name;
		return s;
	}
}

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


typedef InsertAnchorChoice = {
	var id:String;
	var holeId:String;
	var lengthMm:Float;
	var insertionMm:Float;
	var locking:InsertAnchorLock;
}

typedef InsertBackAnchorChoice = {
	var id:String;
	var xMm:Float;
	var yMm:Float;
	var lengthMm:Float;
	var insertionMm:Float;
	var locking:InsertAnchorLock;
}


typedef EmbeddedInsertAnchors = {
	var owner:String;
	var pose:AssemblyFrame;
	var spec:InsertSpec;
}


/** Derived source occupancy, copied from the owning inserts when a foundation
 * is built. It is never a second independent anchor layout or a box substitute. */
class EmbeddedAnchors {
	/** Exact nominal cylinder/external-hex moments of the declared hardware.
	 * exposedOnly removes the shank portion inside the rectangular steel blank;
	 * otherwise returns the metal added to the steel union (thread-envelope
	 * overlap is counted once). Callers qualify containment with source CAD. */
	public static function moments(spec:InsertSpec, exposedOnly:Bool):{volume:Float, center:{x:Float,y:Float,z:Float}} {
		var volume=0.0,mx=0.0,my=0.0,mz=0.0;
		function add(v:Float,frame:AssemblyFrame,z:Float):Void {
			var p=materia.assembly.AssemblyFrames.transformPoint(frame,0,0,z);
			volume+=v;mx+=v*p.x;my+=v*p.y;mz+=v*p.z;
		}
		switch spec.anchorage {
			case Undeclared,CatalogueRetention:
			case Bolted(_,anchors):for (anchor in anchors) {
				var hole=spec.holes.filter(h -> h.id==anchor.holeId)[0];
				var frame=InsertAnchorage.InsertAnchorageCodec.bearingFrame(hole,anchor);
				var screw=anchor.screw;
				var shankLength=exposedOnly ? -frame.z : screw.lengthMm;
				if (shankLength<0 || shankLength>screw.lengthMm) throw "Invalid anchor exposure: "+anchor.id;
				add(Math.PI*Math.pow(screw.thread.majorDiameter/2,2)*shankLength,frame,shankLength/2);
				add(Math.PI*Math.pow(screw.flangeDiameterMm/2,2)*screw.flangeThicknessMm,frame,-screw.flangeThicknessMm/2);
				var hexHeight=screw.headHeightMm-screw.flangeThicknessMm;
				add(Math.sqrt(3)/2*screw.headAcrossFlatsMm*screw.headAcrossFlatsMm*hexHeight,
					frame,-screw.flangeThicknessMm-hexHeight/2);
				if (!exposedOnly) {
					var raw=InsertStages.holes(spec,InsertStages.InsertStage.BeforeCasting).filter(h -> h.id==hole.id)[0];
					var end=hole.frame.z+anchor.insertionMm;
					var events=[0.0,end];
					if (raw.counterbore!=null && raw.counterbore.depth<end) events.push(raw.counterbore.depth);
					if (raw.relief!=null && raw.relief.start<end) events.push(raw.relief.start);
					events.sort(Reflect.compare);
					for (index in 0...events.length-1) {
						var low=events[index],high=events[index+1],mid=(low+high)/2;
						var diameter=raw.diameter;
						if (raw.counterbore!=null && mid<raw.counterbore.depth) diameter=Math.max(diameter,raw.counterbore.diameter);
						if (raw.relief!=null && mid>=raw.relief.start) diameter=Math.max(diameter,raw.relief.diameter);
						var overlap=Math.PI/4*Math.max(0,screw.thread.majorDiameter*screw.thread.majorDiameter-diameter*diameter)*(high-low);
						add(-overlap,raw.frame,mid);
					}
				}
			}
		}
		return {volume:volume,center:volume==0 ? {x:0.0,y:0.0,z:0.0} : {x:mx/volume,y:my/volume,z:mz/volume}};
	}

	public static function snapshot(raw:Dynamic):Array<EmbeddedInsertAnchors> {
		if (!Std.isOfType(raw,Array)) throw "Invalid embedded anchor list";
		var rows:Array<Dynamic> = cast raw,result:Array<EmbeddedInsertAnchors> = [],seen:Map<String,Bool> = [];
		if (rows.length>512) throw "Too many embedded anchor owners";
		for (row in rows) {
			CastingFeatureCodec.fields(row,["owner","pose","spec"]);
			var owner=CastingRecordCodec.string(row,"owner");CastingFeatureCodec.identifier(owner);
			if (seen.exists(owner)) throw "Duplicate embedded anchor owner: "+owner;seen.set(owner,true);
			var spec=InsertSpec.InsertSpecCodec.snapshot(Reflect.field(row,"spec"));
			switch spec.anchorage {case Bolted(_,_):default:throw "Embedded hardware requires bolted source anchorage: "+owner;}
			result.push({owner:owner,pose:CastingRecordCodec.frame(Reflect.field(row,"pose")),spec:spec});
		}
		result.sort((a,b) -> Reflect.compare(a.owner,b.owner));return result;
	}
	public static function records(rows:Array<EmbeddedInsertAnchors>):Array<Dynamic> {
		var raw:Array<Dynamic> = [for (row in rows) {owner:row.owner,pose:row.pose,spec:InsertSpec.InsertSpecCodec.record(row.spec)}];
		return [for (row in snapshot(raw)) {owner:row.owner,pose:row.pose,spec:InsertSpec.InsertSpecCodec.record(row.spec)}];
	}
}

/** Source geometry/rules are independent of pull-out capacity and process evidence. */
class InsertAnchorageChecks {
	public static inline var BEARING_MODEL="projected-bearing-area-times-allowable-stress-v1";
	public static inline var CONE_MODEL="45-degree-projected-cone-area-times-tensile-strength-v1";

	/** Elementary force = stress x area. The 45-degree cone and equal load
	 * sharing are explicit in-house screening assumptions, not EN 1992-4 or a
	 * concrete-anchor certification. Exact circle union removes double-counted
	 * overlap; no empirical edge/group factor is introduced. */
	public static function materialChecks(insert:CastingToolingInput.CastingToolingInsert,
		facts:Array<{anchorId:String,bearingAreaMm2:Float,embedmentMm:Float}>,
		bearingStrengthMPa:CastingInput,tensileStrengthMPa:CastingInput):Array<ToolingValidation> {
		for (value in [bearingStrengthMPa,tensileStrengthMPa])
			switch value {
				case Known(strength):if (!Math.isFinite(strength) || strength<=0) throw "Mix anchor strengths must be positive";
				case Unresolved(reason):if (StringTools.trim(reason).length==0) throw "Missing unresolved mix strength reason";
			}
		var result:Array<ToolingValidation> = [];
		switch insert.anchorage {
			case Undeclared,CatalogueRetention:
			case Bolted(rules,anchors):
				if (facts.length!=anchors.length) throw "Anchor capacity geometry differs from source: "+insert.id;
				var circles:Array<{x:Float,y:Float,radius:Float}> = [];
				for (anchor in anchors) {
					var matches=facts.filter(fact -> fact.anchorId==anchor.id);
					if (matches.length!=1) throw "Missing/duplicate anchor capacity geometry: "+insert.id+"/"+anchor.id;
					var fact=matches[0];
					if (!Math.isFinite(fact.bearingAreaMm2+fact.embedmentMm) || fact.bearingAreaMm2<0 || fact.embedmentMm<0)
						throw "Invalid anchor capacity geometry: "+insert.id+"/"+anchor.id;
					var hole=insert.holes.filter(h -> h.id==anchor.holeId)[0];
					circles.push({x:hole.frame.x,y:hole.frame.y,radius:fact.embedmentMm});
				}
				var projected=projectedConeUnionArea(circles),load=InsertAnchorageCodec.demandN(rules)/anchors.length;
				for (index in 0...anchors.length) {
					var anchor=anchors[index],fact=facts.filter(value -> value.anchorId==anchor.id)[0];
					var bearing=Math.min(InsertAnchorageCodec.bearingArea(anchor.screw),fact.bearingAreaMm2);
					// Each anchor receives no more than its own cone, nor more than an
					// equal share of the union. Summed credit cannot exceed the union.
					var cone=Math.min(Math.PI*fact.embedmentMm*fact.embedmentMm,projected/anchors.length);
					var prefix="tooling.insert.anchorage."+insert.id+"."+anchor.id+".capacity.";
					for (item in [{name:"bearing",area:bearing,strength:bearingStrengthMPa,model:BEARING_MODEL},
						{name:"cone",area:cone,strength:tensileStrengthMPa,model:CONE_MODEL}]) {
						var status=item.area<=0 ? ToolingCheckStatus.Fail : switch item.strength {
							case Unresolved(_):ToolingCheckStatus.Pending;
							case Known(strength):item.area*strength>=load ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail;
						};
						var strengthText=switch item.strength {
							case Unresolved(reason):"unresolved cured-mix strength ("+reason+")";
							case Known(strength):strength+" MPa";
						};
						var message=item.model+": "+item.area+" mm2 x "
							+strengthText
							+"; demand "+load+" N per anchor; required strength "+(item.area>0 ? Std.string(load/item.area) : "undefined")+" MPa.";
						var finding=status==ToolingCheckStatus.Pending
							? ToolingReadiness.pending(prefix+item.name,[insert.id],message) : record(prefix+item.name,insert.id,status,message);
						finding.affectedIds.push(anchor.id);result.push(finding);
					}
				}
		}
		result.sort((a,b) -> Reflect.compare(a.rule,b.rule));
		return result;
	}

	/** Exact projected circle-union area from exposed circular arcs (Green's
	 * theorem). This handles multiple/triple overlaps without pairwise area
	 * subtraction, sampling or an interaction coefficient. */
	public static function projectedConeUnionArea(input:Array<{x:Float,y:Float,radius:Float}>):Float {
		if (input.length==0) return 0;
		var circles=input.copy();
		circles.sort((a,b) -> a.x!=b.x ? Reflect.compare(a.x,b.x)
			: a.y!=b.y ? Reflect.compare(a.y,b.y) : Reflect.compare(a.radius,b.radius));
		var originX=circles[0].x,originY=circles[0].y,area=0.0,tau=2*Math.PI;
		for (index in 0...circles.length) {
			var c=circles[index],intervals:Array<{a:Float,b:Float}> = [],covered=false;
			if (!Math.isFinite(c.x+c.y+c.radius) || c.radius<0) throw "Invalid projected anchor cone";
			if (c.radius==0) continue;
			for (other in 0...circles.length) if (index!=other) {
				var q=circles[other],dx=q.x-c.x,dy=q.y-c.y,d=Math.sqrt(dx*dx+dy*dy);
				if (d+c.radius<=q.radius && (c.radius!=q.radius || d!=0 || other<index)) {covered=true;break;}
				if (d==0 || d>=c.radius+q.radius || d+q.radius<=c.radius) continue;
				var half=Math.acos(Math.max(-1,Math.min(1,(c.radius*c.radius+d*d-q.radius*q.radius)/(2*c.radius*d))));
				var middle=Math.atan2(dy,dx),start=middle-half,finish=middle+half;
				while (start<0) {start+=tau;finish+=tau;}
				while (start>=tau) {start-=tau;finish-=tau;}
				intervals.push({a:start,b:Math.min(tau,finish)});
				if (finish>tau) intervals.push({a:0,b:finish-tau});
			}
			if (covered) continue;
			intervals.sort((a,b) -> Reflect.compare(a.a,b.a));
			function integral(from:Float,to:Float):Float
				return 0.5*(c.radius*c.radius*(to-from)+c.radius*(c.x-originX)*(Math.sin(to)-Math.sin(from))
					-c.radius*(c.y-originY)*(Math.cos(to)-Math.cos(from)));
			var end=0.0;
			for (interval in intervals) {
				if (interval.a>end) area+=integral(end,interval.a);
				end=Math.max(end,interval.b);
			}
			if (end<tau) area+=integral(end,tau);
		}
		return Math.max(0,area);
	}

	public static function check(insert:CastingToolingInput.CastingToolingInsert):Array<ToolingValidation> {
		var result:Array<ToolingValidation> = [],prefix="tooling.insert.anchorage."+insert.id;
		var holes=insert.holes;
		switch insert.anchorage {
			case Undeclared:
				result.push(ToolingReadiness.pending(prefix+".source",[insert.id],"Declare source anchorage: normally blind back-face taps with bolted bearing anchors."));
			case CatalogueRetention:
				result.push(ToolingReadiness.pending(prefix+".capacity",[insert.id],"Product retention geometry is declared; compound-specific pull-out/shear evidence is required."));
			case Bolted(rules,anchors):
				var admitted=0.0;
				for (anchor in anchors) admitted+=anchor.screw.assemblyPreloadN;
				var demand=InsertAnchorageCodec.demandN(rules);
				result.push(record(prefix+".demand",insert.id,admitted>=demand ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail,
					"Design demand "+demand+" N; aggregate class-8.8 assembly-preload admission "+admitted
					+" N. This steel/hole-policy screen does not establish mix bearing or breakout capacity."));
				for (anchor in anchors) {
					var source=holes.filter(hole -> hole.id==anchor.holeId);
					if (source.length!=1) throw "Missing pre-cast anchor source hole: "+anchor.id;
					var hole=source[0],bearing=InsertAnchorageCodec.bearingFrame(hole,anchor),key=prefix+"."+anchor.id;
					for (finding in materia.casting.ThreadedFastenerFit.checkIntervals(hole,hole.frame,
						{shankLength:anchor.screw.lengthMm,threadedLength:anchor.screw.threadedLengthMm,bearingFrame:bearing},anchor.id))
						result.push(record(key+"."+finding.code,insert.id,finding.passed ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail,finding.message));
					var area=InsertAnchorageCodec.bearingArea(anchor.screw);
					result.push(record(key+".bearing-area",insert.id,area>=rules.minimumBearingAreaMm2 ? ToolingCheckStatus.Unsupported : ToolingCheckStatus.Fail,
						"Guaranteed Type F flange bearing annulus "+area+" mm2; source minimum "+rules.minimumBearingAreaMm2
						+" mm2. Certify the actual face bearing on resin, excluding steel/tooling contact, before this rule can pass."));
					if (anchor.locking==InsertAnchorLock.Shoulder)
						result.push(record(key+".locking",insert.id,ToolingCheckStatus.Unsupported,
							"Declare and realize the actual shoulder/stop geometry; no flange-to-insert contact or thread-runout stop is inferred."));
					else {
						var detail=switch anchor.locking {
							case RetainingCompound(reference):switch reference {case Known(value):"Process reference: "+value;case Unresolved(reason):reason;};
							case Shoulder:throw "Unexpected shoulder route";
						};
						result.push(ToolingReadiness.pending(key+".locking",[insert.id],detail+
							". Qualify the retaining process against the actual source anchor; a reference alone is not cure/torque evidence."));
					}
				}
				for (a in 0...anchors.length) for (b in a+1...anchors.length) {
					var first=anchors[a],second=anchors[b];
					var p=holes.filter(h -> h.id==first.holeId)[0].frame,q=holes.filter(h -> h.id==second.holeId)[0].frame;
					var dx=p.x-q.x,dy=p.y-q.y;
					var gap=Math.sqrt(dx*dx+dy*dy)-(first.screw.flangeDiameterMm+second.screw.flangeDiameterMm)/2;
					result.push(record(prefix+".spacing."+first.id+"."+second.id,insert.id,gap>=rules.minimumSpacingMm-1e-7 ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail,
						"Clear flange-envelope spacing "+gap+" mm; source minimum "+rules.minimumSpacingMm+" mm."));
				}
				result.push(record(prefix+".casting-geometry",insert.id,ToolingCheckStatus.Unsupported,
					"Native anchor occupancy is realized; qualify casting embedment/cover, tooling interference and exact non-box fill before fabrication. No box approximation is credited."));
				result.push(ToolingReadiness.pending(prefix+".capacity",[insert.id],"Bolted anchor geometry never establishes compound-specific pull-out/shear capacity; record physical coupon evidence."));
		}
		result.sort((a,b) -> Reflect.compare(a.rule,b.rule));return result;
	}
	static function record(rule:String,id:String,status:ToolingCheckStatus,message:String):ToolingValidation
		return {rule:rule,affectedIds:[id],status:status,severity:status==ToolingCheckStatus.Pass ? ToolingSeverity.Info : ToolingSeverity.Error,
			message:message,evidence:[]};
}

/** Resolved catalogue facts enter from the native source adapter. Selection and
 * candidate order depend only on these neutral records and the declared rules. */
typedef InsertAnchorStockChoice = {
	var size:String;
	var diameter:Float;
	var flangeDiameter:Float;
	var bearingInnerDiameter:Float;
	var bearingOuterDiameter:Float;
	var hole:InsertHole;
	var assemblyPreloadN:Float;
	var lengths:Array<Float>;
}

typedef InsertAnchorCandidate = {
	var size:String;
	var diameter:Float;
	var thread:InsertThread;
	var rules:InsertAnchorRules;
	var points:Array<InsertBackAnchorChoice>;
	var lengthMm:Float;
	var landMm:Float;
	var key:String;
}

class InsertAnchorSelection {
	public static function validateRule(rule:InsertAnchorPatternRule):Void {
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


	public static function requiredHoleLigament(a:InsertHole,b:InsertHole):Float
		return Math.max(Math.max(a.limits.minWall,b.limits.minWall),Math.max(a.limits.minClearance,b.limits.minClearance));

	public static function select(spec:InsertSpec,rule:InsertAnchorPatternRule,demand:InsertAnchorDemand,
		choices:Array<InsertAnchorStockChoice>,failures:Array<String>,admit:InsertAnchorCandidate->Bool):InsertAnchorCandidate {
		validateRule(rule);
		var steel=switch InsertFinishing.assess(spec.body,spec.interfaces) {
			case Resolved(result):result;
			case Invalid(reason),Unsupported(reason):throw reason;
		};
		var stock=InsertFinishing.box(steel);
		if (stock==null) throw "Back-face anchor rule requires rectangular steel";
		var total=demand.liftingShareN;
		for (load in demand.servicePreloads) total+=load.preloadN;
		total*=demand.factor;
		var candidates:Array<InsertAnchorCandidate>=[];
		for (choice in choices) {
			var size=choice.size,diameter=choice.diameter,nominal=choice,hole=choice.hole,lengths=choice.lengths;
			var insertion=InsertHole.InsertHoleCodec.threadLimits(hole).minEngagement;
			var holeThread=hole.thread;
			if (holeThread==null) throw "Missing resolved source anchor thread";
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
				var limit=choice.assemblyPreloadN;
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
							var points:Array<InsertBackAnchorChoice> = [];
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
									if (Math.sqrt(dx*dx+dy*dy)<radius+diameter/2+requiredHoleLigament(hole,other)) clear=false;
								}
								if (clear) points.push({id:"back-anchor-"+ix+"-"+iy,xMm:x,yMm:y,lengthMm:lengths[0],insertionMm:insertion,
									locking:InsertAnchorLock.RetainingCompound(materia.casting.CastingInput.CastingTextInput.Unresolved("Anchor retaining compound and cure/torque evidence"))});
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
								for (length in lengths) candidates.push({size:size,diameter:diameter,thread:holeThread,rules:rules,
									points:points,lengthMm:length,landMm:landX+landY,key:key});
							}
						}
					}
				}
			}
		}
		// Preserve demand/count, diameter, length, land and coordinate ordering.
		// Compare concrete numeric/string fields directly; Reflect boxes each
		// comparison in the large rail-seat candidate sets.
		candidates.sort((a,b) -> {
			if (a.points.length!=b.points.length) return a.points.length<b.points.length ? -1 : 1;
			if (a.diameter!=b.diameter) return a.diameter<b.diameter ? -1 : 1;
			if (a.lengthMm!=b.lengthMm) return a.lengthMm<b.lengthMm ? -1 : 1;
			if (a.landMm!=b.landMm) return a.landMm<b.landMm ? -1 : 1;
			return a.key==b.key ? 0 : a.key<b.key ? -1 : 1;
		});
		for (candidate in candidates) if (admit(candidate)) return candidate;
		throw "No ISO 4162 pattern meets design demand "+total+" N, hole policy, footprint and casting cover; "
			+candidates.length+" demand-admissible patterns checked. "+failures.join("; ");
	}
}

typedef AnchorPoint = {var x:Float;var y:Float;var z:Float;}
typedef AnchorEdge = {var a:AnchorPoint;var b:AnchorPoint;}
typedef AnchorFace = {var normal:AnchorPoint;var edges:Array<AnchorEdge>;}

/** Exact primitive screen against finite planar boundary faces. Edges are the
 * actual trimmed face edges, including inner loops. No mesh or pose sampling.
 * Native CAD confirms the one selected complete pattern. */
class InsertAnchorScreen {
	static inline var EPS=1e-8;
	static function add(a:AnchorPoint,b:AnchorPoint):AnchorPoint return {x:a.x+b.x,y:a.y+b.y,z:a.z+b.z};
	static function sub(a:AnchorPoint,b:AnchorPoint):AnchorPoint return {x:a.x-b.x,y:a.y-b.y,z:a.z-b.z};
	static function scale(a:AnchorPoint,s:Float):AnchorPoint return {x:a.x*s,y:a.y*s,z:a.z*s};
	static function dot(a:AnchorPoint,b:AnchorPoint):Float return a.x*b.x+a.y*b.y+a.z*b.z;
	static function norm(a:AnchorPoint):Float return Math.sqrt(dot(a,a));
	static function clamp(t:Float,lo:Float,hi:Float):Float return Math.max(lo,Math.min(hi,t));
	static function vertex(frame:AssemblyFrame,x:Float,y:Float,z:Float):AnchorPoint return F.transformPoint(frame,x,y,z);

	static function pointSegment(p:AnchorPoint,a:AnchorPoint,b:AnchorPoint):Float {
		var v=sub(b,a),q=dot(v,v),t=q==0 ? 0 : clamp(dot(sub(p,a),v)/q,0,1);
		return norm(sub(p,add(a,scale(v,t))));
	}

	/** Even/odd planar containment handles non-convex trims and inner loops. */
	static function inFace(p:AnchorPoint,face:AnchorFace):Bool {
		var n=face.normal,drop=Math.abs(n.x)>Math.abs(n.y) ? 0 : 1;
		if (Math.abs(n.z)>(drop==0 ? Math.abs(n.x) : Math.abs(n.y))) drop=2;
		function u(q:AnchorPoint):Float return drop==0 ? q.y : q.x;
		function v(q:AnchorPoint):Float return drop==2 ? q.y : q.z;
		var inside=false,px=u(p),py=v(p);
		for (edge in face.edges) {
			if (pointSegment(p,edge.a,edge.b)<=EPS) return true;
			var ax=u(edge.a),ay=v(edge.a),bx=u(edge.b),by=v(edge.b);
			if ((ay>py)!=(by>py) && px<(bx-ax)*(py-ay)/(by-ay)+ax) inside=!inside;
		}
		return inside;
	}

	static function pointFace(p:AnchorPoint,face:AnchorFace):Float {
		var n=face.normal,d=dot(sub(p,face.edges[0].a),n),projected=sub(p,scale(n,d));
		if (inFace(projected,face)) return Math.abs(d);
		var best=Math.POSITIVE_INFINITY;
		for (edge in face.edges) best=Math.min(best,pointSegment(p,edge.a,edge.b));
		return best;
	}

	public static function inside(p:AnchorPoint,faces:Array<AnchorFace>):Bool {
		var direction:AnchorPoint={x:1,y:0.3713906763541037,z:0.198912367379658},hits:Array<Float>=[];
		for (face in faces) {
			var distance=dot(sub(face.edges[0].a,p),face.normal),denominator=dot(direction,face.normal);
			if (Math.abs(distance)<=EPS && inFace(p,face)) return true;
			if (Math.abs(denominator)<=1e-12) continue;
			var t=distance/denominator;
			if (t>EPS && inFace(add(p,scale(direction,t)),face)) hits.push(t);
		}
		hits.sort(Reflect.compare);
		var count=0,last=Math.NEGATIVE_INFINITY;
		for (hit in hits) if (hit-last>EPS) {count++;last=hit;}
		return count%2==1;
	}

	public static function axialDepth(p:AnchorPoint,axis:AnchorPoint,faces:Array<AnchorFace>):Float {
		var best=Math.POSITIVE_INFINITY;
		for (face in faces) {
			var denominator=dot(axis,face.normal);
			if (Math.abs(denominator)<=1e-12) continue;
			var t=dot(sub(face.edges[0].a,p),face.normal)/denominator;
			if (t>EPS && t<best && inFace(add(p,scale(axis,t)),face)) best=t;
		}
		return best;
	}

	static function cylinderPoint(p:AnchorPoint,origin:AnchorPoint,axis:AnchorPoint,r:Float,h:Float):Float {
		var q=sub(p,origin),z=dot(q,axis),radial=norm(sub(q,scale(axis,z)));
		var dz=z-clamp(z,0,h),dr=Math.max(0,radial-r);
		return Math.sqrt(dz*dz+dr*dr);
	}

	/** Squared distance to a capped cylinder is convex along a line. Solve its
	 * monotone analytic derivative; this is root finding, not sampled geometry. */
	static function cylinderEdge(edge:AnchorEdge,origin:AnchorPoint,axis:AnchorPoint,r:Float,h:Float):Float {
		var v=sub(edge.b,edge.a),vz=dot(v,axis),vr=sub(v,scale(axis,vz));
		function derivative(t:Float):Float {
			var q=sub(add(edge.a,scale(v,t)),origin),z=dot(q,axis),radial=sub(q,scale(axis,z)),rho=norm(radial);
			return (z-clamp(z,0,h))*vz+(rho>r && rho>0 ? (rho-r)*dot(radial,vr)/rho : 0);
		}
		var t=0.0;
		if (derivative(0)>=0) t=0;
		else if (derivative(1)<=0) t=1;
		else {
			var lo=0.0,hi=1.0;
			for (_ in 0...64) {var middle=(lo+hi)/2;if (derivative(middle)>0) hi=middle;else lo=middle;}
			t=(lo+hi)/2;
		}
		return cylinderPoint(add(edge.a,scale(v,t)),origin,axis,r,h);
	}

	static function cylinderSupport(origin:AnchorPoint,axis:AnchorPoint,r:Float,h:Float,n:AnchorPoint):AnchorPoint {
		var along=dot(n,axis),radial=sub(n,scale(axis,along)),length=norm(radial);
		return add(add(origin,scale(axis,along>EPS ? h : along < -EPS ? 0 : h/2)),
			length>EPS ? scale(radial,r/length) : {x:0,y:0,z:0});
	}

	public static function cylinderFace(origin:AnchorPoint,axis:AnchorPoint,r:Float,h:Float,face:AnchorFace):Float {
		var n=face.normal,d=dot(n,face.edges[0].a),low=cylinderSupport(origin,axis,r,h,scale(n,-1)),
			high=cylinderSupport(origin,axis,r,h,n),a=dot(n,low)-d,b=dot(n,high)-d;
		var best=Math.POSITIVE_INFINITY;
		if (a<=0 && b>=0) {
			var witness=Math.abs(b-a)>1e-14 ? add(low,scale(sub(high,low),-a/(b-a))) : low;
			if (inFace(witness,face)) return 0;
		} else {
			var support=a>0 ? low : high,signed=a>0 ? a : b;
			if (inFace(sub(support,scale(n,signed)),face)) return Math.abs(signed);
		}
		for (edge in face.edges) {
			best=Math.min(best,cylinderEdge(edge,origin,axis,r,h));
			if (best<=EPS) return 0;
		}
		return best;
	}

	static function segmentSegment(first:AnchorEdge,second:AnchorEdge):Float {
		var u=sub(first.b,first.a),v=sub(second.b,second.a),w=sub(first.a,second.a),
			a=dot(u,u),b=dot(u,v),c=dot(v,v),d=dot(u,w),e=dot(v,w),denominator=a*c-b*b;
		var best=Math.min(Math.min(pointSegment(first.a,second.a,second.b),pointSegment(first.b,second.a,second.b)),
			Math.min(pointSegment(second.a,first.a,first.b),pointSegment(second.b,first.a,first.b)));
		if (denominator>1e-20) {
			var s=(b*e-c*d)/denominator,t=(a*e-b*d)/denominator;
			if (s>=0 && s<=1 && t>=0 && t<=1) best=Math.min(best,norm(sub(add(first.a,scale(u,s)),add(second.a,scale(v,t)))));
		}
		return best;
	}

	static function segmentHitsFace(edge:AnchorEdge,face:AnchorFace):Bool {
		var direction=sub(edge.b,edge.a),denominator=dot(direction,face.normal);
		if (Math.abs(denominator)<1e-14) return false;
		var t=dot(sub(face.edges[0].a,edge.a),face.normal)/denominator;
		return t>=0 && t<=1 && inFace(add(edge.a,scale(direction,t)),face);
	}

	static function hexPoints(frame:AssemblyFrame,acrossFlats:Float,z:Float):Array<AnchorPoint> {
		var radius=acrossFlats/(2*Math.cos(Math.PI/6));
		return [for (i in 0...6) vertex(frame,radius*Math.cos(Math.PI/6+2*Math.PI*i/6),
			radius*Math.sin(Math.PI/6+2*Math.PI*i/6),z)];
	}

	static function hexFaces(frame:AssemblyFrame,acrossFlats:Float,z0:Float,z1:Float):Array<AnchorFace> {
		var low=hexPoints(frame,acrossFlats,z0),high=hexPoints(frame,acrossFlats,z1);
		function face(points:Array<AnchorPoint>,normal:AnchorPoint):AnchorFace
			return {normal:normal,edges:[for (i in 0...points.length) {a:points[i],b:points[(i+1)%points.length]}]};
		var result=[face(low,F.transformVector(frame,0,0,-1)),face(high,F.transformVector(frame,0,0,1))];
		for (i in 0...6) {
			var j=(i+1)%6,angle=2*Math.PI*(i+1)/6;
			result.push(face([low[i],low[j],high[j],high[i]],F.transformVector(frame,Math.cos(angle),Math.sin(angle),0)));
		}
		return result;
	}

	public static function hexFace(frame:AssemblyFrame,acrossFlats:Float,z0:Float,z1:Float,target:AnchorFace):Float {
		var hull=hexFaces(frame,acrossFlats,z0,z1),best=Math.POSITIVE_INFINITY;
		for (face in hull) for (edge in face.edges) {
			if (segmentHitsFace(edge,target)) return 0;
			best=Math.min(best,pointFace(edge.a,target));
			for (other in target.edges) best=Math.min(best,segmentSegment(edge,other));
		}
		for (edge in target.edges) {
			var p=edge.a;
			if (hull.filter(face -> dot(sub(p,face.edges[0].a),face.normal)>EPS).length==0) return 0;
			for (face in hull) {
				if (segmentHitsFace(edge,face)) return 0;
				best=Math.min(best,pointFace(p,face));
			}
		}
		return best;
	}

	static function near(origin:AnchorPoint,axis:AnchorPoint,r:Float,h:Float,face:AnchorFace,cover:Float):Bool {
		var end=add(origin,scale(axis,h)),low={x:Math.min(origin.x,end.x)-r,y:Math.min(origin.y,end.y)-r,z:Math.min(origin.z,end.z)-r},
			high={x:Math.max(origin.x,end.x)+r,y:Math.max(origin.y,end.y)+r,z:Math.max(origin.z,end.z)+r};
		var minimum={x:Math.POSITIVE_INFINITY,y:Math.POSITIVE_INFINITY,z:Math.POSITIVE_INFINITY},
			maximum={x:Math.NEGATIVE_INFINITY,y:Math.NEGATIVE_INFINITY,z:Math.NEGATIVE_INFINITY};
		for (edge in face.edges) for (p in [edge.a,edge.b]) {
			minimum.x=Math.min(minimum.x,p.x);minimum.y=Math.min(minimum.y,p.y);minimum.z=Math.min(minimum.z,p.z);
			maximum.x=Math.max(maximum.x,p.x);maximum.y=Math.max(maximum.y,p.y);maximum.z=Math.max(maximum.z,p.z);
		}
		var dx=Math.max(0,Math.max(minimum.x-high.x,low.x-maximum.x)),
			dy=Math.max(0,Math.max(minimum.y-high.y,low.y-maximum.y)),dz=Math.max(0,Math.max(minimum.z-high.z,low.z-maximum.z));
		return dx*dx+dy*dy+dz*dz<cover*cover;
	}

	public static function accepts(point:InsertBackAnchorChoice,screw:InsertAnchorScrew,stockZ:Float,
		pose:AssemblyFrame,faces:Array<AnchorFace>,cover:Float,minimumEmbedment:Float):Bool {
		var bearing=F.compose(pose,F.translation(point.xMm,point.yMm,stockZ+point.insertionMm-point.lengthMm)),
			origin=vertex(bearing,0,0,0),axis=F.transformVector(bearing,0,0,1);
		if (axialDepth(origin,axis,faces)+1e-7<minimumEmbedment) return false;
		// The source insert exterior is a box beginning at local Z=0. Its steel
		// occupancy removes the engaged part of the shank from exposed hardware.
		var shankHeight=point.lengthMm-point.insertionMm-stockZ;
		if (shankHeight<0) throw "Anchor screen has an inverted exposed shank";
		var flangeOrigin=vertex(bearing,0,0,-screw.flangeThicknessMm),hexOrigin=vertex(bearing,0,0,-screw.headHeightMm);
		for (cylinder in [{origin:origin,r:screw.thread.majorDiameter/2,h:shankHeight},
			{origin:flangeOrigin,r:screw.flangeDiameterMm/2,h:screw.flangeThicknessMm}]) {
			if (!inside(add(cylinder.origin,scale(axis,cylinder.h/2)),faces)) return false;
			for (face in faces) if (near(cylinder.origin,axis,cylinder.r,cylinder.h,face,cover)
				&& cylinderFace(cylinder.origin,axis,cylinder.r,cylinder.h,face)+1e-7<cover) return false;
		}
		var hexHeight=screw.headHeightMm-screw.flangeThicknessMm;
		if (!inside(add(hexOrigin,scale(axis,hexHeight/2)),faces)) return false;
		for (face in faces) if (near(hexOrigin,axis,screw.headAcrossFlatsMm/Math.sqrt(3),hexHeight,face,cover)
			&& hexFace(bearing,screw.headAcrossFlatsMm,-screw.headHeightMm,-screw.flangeThicknessMm,face)+1e-7<cover) return false;
		return true;
	}
}
