package materia.tooling.casting;

import materia.casting.InsertAnchorage;
import materia.casting.InsertAnchorage.InsertAnchorageCodec as C;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;

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
		bearingStrengthMPa:Null<Float>,tensileStrengthMPa:Null<Float>):Array<ToolingValidation> {
		for (value in [bearingStrengthMPa,tensileStrengthMPa])
			if (value!=null && (!Math.isFinite(value) || value<=0)) throw "Mix anchor strengths must be positive or unresolved";
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
				var projected=projectedConeUnionArea(circles),load=C.demandN(rules)/anchors.length;
				for (index in 0...anchors.length) {
					var anchor=anchors[index],fact=facts.filter(value -> value.anchorId==anchor.id)[0];
					var bearing=Math.min(C.bearingArea(anchor.screw),fact.bearingAreaMm2);
					// Each anchor receives no more than its own cone, nor more than an
					// equal share of the union. Summed credit cannot exceed the union.
					var cone=Math.min(Math.PI*fact.embedmentMm*fact.embedmentMm,projected/anchors.length);
					var prefix="tooling.insert.anchorage."+insert.id+"."+anchor.id+".capacity.";
					for (item in [{name:"bearing",area:bearing,strength:bearingStrengthMPa,model:BEARING_MODEL},
						{name:"cone",area:cone,strength:tensileStrengthMPa,model:CONE_MODEL}]) {
						var strength=item.strength;
						var status=item.area<=0 ? ToolingCheckStatus.Fail : strength==null ? ToolingCheckStatus.Pending
							: item.area*strength>=load ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail;
						var message=item.model+": "+item.area+" mm2 x "
							+(item.strength==null ? "unresolved cured-mix strength" : item.strength+" MPa")
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
				var demand=C.demandN(rules);
				result.push(record(prefix+".demand",insert.id,admitted>=demand ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail,
					"Design demand "+demand+" N; aggregate class-8.8 assembly-preload admission "+admitted
					+" N. This steel/hole-policy screen does not establish mix bearing or breakout capacity."));
				for (anchor in anchors) {
					var source=holes.filter(hole -> hole.id==anchor.holeId);
					if (source.length!=1) throw "Missing pre-cast anchor source hole: "+anchor.id;
					var hole=source[0],bearing=C.bearingFrame(hole,anchor),key=prefix+"."+anchor.id;
					for (finding in materia.casting.ThreadedFastenerFit.checkIntervals(hole,hole.frame,
						{shankLength:anchor.screw.lengthMm,threadedLength:anchor.screw.threadedLengthMm,bearingFrame:bearing},anchor.id))
						result.push(record(key+"."+finding.code,insert.id,finding.passed ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail,finding.message));
					var area=C.bearingArea(anchor.screw);
					result.push(record(key+".bearing-area",insert.id,area>=rules.minimumBearingAreaMm2 ? ToolingCheckStatus.Unsupported : ToolingCheckStatus.Fail,
						"Guaranteed Type F flange bearing annulus "+area+" mm2; source minimum "+rules.minimumBearingAreaMm2
						+" mm2. Certify the actual face bearing on resin, excluding steel/tooling contact, before this rule can pass."));
					if (anchor.locking==InsertAnchorLock.Shoulder)
						result.push(record(key+".locking",insert.id,ToolingCheckStatus.Unsupported,
							"Declare and realize the actual shoulder/stop geometry; no flange-to-insert contact or thread-runout stop is inferred."));
					else result.push(ToolingReadiness.pending(key+".locking",[insert.id],
						"Qualify the retaining-compound process against the actual source anchor; a process reference alone is not cure evidence."));
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
