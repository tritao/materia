package materia.tooling.casting;

import materia.casting.InsertAnchorage;
import materia.casting.InsertAnchorage.InsertAnchorageCodec as C;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;

/** Source geometry/rules are independent of pull-out capacity and process evidence. */
class InsertAnchorageChecks {
	public static function check(insert:CastingToolingInput.CastingToolingInsert):Array<ToolingValidation> {
		var result:Array<ToolingValidation> = [],prefix="tooling.insert.anchorage."+insert.id;
		var holes=insert.holes;
		switch insert.anchorage {
			case Undeclared:
				result.push(ToolingReadiness.pending(prefix+".source",[insert.id],"Declare source anchorage: normally blind back-face taps with bolted bearing anchors."));
			case CatalogueRetention:
				result.push(ToolingReadiness.pending(prefix+".capacity",[insert.id],"Product retention geometry is declared; compound-specific pull-out/shear evidence is required."));
			case Bolted(rules,anchors):
				for (anchor in anchors) {
					var source=holes.filter(hole -> hole.id==anchor.holeId);
					if (source.length!=1) throw "Missing pre-cast anchor source hole: "+anchor.id;
					var hole=source[0],bearing=C.bearingFrame(hole,anchor),key=prefix+"."+anchor.id;
					for (finding in materia.casting.ThreadedFastenerFit.checkIntervals(hole,hole.frame,
						{shankLength:anchor.screw.lengthMm,threadedLength:anchor.screw.threadedLengthMm,bearingFrame:bearing},anchor.id))
						result.push(record(key+"."+finding.code,insert.id,finding.passed ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail,finding.message));
					var washer=anchor.washer,area=Math.PI*(washer.outerDiameterMm*washer.outerDiameterMm-washer.innerDiameterMm*washer.innerDiameterMm)/4;
					result.push(record(key+".bearing-area",insert.id,area>=rules.minimumBearingAreaMm2 ? ToolingCheckStatus.Unsupported : ToolingCheckStatus.Fail,
						"Potential washer annular area "+area+" mm2; source minimum "+rules.minimumBearingAreaMm2
						+" mm2. Certify the actual face bearing on resin, excluding steel/tooling contact, before this rule can pass."));
					if (anchor.locking==InsertAnchorLock.Shoulder)
						result.push(record(key+".locking",insert.id,ToolingCheckStatus.Unsupported,
							"Declare and realize the actual shoulder/stop geometry; no washer-to-insert contact or thread-runout stop is inferred."));
					else result.push(ToolingReadiness.pending(key+".locking",[insert.id],
						"Qualify the retaining-compound process against the actual source anchor; a process reference alone is not cure evidence."));
				}
				for (a in 0...anchors.length) for (b in a+1...anchors.length) {
					var first=anchors[a],second=anchors[b];
					var p=holes.filter(h -> h.id==first.holeId)[0].frame,q=holes.filter(h -> h.id==second.holeId)[0].frame;
					var dx=p.x-q.x,dy=p.y-q.y;
					var gap=Math.sqrt(dx*dx+dy*dy)-(first.washer.outerDiameterMm+second.washer.outerDiameterMm)/2;
					result.push(record(prefix+".spacing."+first.id+"."+second.id,insert.id,gap>=rules.minimumSpacingMm ? ToolingCheckStatus.Pass : ToolingCheckStatus.Fail,
						"Clear washer-envelope spacing "+gap+" mm; source minimum "+rules.minimumSpacingMm+" mm."));
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
