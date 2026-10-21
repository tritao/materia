package machinekit.tooling;

import cadkit.modeling.Part;
import machinekit.assembly.PosedParts;
import machinekit.structural.InsertAnchorHardware;
import machinekit.structural.CastingCavityCad;
import machinekit.structural.InsertAnchorHardware.InsertAnchorComponentKind;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.casting.CastingToolingInput;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingDiagnostic.ToolingSeverity;

/** Native checks of source hardware. Strength, locking evidence and tooling
 * interference remain separate; source shape bounds never substitute for solids. */
class InsertAnchorGeometry {
	public static function review(input:CastingToolingInput):Array<ToolingValidation> {
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
					var spec:materia.casting.InsertSpec={body:insert.body,holes:insert.holes,origin:insert.origin,
						anchorage:insert.anchorage,interfaces:insert.interfaces};
					var exterior=own(PosedParts.moved(own(machinekit.structural.InsertBodies.part(insert.body)),insert.pose));
					var hardware=InsertAnchorHardware.parts(spec);
					for (anchor in anchors) {
						var metal:Array<Part> = [],washer:Null<Part> = null;
						var washerPose=F.identity();
						for (component in hardware) if (component.anchorId==anchor.id) {
							var pose=F.compose(insert.pose,component.pose);
							var placed=own(PosedParts.moved(own(component.part.geometry()),pose));metal.push(placed);
							if (component.kind==InsertAnchorComponentKind.Washer) {washer=placed;washerPose=pose;}
						}
						var exposed=own(own(Part.fuseAll(metal)).subtractAll([exterior]));
						var outside=own(exposed.subtractAll([bulk]));
						var contained=outside.volume()<=1e-6,cover=contained ? boundaryDistance(exposed,bulk) : 0.0;
						result.push(check(insert.id,anchor.id,"cover",contained && cover+1e-7>=rules.minimumCoverMm,
							contained ? "Minimum exposed-casting boundary cover "+cover+" mm; requires "+rules.minimumCoverMm+" mm."
							: "Anchor hardware outside its steel insert extends into a source cavity or outside the casting."));
						if (washer==null) throw "Missing source anchor washer: "+insert.id+"/"+anchor.id;
						var placedWasher:Part=washer;
						var point=F.transformPoint(washerPose,0,0,anchor.washer.thicknessMm);
						var normal=F.transformVector(washerPose,0,0,1);
						var origin=cadkit.Geometry.vec3(point.x,point.y,point.z);
						var direction=cadkit.Geometry.vec3(normal.x,normal.y,normal.z);
						var face=ownShape(placedWasher.shape.section(origin,direction));
						var resinFace=ownShape(resin.shape.section(origin,direction));
						var contact=ownShape(face.planarCommon(resinFace)),area=contact.area();
						result.push(check(insert.id,anchor.id,"bearing-area",area+1e-7>=rules.minimumBearingAreaMm2,
							"Actual washer face bearing on nominal source resin "+area+" mm2; requires "+rules.minimumBearingAreaMm2
							+" mm2. Tooling interference and physical capacity are not qualified by this source-only check."));
					}
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
}
