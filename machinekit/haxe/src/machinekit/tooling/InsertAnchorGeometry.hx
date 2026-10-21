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
	/** Actual source hardware against every realized tooling member. The insert's
	 * own threaded engagement and its own screw/washer contact are intentional. */
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
						var embedment=axialEmbedment(point,normal,bulk);
						result.push(check(insert.id,anchor.id,"embedment",embedment+1e-7>=rules.minimumEmbedmentMm,
							"Axial depth from washer bearing plane to the exposed casting boundary toward the loaded insert face "
							+embedment+" mm; requires "+rules.minimumEmbedmentMm+" mm."));
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

	/** Exact line/face intersections along the source pull direction. Steel
	 * recesses are included in bulk; an internal steel/resin interface never
	 * substitutes for the exposed load-side casting boundary. */
	static function axialEmbedment(point:materia.assembly.AssemblyDefinition.AssemblyVector,
		direction:materia.assembly.AssemblyDefinition.AssemblyVector,bulk:Part):Float {
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
