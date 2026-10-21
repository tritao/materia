package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.casting.Box3;
import materia.casting.BoxAlgebra;
import materia.casting.CastingBounds;
import materia.casting.FixtureConstraints;
import materia.casting.FixtureConstraints.FixtureConstraint;
import materia.casting.FixtureConstraints.FixtureConstraintReport;
import materia.casting.FixtureConstraints.FixtureVector;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingToolingInput.CastingToolingInsert;
import materia.tooling.ToolingValidation;

typedef DatumLocationPlan = {
	var contacts:Array<FixtureConstraint>;
	var origin:FixtureVector;
	var referenceLengthMm:Float;
	var report:FixtureConstraintReport;
	var unsupportedReasons:Array<String>;
}

typedef DatumLocationInput = {
	var mountingFace:materia.assembly.AssemblyRecord.AssemblyFrame;
	var datum:MoldVolume;
	var fixings:Array<InsertFixtureFixing>;
}

/** Checks insert-to-datum location. Bed pins locate the datum, never the insert.
 * Rectangular subsets outside conservative hole/cut bounds certify contact witnesses;
 * they do not approximate a drilled solid or infer a press fit from a clearance screw.
 */
class DatumFixtureLocation {
	static inline var EPS = 1e-7;
	static inline var RANK_TOLERANCE = 1e-9;

	public static function plan(source:CastingToolingInsert,sourceToMold:materia.assembly.AssemblyRecord.AssemblyFrame,
			fixture:DatumLocationInput):DatumLocationPlan {
		return contactPlan(source,sourceToMold,fixture.mountingFace,fixture.datum.id,
			fixture.datum.pieces,fixture.datum.cuts,fixture.datum.profiles.length>0,fixture.fixings);
	}

	/** Direct bed support uses a rectangle proved inside the finished panel outline,
	 * excluding conservative bounds of every cut that reaches its contact plane. */
	public static function onBed(source:CastingToolingInsert,sourceToMold:materia.assembly.AssemblyRecord.AssemblyFrame,
			face:materia.assembly.AssemblyRecord.AssemblyFrame,bed:materia.tooling.panel.PanelAssemblyPlan.PanelPlacement,
			cavity:MoldPlan.MoldBox,fixings:Array<InsertFixtureFixing>):DatumLocationPlan {
		var rotation=F.toRotationMatrix(bed.pose);
		for (i in 0...9) if (Math.abs(rotation[i]-(i==0 || i==4 || i==8 ? 1 : 0))>EPS)
			throw "Box fixture bed contact requires an XY panel: "+bed.panel.id;
		var outlines=bed.panel.operations.filter(op -> op.kind==materia.tooling.panel.PanelPlan.PanelOperationKind.OutsideProfile);
		if (outlines.length!=1 || !materia.tooling.panel.PanelGeometry.containsRect(outlines[0].contour,
			cavity.x-bed.pose.x,cavity.y-bed.pose.y,cavity.width,cavity.depth))
			throw "Box fixture contact rectangle is outside its bed: "+bed.panel.id;
		var thickness=bed.panel.stock.measuredThickness;
		var pieces:Array<MoldPlan.MoldBox> = [{x:cavity.x,y:cavity.y,z:bed.pose.z,
			width:cavity.width,depth:cavity.depth,height:thickness}];
		var cuts:Array<MoldVolumeCut> = [];
		for (operation in bed.panel.operations) if (operation.kind!=materia.tooling.panel.PanelPlan.PanelOperationKind.OutsideProfile) {
			var z=operation.side==materia.tooling.ToolingEnums.MachiningSide.PositiveZ ? thickness-operation.depth : 0.0;
			cuts.push({operation:operation,pose:F.compose(bed.pose,F.translation(0,0,z))});
		}
		return contactPlan(source,sourceToMold,face,bed.panel.id,pieces,cuts,false,fixings);
	}

	static function contactPlan(source:CastingToolingInsert,sourceToMold:materia.assembly.AssemblyRecord.AssemblyFrame,
			face:materia.assembly.AssemblyRecord.AssemblyFrame,datumId:String,pieces:Array<MoldPlan.MoldBox>,
			cuts:Array<MoldVolumeCut>,hasProfiles:Bool,fixings:Array<InsertFixtureFixing>):DatumLocationPlan {
		var pose=F.compose(sourceToMold,source.pose);
		var dimensions=materia.casting.InsertSpec.InsertBodyCodec.box(source.body,source.id);
		var bounds=CastingBounds.box({solid:materia.casting.CastingFeature.CastingGeometrySolid.Insert(source.body),pose:pose});
		var origin:FixtureVector={x:face.x,y:face.y,z:face.z};
		var scale=Math.max(dimensions.width,Math.max(dimensions.depth,dimensions.height));
		var contacts:Array<FixtureConstraint> = [],reasons:Array<String> = [],patches:Array<Box3> = [];
		var normal=F.transformVector(face,0,0,1);
		if (Math.abs(normal.x)>EPS || Math.abs(normal.y)>EPS || Math.abs(normal.z+1)>EPS)
			reasons.push("Only a downward box mounting face has a supported contact model");
		if (hasProfiles)
			reasons.push("Profile-derived datum contact needs an exact finite-face contact model");
		if (reasons.length==0) {
			var footprint:Box3={x:bounds.x,y:bounds.y,z:0,width:bounds.width,depth:bounds.depth,height:1};
			for (piece in pieces) if (Math.abs(piece.z+piece.height-face.z)<=EPS) {
				var patch=BoxAlgebra.intersection(footprint,{x:piece.x,y:piece.y,z:0,width:piece.width,depth:piece.depth,height:1});
				if (patch!=null) patches.push(patch);
			}
			var excluded:Array<Box3> = [];
			for (hole in InsertFixturePlanner.holes(source)) {
				var entry=F.compose(pose,hole.frame),axis=F.transformVector(entry,0,0,1);
				var end=F.transformPoint(entry,0,0,hole.depth);
				var diameter=hole.thread==null ? hole.diameter : hole.thread.majorDiameter;
				if (hole.counterbore!=null) diameter=Math.max(diameter,hole.counterbore.diameter);
				if (hole.relief!=null) diameter=Math.max(diameter,hole.relief.diameter);
				var radius=diameter/2;
				var rx=radius*Math.sqrt(Math.max(0,1-axis.x*axis.x));
				var ry=radius*Math.sqrt(Math.max(0,1-axis.y*axis.y));
				var rz=radius*Math.sqrt(Math.max(0,1-axis.z*axis.z));
				if (face.z>=Math.min(entry.z,end.z)-rz-EPS && face.z<=Math.max(entry.z,end.z)+rz+EPS)
					excluded.push({x:Math.min(entry.x,end.x)-rx-EPS,y:Math.min(entry.y,end.y)-ry-EPS,z:0,
						width:Math.abs(entry.x-end.x)+2*(rx+EPS),depth:Math.abs(entry.y-end.y)+2*(ry+EPS),height:1});
			}
			for (cut in cuts) {
				var b=materia.tooling.panel.PanelGeometry.bounds(cut.operation.contour);
				var xs:Array<Float> = [],ys:Array<Float> = [],zs:Array<Float> = [];
				for (x in [b.x,b.x+b.width]) for (y in [b.y,b.y+b.height]) for (z in [0.0,cut.operation.depth]) {
					var point=F.transformPoint(cut.pose,x,y,z);
					xs.push(point.x);ys.push(point.y);zs.push(point.z);
				}
				xs.sort(Reflect.compare);ys.sort(Reflect.compare);zs.sort(Reflect.compare);
				if (face.z>=zs[0]-EPS && face.z<=zs[7]+EPS)
					excluded.push({x:xs[0]-EPS,y:ys[0]-EPS,z:0,width:xs[7]-xs[0]+2*EPS,depth:ys[7]-ys[0]+2*EPS,height:1});
			}
			for (boxes in [patches,excluded]) boxes.sort((a,b) ->
				Reflect.compare(materia.tooling.ToolingCodec.canonical(a),materia.tooling.ToolingCodec.canonical(b)));
			patches=BoxAlgebra.subtract(patches,excluded);
			patches.sort((a,b) -> {
				var area=Reflect.compare(b.width*b.depth,a.width*a.depth);
				return area!=0 ? area : Reflect.compare(materia.tooling.ToolingCodec.canonical(a),materia.tooling.ToolingCodec.canonical(b));
			});
			if (patches.length==0) reasons.push("No finite contact patch certified outside the source holes and datum cuts");
			else {
				var patch=patches[0];
				// Three non-collinear interior witnesses span the continuous plane's row space.
				for (index in 0...3) contacts.push({
					id:materia.tooling.ToolingIds.derive("contact-",source.id,"-"+index),sourceId:datumId,role:Location,
					point:{x:patch.x+patch.width*(index==1 ? 0.75 : 0.25),
						y:patch.y+patch.depth*(index==2 ? 0.75 : 0.25),z:face.z},normal:{x:0,y:0,z:1}
				});
			}
		}
		for (fixing in fixings) {
			var axis=F.transformVector(fixing.entry,0,0,1);
			contacts.push({id:fixing.hardwareId,sourceId:fixing.hardwareId,role:Preload,
				point:{x:fixing.entry.x,y:fixing.entry.y,z:fixing.entry.z},normal:{x:-axis.x,y:-axis.y,z:-axis.z}});
		}
		contacts.sort((a,b) -> Reflect.compare(a.id,b.id));
		return {contacts:contacts,origin:origin,referenceLengthMm:scale,
			report:FixtureConstraints.analyze(contacts,origin,scale,RANK_TOLERANCE),unsupportedReasons:reasons};
	}

	public static function validation(insertId:String,location:DatumLocationPlan):ToolingValidation {
		var rule="tooling.fixture.location."+insertId;
		if (location.unsupportedReasons.length>0)
			return {rule:rule,affectedIds:[insertId],status:materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported,
				severity:materia.tooling.ToolingDiagnostic.ToolingSeverity.Error,
				message:location.unsupportedReasons.join("; "),evidence:[]};
		var directions:Array<String> = [];
		for (motion in location.report.freeMotions) for (coordinate in motion.affectedCoordinates)
			if (directions.indexOf(coordinate)<0) directions.push(coordinate);
		return ToolingReadiness.pending(rule,[insertId],"Insert location rank "+location.report.rank+"/6; free directions: "+directions.join(", ")
			+". Datum-to-bed pins and clearance fixing screws provide no insert-to-datum lateral location.");
	}
}
