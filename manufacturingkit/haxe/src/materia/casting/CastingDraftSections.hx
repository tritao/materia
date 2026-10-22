package materia.casting;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingFeature;
import materia.casting.CastingProfile.CastingSegment;

typedef DraftedCastingSections = {
	var featureId:String;
	/** Local Z points toward withdrawal; the opening lies at local Z = depth. */
	var pose:AssemblyFrame;
	var depth:Float;
	var opening:Array<CastingSegment>;
	var inner:Array<CastingSegment>;
}

/** Authored dimensions belong to the declared opening, never the narrow end.
 * This neutral construction is shared by CAD, former drawings and machining.
 */
class CastingDraftSections {
	static inline var EPS=1e-7;
	public static function derive(body:Array<CastingPrism>,feature:CastingFeature):DraftedCastingSections {
		try return build(body,feature) catch (error:Dynamic)
			throw "Casting feature "+feature.id+": "+Std.string(error);
	}
	public static function sections(featureId:String,profile:Array<CastingSegment>,pose:AssemblyFrame,
			depth:Float,degrees:Float):DraftedCastingSections {
		CastingFeatureCodec.identifier(featureId);
		if (!Math.isFinite(depth) || depth<=0 || !Math.isFinite(degrees) || degrees<0 || degrees>=45)
			throw "Invalid draft section: "+featureId;
		var opening=CastingProfile.CastingProfileCodec.snapshot(profile);
		try return {featureId:featureId,pose:CastingRecordCodec.frame(pose),depth:depth,opening:opening,
			inner:CastingProfileOffsets.inward(opening,depth*Math.tan(degrees*Math.PI/180))}
		catch (error:Dynamic) throw "Casting feature "+featureId+": "+Std.string(error);
	}

	/** Capability only: invalid reached depths and collapsing offsets remain
	 * design errors, while these unrealized axes/openings stay Unsupported.
	 */
	public static function unsupportedReason(body:Array<CastingPrism>,feature:CastingFeature):Null<String> {
		var axis=feature.draft.axis,local=F.transformVector(F.inverse(feature.geometry.pose),axis.x,axis.y,axis.z);
		var components=[local.x,local.y,local.z],dimension=-1;
		for (index in 0...3) if (Math.abs(Math.abs(components[index])-1)<EPS) dimension=index;
		if (dimension<0) return "Unsupported draft axis oblique to the feature extrusion";
		switch feature.geometry.solid {
			case Cut(CastingSolid.Box(_,_,_)):
			default:if (dimension!=2) return "Unsupported draft axis transverse to a profile prism";
		}
		var faces=CastingCavities.boundary(body),planes:Array<Float> =[];
		for (opening in feature.openings) if (opening.kind==CastingOpeningKind.BodyFace) {
			var normal=CastingCavities.bodyFaceNormal(body,opening.target);
			if (normal.x*axis.x+normal.y*axis.y+normal.z*axis.z<1-EPS) continue;
			var point=faces.filter(face -> face.id==opening.target)[0].points[0];
			planes.push(point.x*axis.x+point.y*axis.y+point.z*axis.z);
		}
		if (planes.length==0) return "Unsupported draft without a declared planar withdrawal opening";
		for (plane in planes) if (Math.abs(plane-planes[0])>EPS) return "Unsupported draft across multiple withdrawal planes";
		return null;
	}

	static function build(body:Array<CastingPrism>,feature:CastingFeature):DraftedCastingSections {
		var axis=feature.draft.axis,local=F.transformVector(F.inverse(feature.geometry.pose),axis.x,axis.y,axis.z);
		var components=[local.x,local.y,local.z],dimension=-1;
		for (index in 0...3) if (Math.abs(Math.abs(components[index])-1)<EPS) dimension=index;
		if (dimension<0) throw "Unsupported draft axis oblique to the feature extrusion";
		var sign=components[dimension]>0 ? 1.0 : -1.0;
		var u=[0.0,0.0,0.0],v=[0.0,0.0,0.0],w=[0.0,0.0,0.0];
		u[(dimension+1)%3]=1;v[(dimension+2)%3]=sign;w[dimension]=sign;
		var center=[0.0,0.0,0.0],length=0.0,profile:Array<CastingSegment>;
		switch feature.geometry.solid {
			case Cut(CastingSolid.Box(width,depth,height)):
				var sizes=[width,depth,height];center[2]=height/2;length=sizes[dimension];
				var transverseWidth=sizes[(dimension+1)%3],transverseDepth=sizes[(dimension+2)%3];
				profile=feature.cornerRadius==0 ? CastingProfiles.rectangle(-transverseWidth/2,-transverseDepth/2,transverseWidth,transverseDepth)
					: CastingProfiles.roundedRectangle(-transverseWidth/2,-transverseDepth/2,transverseWidth,transverseDepth,feature.cornerRadius);
			default:
				if (dimension!=2) throw "Unsupported draft axis transverse to a profile prism";
				length=CastingFeature.CastingGeometrySolids.height(feature.geometry.solid);center[2]=length/2;
				profile=CastingFeatureContours.contour(feature);
				if (sign<0) {
					var reversed:Array<CastingSegment> =[];
					for (index in 0...profile.length) {
						var segment=profile[profile.length-1-index];
						reversed.push({kind:segment.kind,start:{x:segment.end.x,y:-segment.end.y},
							end:{x:segment.start.x,y:-segment.start.y},sweep:segment.sweep,
							center:segment.center==null ? null : {x:segment.center.x,y:-segment.center.y}});
					}
					profile=reversed;
				}
		}
		var localPose=F.fromRotationMatrix(center[0]-w[0]*length/2,center[1]-w[1]*length/2,center[2]-w[2]*length/2,
			[u[0],v[0],w[0],u[1],v[1],w[1],u[2],v[2],w[2]]);
		var pose=F.compose(feature.geometry.pose,localPose),planes:Array<Float> =[];
		for (opening in feature.openings) if (opening.kind==CastingOpeningKind.BodyFace) {
			var normal=CastingCavities.bodyFaceNormal(body,opening.target);
			if (normal.x*axis.x+normal.y*axis.y+normal.z*axis.z<1-EPS) continue;
			var face=CastingCavities.boundary(body).filter(face -> face.id==opening.target)[0];
			var point=face.points[0];
			planes.push((point.x-pose.x)*axis.x+(point.y-pose.y)*axis.y+(point.z-pose.z)*axis.z);
		}
		if (planes.length==0) throw "Unsupported draft without a declared planar withdrawal opening";
		var depth=planes[0];
		for (plane in planes) if (Math.abs(plane-depth)>EPS) throw "Unsupported draft across multiple withdrawal planes";
		if (depth<=EPS || depth>length+EPS) throw "Draft cut does not reach its withdrawal opening";
		return sections(feature.id,profile,pose,depth,feature.draft.degrees);
	}
}
