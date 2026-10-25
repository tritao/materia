package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingFeature;
import materia.casting.CastingRegion;
import materia.casting.CastingDraftSections;
import materia.casting.CastingProfileGeometry;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.ToolingDocument;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.tooling.ToolingEnums.MachiningSide;

typedef FormerWithdrawal = {var x:Float;var y:Float;var z:Float;}

/** Source-section access through a planar panel wall. Source dimensions remain
 * finished geometry; cutter compensation belongs to CamKit.
 */
class MoldFormerAccess {
	static inline var EPS=1e-7;

	static function sectionFeature(d:ToolingDocument,feature:CastingFeature,direction:FormerWithdrawal):CastingFeature {
		var copy=materia.casting.CastingFeatureCodec.copy([feature])[0];
		if (copy.draft.degrees==0)
			copy.draft.axis=F.transformVector(F.inverse(d.setup.sourceToMold),direction.x,direction.y,direction.z);
		return copy;
	}

	public static function supported(d:ToolingDocument,feature:CastingFeature,direction:FormerWithdrawal):Bool {
		if (Math.abs(direction.z)>1-EPS) return true;
		if (Math.abs(direction.z)>EPS || (Math.abs(direction.x)<1-EPS && Math.abs(direction.y)<1-EPS)) return false;
		var source=sectionFeature(d,feature,direction);
		// Without draft the authored radius belongs to the original XY prism.
		// A transverse pull needs a true section, never a reoriented fillet.
		if (feature.draft.degrees==0 && feature.cornerRadius>0) {
			var local=F.transformVector(F.inverse(feature.geometry.pose),source.draft.axis.x,source.draft.axis.y,source.draft.axis.z);
			if (Math.abs(local.z)<1-EPS) return false;
		}
		if (CastingDraftSections.unsupportedReason(d.input.envelope,source)!=null) return false;
		var normal=F.transformVector(d.setup.sourceToMold,source.draft.axis.x,source.draft.axis.y,source.draft.axis.z);
		if (normal.x*direction.x+normal.y*direction.y+normal.z*direction.z<1-EPS) return false;
		return feature.openings.filter(opening -> opening.kind==CastingOpeningKind.BodyFace).length==1;
	}

	public static function add(plan:CastingMoldPlan,feature:CastingFeature,direction:FormerWithdrawal,
			volume:MoldVolume,exclusions:Array<CastingRegion>,registeredEPS:Bool=false):Array<CastingRegion> {
		// Rigid bed-facing tools leave after the bed is removed. Registered
		// EPS uses registeredBedAccess explicitly, before its material conversion.
		if (Math.abs(direction.z)>1-EPS) return [];
		try return build(plan,feature,direction,volume,exclusions,registeredEPS) catch (error:Dynamic)
			throw "Former access "+feature.id+": "+Std.string(error);
	}

	/** Bed access lives entirely outside the source envelope. A rectangular
	 * removable neck can therefore include front-face backing without rounding
	 * or enlarging the casting cavity. It uses the existing relieved profile.
	 */
	public static function registeredBedAccess(plan:CastingMoldPlan,feature:CastingFeature,volume:MoldVolume,
			exclusions:Array<CastingRegion>):Array<CastingRegion> {
		var d=plan.document,r=d.recipe,bed=plan.panels.members.filter(member -> member.panel.id=="bed")[0];
		var bounds=MoldRegions.exactBounds(MoldRegions.feature(d,feature.id),feature.id);
		var inner=bed.pose.z+r.panel.measuredThickness;
		if (bounds.z<inner-EPS) throw "Source opening is below the mold bed: "+feature.id;
		var margin=r.minimumLigament,cx=bounds.x+bounds.width/2,cy=bounds.y+bounds.depth/2;
		var width=Math.max(bounds.width+2*margin,r.routerDiameter+2*margin);
		var height=Math.max(bounds.depth+2*margin,r.routerDiameter+2*margin);
		var contour=PanelContours.relievedRectangle(cx-bed.pose.x,cy-bed.pose.y,width,height,r.routerDiameter/2);
		// The through-bed profile may enter a wall's dry bottom rebate.
		// Above the bed it stops at the actual inner wall planes instead.
		var neck:Null<CastingRegion>=bounds.z>inner+EPS ? Intersection([
			Placed({solid:Prism(contour,bounds.z-inner),pose:F.compose(bed.pose,F.translation(0,0,r.panel.measuredThickness))}),
			MoldRegions.box(plan.cavity)]) : null;
		if (neck!=null && MoldRegions.volume(Intersection([neck,MoldRegions.envelope(d)]),feature.id)>EPS)
			throw "Bed access would change source casting geometry: "+feature.id;
		var operation=PanelJoints.operation(materia.tooling.ToolingIds.derive("access-",feature.id),InsideProfile,
			contour,r.panel.measuredThickness,r.routerDiameter,MachiningSide.PositiveZ,true,r.routerDiameter);
		bed.panel.operations.push(operation);bed.panel.contactOpenings.push(operation.id);
		bed.panel=PanelCodec.copy(bed.panel);
		var profile:MoldVolumeProfile={panelId:bed.panel.id,operationId:operation.id,contour:operation.contour,
			pose:bed.pose,depth:operation.depth,subtract:exclusions.copy()};
		volume.profiles.push(profile);
		var flange:CastingRegion=Placed({solid:Cut(materia.casting.CastingSolid.Box(
			width+2*r.hardware.formerFlangeMargin,height+2*r.hardware.formerFlangeMargin,r.hardware.formerFlangeThickness)),
			pose:F.translation(cx,cy,bed.pose.z-r.hardware.formerFlangeThickness)});
		var extensions:Array<CastingRegion>=[flange];
		if (neck!=null) extensions.unshift(neck);
		for (region in extensions) volume.regions.push(Difference(region,exclusions.copy()));
		plan.openings.push({panelId:bed.panel.id,operationId:operation.id,kind:ToolingOpening,
			sealingIds:[volume.id],protection:null,sealSpec:r.hardware.sealMaterial});
		return extensions.concat([MoldRegions.profile(profile)]);
	}

	static function build(plan:CastingMoldPlan,feature:CastingFeature,direction:FormerWithdrawal,
			volume:MoldVolume,exclusions:Array<CastingRegion>,registeredEPS:Bool):Array<CastingRegion> {
		var d=plan.document,r=d.recipe;
		if (!supported(d,feature,direction)) throw "Unsupported source opening or withdrawal";
		var section=CastingDraftSections.derive(d.input.envelope,sectionFeature(d,feature,direction));
		var opening=F.compose(d.setup.sourceToMold,F.compose(section.pose,F.translation(0,0,section.depth)));
		var wallId=Math.abs(direction.x)>0.9 ? (direction.x>0 ? "east" : "west") : (direction.y>0 ? "north" : "south");
		var wall=plan.panels.members.filter(member -> member.panel.id==wallId)[0];
		var local=F.compose(F.inverse(wall.pose),opening);
		var normal=F.transformVector(local,0,0,1),u=F.transformVector(local,1,0,0);
		if (Math.abs(local.z)>EPS || normal.z<1-EPS || Math.abs(u.z)>EPS)
			throw "Source withdrawal section is not on its panel contact plane";
		var contour=PanelContours.placed(section.opening,Math.atan2(u.y,u.x),local.x,local.y);
		if (registeredEPS) {
			// A dry registration neck occupies the wall thickness outside the
			// source envelope. Its relief belongs to the matched EPS neck and
			// panel opening, never the finished casting cavity.
			var sourceBounds=CastingProfileGeometry.bounds(contour),margin=r.minimumLigament;
			contour=PanelContours.relievedRectangle(sourceBounds.x+sourceBounds.width/2,
				sourceBounds.y+sourceBounds.height/2,sourceBounds.width+2*margin,
				sourceBounds.height+2*margin,r.routerDiameter/2);
		}
		var operation=PanelJoints.operation(materia.tooling.ToolingIds.derive("access-",feature.id),InsideProfile,
			contour,r.panel.measuredThickness,r.routerDiameter,MachiningSide.PositiveZ,true,
			registeredEPS ? r.routerDiameter : null);
		wall.panel.operations.push(operation);
		wall.panel.contactOpenings.push(operation.id);
		wall.panel=PanelCodec.copy(wall.panel);
		volume.profiles.push({panelId:wall.panel.id,operationId:operation.id,contour:operation.contour,
			pose:wall.pose,depth:operation.depth,subtract:exclusions.copy()});
		var bounds=CastingProfileGeometry.bounds(contour),margin=r.hardware.formerFlangeMargin;
		var flange=CastingRegion.Placed({solid:Cut(materia.casting.CastingSolid.Box(
			bounds.width+2*margin,bounds.height+2*margin,r.hardware.formerFlangeThickness)),
			pose:F.compose(wall.pose,F.translation(bounds.x+bounds.width/2,bounds.y+bounds.height/2,operation.depth))});
		volume.regions.push(CastingRegion.Difference(flange,exclusions.copy()));
		plan.openings.push({panelId:wall.panel.id,operationId:operation.id,kind:ToolingOpening,
			sealingIds:[volume.id],protection:null,sealSpec:r.hardware.sealMaterial});
		return [flange,MoldRegions.profile(volume.profiles[volume.profiles.length-1])];
	}
}
