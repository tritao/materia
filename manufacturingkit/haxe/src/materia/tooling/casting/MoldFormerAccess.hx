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
		var copy=materia.casting.CastingFeatureCodec.snapshot([feature])[0];
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
			volume:MoldVolume,exclusions:Array<CastingRegion>):Array<CastingRegion> {
		if (Math.abs(direction.z)>1-EPS) return [];
		try return build(plan,feature,direction,volume,exclusions) catch (error:Dynamic)
			throw "Former access "+feature.id+": "+Std.string(error);
	}

	static function build(plan:CastingMoldPlan,feature:CastingFeature,direction:FormerWithdrawal,
			volume:MoldVolume,exclusions:Array<CastingRegion>):Array<CastingRegion> {
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
		var operation=PanelJoints.operation(materia.tooling.ToolingIds.derive("access-",feature.id),InsideProfile,
			contour,r.panel.measuredThickness,r.routerDiameter,MachiningSide.PositiveZ,true);
		wall.panel.operations.push(operation);
		wall.panel.contactOpenings.push(operation.id);
		wall.panel=PanelCodec.snapshot(wall.panel);
		volume.profiles.push({panelId:wall.panel.id,operationId:operation.id,contour:operation.contour,
			pose:wall.pose,depth:operation.depth,subtract:exclusions.copy()});
		var bounds=CastingProfileGeometry.bounds(contour),margin=r.hardware.formerFlangeMargin;
		var flange=CastingRegion.Placed({solid:Cut(materia.casting.CastingSolid.Box(
			bounds.width+2*margin,bounds.height+2*margin,r.hardware.formerFlangeThickness)),
			pose:F.compose(wall.pose,F.translation(bounds.x+bounds.width/2,bounds.y+bounds.height/2,operation.depth))});
		volume.regions.push(CastingRegion.Difference(flange,exclusions.copy()));
		plan.openings.push({panelId:wall.panel.id,operationId:operation.id,kind:ToolingOpening,
			sealingIds:[volume.id],sealSpec:r.hardware.sealMaterial});
		return [flange,MoldRegions.profile(volume.profiles[volume.profiles.length-1])];
	}
}
