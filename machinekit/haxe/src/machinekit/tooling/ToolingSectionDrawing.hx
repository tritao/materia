package machinekit.tooling;

import cadkit.drawing.Drawing2D;
import cadkit.drawing.DrawingEntity;
import cadkit.drawing.DrawingLayer;
import cadkit.sheetmetal.Point2;
import cadkit.modeling.Scope;
import cadkit.modeling.Part;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.CastingProfiles;
import materia.casting.CastingProfileGeometry;
import materia.tooling.casting.CastingMoldPlan.MoldVolume;
import materia.casting.CastingSection;

typedef ToolingSectionSheet = {
	var drawing:Drawing2D;
	/** Unshifted finished contours in each declared plane, also usable by CAM. */
	var sections:Array<CastingSection>;
}

/** Nominal section sheets from actual finished tooling, without stock-bound
 * silhouettes or cutter compensation. Existing CadKit exporters own text output.
 */
class ToolingSectionDrawing {
	public static function sheet(volume:MoldVolume,frame:AssemblyFrame,levels:Array<Float>):ToolingSectionSheet {
		if (levels==null || levels.length==0 || levels.length>64) throw "Invalid tooling drawing section count: "+volume.id;
		frame=materia.casting.CastingRecordCodec.frame(frame);
		var scope=new Scope();
		try {
			var part=scope.own(ToolingVolume.geometryOf(volume)),result=draw(part,volume.id,frame,levels);
			scope.close();return result;
		} catch (error:Dynamic) {scope.close();throw error;}
	}

	/** Reference sections along a declared removal direction. Actual extrema,
	 * not bounding-box faces, determine the end sections.
	 */
	public static function byDirection(volume:MoldVolume,direction:{x:Float,y:Float,z:Float}):ToolingSectionSheet {
		var frame=CastingSectionContours.normalFrame(direction,volume.id);
		var scope=new Scope();
		try {
			var part=scope.own(ToolingVolume.geometryOf(volume));
			var local=scope.own(machinekit.assembly.PosedParts.moved(part,F.inverse(frame)));
			var geometry=new CastingFillGeometry(volume.id,local),levels:Array<Float>;
			try levels=geometry.heightInterval() catch (error:Dynamic) {geometry.close();throw error;}
			geometry.close();
			if (levels.length<2) throw "Tooling drawing has no positive depth: "+volume.id;
			var low=levels[0],high=levels[levels.length-1];
			var result=draw(part,volume.id,frame,[low,(low+high)/2,high]);
			scope.close();return result;
		} catch (error:Dynamic) {scope.close();throw error;}
	}

	static function draw(part:Part,id:String,frame:AssemblyFrame,levels:Array<Float>):ToolingSectionSheet {
		var sections:Array<CastingSection> =[];
		var entities:Array<DrawingEntity> =[],offset=0.0;
		for (level in levels) {
			if (!Math.isFinite(level)) throw "Invalid tooling drawing level: "+id;
			var plane=F.compose(frame,F.translation(0,0,level));
			var contours=CastingSectionContours.section(part,plane,id);
			sections.push({plane:plane,contours:contours});
			var minX=0.0,minY=0.0,maxX=0.0,maxY=0.0,first=true;
			for (contour in contours) {
				var bounds=CastingProfileGeometry.bounds(contour.contour);
				if (first) {minX=bounds.x;minY=bounds.y;maxX=bounds.x+bounds.width;maxY=bounds.y+bounds.height;first=false;}
				minX=Math.min(minX,bounds.x);minY=Math.min(minY,bounds.y);
				maxX=Math.max(maxX,bounds.x+bounds.width);maxY=Math.max(maxY,bounds.y+bounds.height);
			}
			for (contour in contours) {
				var layer=DrawingLayer.Process(contour.depth%2==0 ? "OUTSIDE_PROFILE" : "INSIDE_PROFILE");
				for (segment in CastingProfiles.placed(contour.contour,0,offset-minX,-minY))
					entities.push(Curve(layer,PanelCad.segment(segment)));
			}
			entities.push(Label(Text,new Point2(offset,maxY-minY+8),2.5,id+" / section "+level+" mm"));
			if (contours.length==0) entities.push(Label(Text,new Point2(offset,0),2.5,"No material section"));
			offset+=Math.max(40,maxX-minX)+25;
		}
		entities.push(Label(Text,new Point2(0,-10),2.5,"Finished sections / mm / no cutter compensation"));
		return {drawing:new Drawing2D(entities),sections:sections};
	}
}
