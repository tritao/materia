package machinekit.design;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.assembly.PosedParts;
import cadkit.modeling.Part;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import machinekit.design.DesignRequirement.DesignEnvelope;

/** +Z enters the material from the drilling face. Drill depth includes the point envelope. */
typedef BlindThreadInterface = {
	var spec:cadkit.threads.ThreadSpec; var tapDrill:Float; var usableDepth:Float; var cylindricalDepth:Float; var point:Float;
	var stock:Float; var minEngagement:Float; var minTipGap:Float; var minBack:Float;
}
typedef CounterboreInterface = {var radius:Float; var depth:Float; var bearingDepth:Float;}
typedef HoleInterface = {
	@:optional var relief:materia.casting.InsertHole.InsertBoreRelief;
	@:optional var insertClearanceDiameter:Float;
	@:optional var insertLimits:materia.casting.InsertHole.InsertHoleLimits;
	var id:String; var frame:AssemblyFrame; var radius:Float; var depth:Float;
	@:optional var through:Bool;
	var minWall:Float; var minClearance:Float;
	var thread:Null<BlindThreadInterface>; var counterbore:Null<CounterboreInterface>;
	var approach:Null<{radius:Float,length:Float}>;
}
enum DesignDatumKind { AssemblyDatum; LocatingRegister; LubricationPort; }
typedef DesignDatum = {var id:String; var frame:AssemblyFrame; var kind:DesignDatumKind;}

/** Optional component facet. Recipes regenerate these facts; solids are never
 * stored in metadata. The same hole descriptions can generate machining tools.
 */
class DesignInterfaces implements ComponentFacet {
	public final holes:Array<HoleInterface>;
	public final datums:Array<DesignDatum>;
	public final material:Array<DesignEnvelope>;
	final precision:Array<materia.casting.InterfaceRequirement>;
	public function new(holes:Array<HoleInterface>,datums:Array<DesignDatum>,material:Array<DesignEnvelope>,
			?precision:Array<materia.casting.InterfaceRequirement>) {
		this.holes=holes; this.datums=datums; this.material=material;
		var declarations:Array<materia.casting.InterfaceRequirement> = precision == null ? [] : precision;
		this.precision=materia.casting.InterfaceRequirement.InterfaceRequirements.snapshot(declarations);
	}
	public function precisionRequirements():Array<materia.casting.InterfaceRequirement>
		return materia.casting.InterfaceRequirement.InterfaceRequirements.snapshot(precision);
	public static function of(component:MachineComponent):Null<DesignInterfaces> {
		for(facet in component.facets()) if(Std.isOfType(facet,DesignInterfaces)) return cast facet;
		return null;
	}
	public function describe():String return 'Manufacturing interfaces: ${holes.length} holes, ${datums.length} datums, ${material.length} material regions';
	public function check(component:MachineComponent):Void {
		if(of(component)!=null && of(component)!=this) throw "Duplicate design interface facet";
		var ids:Map<String,Bool> = [];
		for(h in holes) {
			name(h.id,ids); validFrame(h.frame);
			positive(h.radius); positive(h.depth); positive(h.minWall); positive(h.minClearance);
			if(h.counterbore!=null) { var c=h.counterbore; positive(c.bearingDepth);positive(c.radius);positive(c.depth); if(c.radius<=h.radius || c.depth<=0 || c.depth>=h.depth) throw "Invalid counterbore"; }
			if(h.thread!=null) {
				var t=h.thread;
				for(v in [t.tapDrill,t.usableDepth,t.cylindricalDepth,t.stock,t.minEngagement,t.minTipGap,t.minBack]) positive(v);
				if(!Math.isFinite(t.point)||t.point<0||t.spec==null || !t.spec.internal || t.tapDrill<=0 || t.tapDrill>=t.spec.majorDiameter || Math.abs(h.depth-t.cylindricalDepth-t.point)>1e-6) throw "Inconsistent blind drilling depth";
			}
			if (h.relief != null) {
				var r = h.relief;
				positive(r.diameter); positive(r.start);
				if (h.thread == null || h.through == true || h.counterbore != null
					|| r.diameter <= h.thread.spec.majorDiameter || r.start >= h.depth
					|| Math.abs(r.start - h.thread.usableDepth) > 1e-8)
					throw "Invalid interface thread relief";
			}
			if(h.approach!=null) {positive(h.approach.radius);positive(h.approach.length);}
		}
		for(d in datums) {name(d.id,ids);validFrame(d.frame);}
		for(e in material) {name(e.id,ids);validFrame(e.frame); var shape=DesignGeometry.envelope(e);shape.close();}
		for (requirement in precision) {
			name(requirement.id,ids);
			validFrame(materia.casting.InterfaceRequirement.InterfaceRequirements.geometryFrame(requirement.geometry));
		}
	}
	static function name(id:String,ids:Map<String,Bool>):Void {if(id==null || id.length==0 || ids.exists(id)) throw "Duplicate/empty design interface";ids.set(id,true);}
	static function positive(v:Float):Void {if(!Math.isFinite(v) || v<=0) throw "Design interface needs finite positive dimensions";}
	static function validFrame(f:AssemblyFrame):Void {DesignGeometry.validateFrame(f);}
	public function frame(id:String):AssemblyFrame {
		for(h in holes) if(h.id==id) return h.frame;
		for(d in datums) if(d.id==id) return d.frame;
		for (requirement in precision) if (requirement.id == id)
			return materia.casting.InterfaceRequirement.InterfaceRequirements.geometryFrame(requirement.geometry);
		throw 'Unknown design interface "$id"';
	}
	public function hole(id:String):HoleInterface {for(h in holes) if(h.id==id) return h; throw 'Unknown hole "$id"';}
	public function drillingTools():Array<Part> {
		var tools:Array<Part> = [];
		try for(h in holes) {
			var local=Part.cylinderSpan(h.radius,-1,h.depth+(h.through==true?1:0));
			tools.push(PosedParts.moved(local,h.frame));local.close();
			if (h.relief != null) {
				var relief = Part.cylinderSpan(h.relief.diameter / 2, h.relief.start, h.depth);
				try tools.push(PosedParts.moved(relief, h.frame))
				catch (error:Dynamic) {relief.close(); throw error;}
				relief.close();
			}
			if(h.counterbore!=null) {
				var cb=Part.cylinderSpan(h.counterbore.radius,-1,h.counterbore.depth);
				tools.push(PosedParts.moved(cb,h.frame)); cb.close();
			}
		} catch(e:Dynamic) {for(t in tools)t.close();throw e;}
		return tools;
	}
}
