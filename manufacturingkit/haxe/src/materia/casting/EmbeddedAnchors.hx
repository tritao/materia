package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef EmbeddedInsertAnchors = {
	var owner:String;
	var pose:AssemblyFrame;
	var spec:InsertSpec;
}

/** Derived source occupancy, copied from the owning inserts when a foundation
 * is built. It is never a second independent anchor layout or a box substitute. */
class EmbeddedAnchors {
	/** Exact nominal cylinder/hex-socket moments of the declared hardware.
	 * exposedOnly removes the shank portion inside the rectangular steel blank;
	 * otherwise returns the metal added to the steel union (thread-envelope
	 * overlap is counted once). Callers qualify containment with source CAD. */
	public static function moments(spec:InsertSpec, exposedOnly:Bool):{volume:Float, center:{x:Float,y:Float,z:Float}} {
		var volume=0.0,mx=0.0,my=0.0,mz=0.0;
		function add(v:Float,frame:AssemblyFrame,z:Float):Void {
			var p=materia.assembly.AssemblyFrames.transformPoint(frame,0,0,z);
			volume+=v;mx+=v*p.x;my+=v*p.y;mz+=v*p.z;
		}
		switch spec.anchorage {
			case Undeclared,CatalogueRetention:
			case Bolted(_,anchors):for (anchor in anchors) {
				var hole=spec.holes.filter(h -> h.id==anchor.holeId)[0];
				var frame=InsertAnchorage.InsertAnchorageCodec.bearingFrame(hole,anchor);
				var screw=anchor.screw,washer=anchor.washer;
				var shankLength=exposedOnly ? -frame.z : screw.lengthMm;
				if (shankLength<0 || shankLength>screw.lengthMm) throw "Invalid anchor exposure: "+anchor.id;
				add(Math.PI*Math.pow(screw.thread.majorDiameter/2,2)*shankLength,frame,shankLength/2);
				add(Math.PI*Math.pow(screw.headDiameterMm/2,2)*screw.headHeightMm,frame,-screw.headHeightMm/2);
				// A regular hexagon's area is sqrt(3)/2 times across-flats squared.
				add(-Math.sqrt(3)/2*screw.socketSizeMm*screw.socketSizeMm*screw.socketDepthMm,
					frame,-screw.headHeightMm+screw.socketDepthMm/2);
				add(Math.PI*(washer.outerDiameterMm*washer.outerDiameterMm-washer.innerDiameterMm*washer.innerDiameterMm)
					/4*washer.thicknessMm,frame,washer.thicknessMm/2);
				if (!exposedOnly) {
					var raw=InsertStages.holes(spec,InsertStages.InsertStage.BeforeCasting).filter(h -> h.id==hole.id)[0];
					var end=hole.frame.z+anchor.insertionMm;
					var events=[0.0,end];
					if (raw.counterbore!=null && raw.counterbore.depth<end) events.push(raw.counterbore.depth);
					if (raw.relief!=null && raw.relief.start<end) events.push(raw.relief.start);
					events.sort(Reflect.compare);
					for (index in 0...events.length-1) {
						var low=events[index],high=events[index+1],mid=(low+high)/2;
						var diameter=raw.diameter;
						if (raw.counterbore!=null && mid<raw.counterbore.depth) diameter=Math.max(diameter,raw.counterbore.diameter);
						if (raw.relief!=null && mid>=raw.relief.start) diameter=Math.max(diameter,raw.relief.diameter);
						var overlap=Math.PI/4*Math.max(0,screw.thread.majorDiameter*screw.thread.majorDiameter-diameter*diameter)*(high-low);
						add(-overlap,raw.frame,mid);
					}
				}
			}
		}
		return {volume:volume,center:volume==0 ? {x:0.0,y:0.0,z:0.0} : {x:mx/volume,y:my/volume,z:mz/volume}};
	}

	public static function snapshot(raw:Dynamic):Array<EmbeddedInsertAnchors> {
		if (!Std.isOfType(raw,Array)) throw "Invalid embedded anchor list";
		var rows:Array<Dynamic> = cast raw,result:Array<EmbeddedInsertAnchors> = [],seen:Map<String,Bool> = [];
		if (rows.length>512) throw "Too many embedded anchor owners";
		for (row in rows) {
			CastingFeatureCodec.fields(row,["owner","pose","spec"]);
			var owner=CastingRecordCodec.string(row,"owner");CastingFeatureCodec.identifier(owner);
			if (seen.exists(owner)) throw "Duplicate embedded anchor owner: "+owner;seen.set(owner,true);
			var spec=InsertSpec.InsertSpecCodec.snapshot(Reflect.field(row,"spec"));
			switch spec.anchorage {case Bolted(_,_):default:throw "Embedded hardware requires bolted source anchorage: "+owner;}
			result.push({owner:owner,pose:CastingRecordCodec.frame(Reflect.field(row,"pose")),spec:spec});
		}
		result.sort((a,b) -> Reflect.compare(a.owner,b.owner));return result;
	}
	public static function records(rows:Array<EmbeddedInsertAnchors>):Array<Dynamic>
		return [for (row in snapshot(rows)) {owner:row.owner,pose:row.pose,spec:InsertSpec.InsertSpecCodec.record(row.spec)}];
}
