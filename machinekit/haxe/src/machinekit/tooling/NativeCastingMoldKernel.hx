package machinekit.tooling;

import cadkit.modeling.Part;
import cadkit.modeling.Scope;
import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingRegion;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldKernel;
import materia.tooling.casting.CastingMoldKernel.CastingRegionMoments;
import materia.tooling.casting.CastingMoldKernel.CastingMassProperties;

/** Exact CAD queries used by non-box planning. Caches retain numbers only;
 * every native resource closes before a query returns, including throw paths.
 */
class NativeCastingMoldKernel implements CastingMoldKernel {
	final regionCache:Map<String,CastingRegionMoments> =[];
	final regionKeys:Array<String> =[];
	final massCache:Map<String,CastingMassProperties> = [];
	final massKeys:Array<String> = [];
	static inline var CACHE_LIMIT=4096;

	public function new() {}

	public function moments(region:CastingRegion):CastingRegionMoments {
		var key=haxe.crypto.Sha256.encode(ToolingCodec.canonical(region));
		var cached=regionCache.get(key);
		if (cached!=null) return copy(cached);
		var part=machinekit.structural.CastingRegionCad.part(region);
		var result:CastingRegionMoments;
		try result=measure(part) catch (error:Dynamic) {part.close();throw error;}
		part.close();
		if (regionKeys.length>=CACHE_LIMIT) regionCache.remove(regionKeys.shift());
		regionKeys.push(key);
		regionCache.set(key,copy(result));
		return result;
	}

	static function copy(value:CastingRegionMoments):CastingRegionMoments
		return {volumeMm3:value.volumeMm3,firstMomentMm4:{x:value.firstMomentMm4.x,y:value.firstMomentMm4.y,z:value.firstMomentMm4.z}};

	static function measure(part:Part):CastingRegionMoments {
		if (part.isEmpty()) return {volumeMm3:0,firstMomentMm4:{x:0,y:0,z:0}};
		var properties=part.massProperties(),volume=properties.volume,center=properties.centerOfMass;
		var result={volumeMm3:volume,firstMomentMm4:{x:volume*center.x,y:volume*center.y,z:volume*center.z}};
		for (value in [volume,result.firstMomentMm4.x,result.firstMomentMm4.y,result.firstMomentMm4.z])
			if (!Math.isFinite(value)) throw "Nonfinite native casting moments";
		if (volume<0) throw "Negative native casting volume";
		return result;
	}

	public function castingMass(document:ToolingDocument):CastingMassProperties {
		var d=ToolingCodec.snapshot(document),key=ToolingCodec.documentFingerprint(d);
		var cached=massCache.get(key);
		if (cached!=null) return copyMass(cached);
		var scope=new Scope();
		var total={mass:0.0,x:0.0,y:0.0,z:0.0};
		try {
			var resin=scope.own(ToolingSourceCasting.fromInput(d.input).geometry());
			contribution(total,measure(resin),d.setup.sourceToMold,d.recipe.mixDensityKgM3);
			for (insert in d.input.inserts) {
				var local=scope.own(ToolingInsertCad.component(insert).geometry());
				local.shape.setThreadedFaces([]);
				var parts=[local];
				var spec:materia.casting.InsertSpec={body:insert.body,holes:insert.holes,origin:insert.origin,
					interfaces:insert.interfaces,anchorage:insert.anchorage};
				for (hardware in machinekit.structural.InsertAnchorHardware.parts(spec)) {
					var raw=scope.own(hardware.part.geometry());
					raw.shape.setThreadedFaces([]);
					parts.push(scope.own(machinekit.assembly.PosedParts.moved(raw,hardware.pose)));
				}
				var steel=scope.own(Part.fuseAll(parts));
				contribution(total,measure(steel),F.compose(d.setup.sourceToMold,insert.pose),d.recipe.steelDensityKgM3);
			}
			scope.close();
		} catch (error:Dynamic) {
			scope.close();
			throw error;
		}
		if (!Math.isFinite(total.mass+total.x+total.y+total.z) || total.mass<=0)
			throw "Invalid native casting mass: "+d.input.id;
		var result={massKg:total.mass,centerOfMassMoldMm:{x:total.x/total.mass,y:total.y/total.mass,z:total.z/total.mass}};
		if (massKeys.length>=CACHE_LIMIT) massCache.remove(massKeys.shift());
		massKeys.push(key);
		massCache.set(key,copyMass(result));
		return result;
	}

	public function resinIntersection(document:ToolingDocument,region:CastingRegion):Float {
		var d=ToolingCodec.snapshot(document),scope=new Scope();
		try {
			var local=scope.own(ToolingSourceCasting.fromInput(d.input).geometry());
			var resin=scope.own(machinekit.assembly.PosedParts.moved(local,d.setup.sourceToMold));
			var bounds=resin.shape.bounds();
			var low=cadkit.modeling.Vector.fromNative(bounds.get_min()),high=cadkit.modeling.Vector.fromNative(bounds.get_max());
			var height=Math.min(high.z,d.setup.fillHeightMm)-low.z;
			var volume=0.0;
			if (height>0) {
				var blank=scope.own(Part.box(high.x-low.x,high.y-low.y,height));
				var clip=scope.own(blank.translated(new cadkit.modeling.Vector(
					(low.x+high.x)/2,(low.y+high.y)/2,low.z)));
				var filled=scope.own(resin.intersect(clip));
				var candidate=scope.own(machinekit.structural.CastingRegionCad.part(region));
				volume=measure(scope.own(filled.intersect(candidate))).volumeMm3;
			}
			scope.close();
			return volume;
		} catch (error:Dynamic) {
			scope.close();
			throw error;
		}
	}

	static function copyMass(value:CastingMassProperties):CastingMassProperties
		return {massKg:value.massKg,centerOfMassMoldMm:{x:value.centerOfMassMoldMm.x,
			y:value.centerOfMassMoldMm.y,z:value.centerOfMassMoldMm.z}};

	static function contribution(total:{mass:Float,x:Float,y:Float,z:Float},moments:CastingRegionMoments,
			pose:materia.assembly.AssemblyRecord.AssemblyFrame,densityKgM3:Float):Void {
		if (moments.volumeMm3==0) return;
		var factor=densityKgM3*1e-9,first=moments.firstMomentMm4;
		var center=F.transformPoint(pose,first.x/moments.volumeMm3,first.y/moments.volumeMm3,first.z/moments.volumeMm3);
		var mass=moments.volumeMm3*factor;
		total.mass+=mass;
		total.x+=mass*center.x;
		total.y+=mass*center.y;
		total.z+=mass*center.z;
	}
}
