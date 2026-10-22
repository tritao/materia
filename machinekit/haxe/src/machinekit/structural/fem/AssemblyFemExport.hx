package machinekit.structural.fem;

import cadkit.modeling.AssemblyState;
import haxe.crypto.Sha256;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;
import materia.project.MaterialLibrary;
import sys.io.File;

/** Study-independent posed bodies. Callers own selections, interfaces and solver assumptions. */
class AssemblyFemExport {
	/** Preserve geometry while stabilizing timestamps and OCCT's Product N labels. */
	public static function stepText(text:String,memberId:String):String {
		var end = text.indexOf("DATA;");
		if (end < 0) throw "STEP export has no DATA section";
		var timestamp = ~/[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}/g;
		var normalized=timestamp.replace(text.substr(0, end), "1970-01-01T00:00:00") + text.substr(end);
        var product=~/PRODUCT\('Product [0-9]+','Product [0-9]+'/g;
        var label=StringTools.replace(memberId,"'","''");
        return product.replace(normalized,"PRODUCT('"+label+"','"+label+"'");
	}

    public static function bodies(assembly:MachineAssembly,state:AssemblyState,folder:String,
        exportSolid:String->Bool):{parts:Array<Dynamic>,materials:Array<Dynamic>,totalMassKg:Float,solidMassKg:Float} {
        var poses=assembly.solvedPoses(state),parts:Array<Dynamic> = [],materials:Array<Dynamic> = [],materialIds:Array<String> = [];
        var members=assembly.components(),parents=assembly.mateParents();
        members.sort((a,b)->Reflect.compare(a.id,b.id));
        var totalMass=0.0,solidMass=0.0;
        PosedParts.scope(cache -> {
			for (member in members) {
				var id = member.id, component = member.component, pose = poses.get(id);
				if (pose == null) throw 'No FEM pose for $id';
				materia.assembly.AssemblyCodec.validateFrame(pose);
				var material = MaterialLibrary.require(component.materialId);
				if (materialIds.indexOf(material.id) < 0) {
					materialIds.push(material.id);
					materials.push({id:material.id, densityKgPerM3:material.physical.density,
						youngsModulusPa:material.physical.youngsModulusPa,
						propertiesAssumed:material.physical.propertiesAssumed == true,
						poissonRatio:null, source:"projectkit/src/materia/project/MaterialLibrary.hx"});
				}
				var mass = component.massProperties(), c = mass.centreOfMass;
				if (!Math.isFinite(mass.mass) || mass.mass <= 0) throw 'Invalid FEM mass: $id';
				var centre = F.transformPoint(pose,c.x,c.y,c.z);
                var inertia = mass.inertia == null ? null : mass.inertia.rotated(pose.qx,pose.qy,pose.qz,pose.qw);
                var inertiaSource = mass.inertia == null ? "missing" : "component centroidal tensor rotated to world axes";
				var solid = exportSolid(id), file:Null<String> = null, digest:Null<String> = null;
				var volume:Null<Float> = null, bounds:Dynamic = null;
				{
					var part = cache.posed(component,pose);
					try {
						volume = part.volume();
						if (!part.valid() || (solid && part.solidCount() != 1) || !Math.isFinite(volume) || volume <= 0)
							throw 'FEM solid must be valid, connected and positive-volume: $id';
						bounds = PosedParts.boxOf(part);
                    if (inertia == null) {
                        var physical = part.massProperties();
                        if (Math.abs(physical.centerOfMass.x-centre.x)+Math.abs(physical.centerOfMass.y-centre.y)+Math.abs(physical.centerOfMass.z-centre.z)>1e-5)
                            throw 'Declared FEM mass needs an explicit centroidal tensor when its COM differs from CAD: $id';
                        inertia = physical.inertiaAtDensity(mass.mass/physical.volume);
                        inertiaSource = "assumed CAD mass distribution scaled to declared component mass";
                    }
						if(solid) {
						file = StringTools.replace(id,"/","__") + ".step";
						part.shape.exportStep(folder + "/" + file);
						var text = stepText(File.getContent(folder + "/" + file),id);
						File.saveContent(folder + "/" + file,text);
						digest = Sha256.encode(text);
						}
					} catch (error:Dynamic) {part.close(); throw error;}
					part.close();
					if(solid)solidMass += mass.mass;
				}
				totalMass += mass.mass;
                var massSource = switch(mass.source) {case Computed(_):"Computed"; case Declared:"Declared"; case EffectiveDensity(_):"EffectiveDensity"; case Modelled:"Modelled";};
				parts.push({id:id, parentId:parents.get(id), materialId:material.id, representation:solid?"solid":"mass-only",
					poseMm:pose, massKg:mass.mass, massSource:massSource, centreOfMassMm:centre, volumeMm3:volume,
                    centroidalInertiaKgMm2World:inertia, inertiaSource:inertiaSource,
					boundsMm:bounds, stepFile:file, stepSha256:digest});
			}
        });
        materials.sort((a,b)->Reflect.compare(a.id,b.id));
        return {parts:parts,materials:materials,totalMassKg:totalMass,solidMassKg:solidMass};
    }
}
