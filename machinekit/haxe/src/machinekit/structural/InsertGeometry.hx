package machinekit.structural;

import cadkit.modeling.Part;
import machinekit.component.Solids;
import materia.casting.InsertSpec;
import materia.casting.InsertStages.InsertStage;

/** Source and tooling realize the same neutral insert, without inventing a
 * component role, material or label to reconstruct contact geometry. */
class InsertGeometry {
	public static function part(authored:InsertSpec,stage:InsertStage):Part {
		var spec=InsertSpecCodec.snapshot(authored);
		var size=switch InsertBodyCodec.mouth(spec.body) {
			case Box(w,d,h):{width:w,depth:d,height:h};
			case Cylinder(d,h):{width:d,depth:d,height:InsertBodyCodec.height(spec.body)};
		};
		var drilling=InsertHoles.facet(materia.casting.InsertStages.holes(spec,stage),size.width,size.depth,size.height);
		var physicalBody=spec.body,pose=materia.assembly.AssemblyFrames.identity();
		if (stage==materia.casting.InsertStages.InsertStage.Service)
			switch materia.casting.InsertFinishing.assess(spec.body,spec.interfaces) {
				case Resolved(steel):physicalBody=steel.body;pose=steel.pose;
				case Invalid(_),Unsupported(_):
			}
		var local=InsertBodies.part(physicalBody);
		var placed:Part;
		try placed=machinekit.assembly.PosedParts.moved(local,pose)
		catch (error:Dynamic) {local.close();throw error;}
		local.close();
		var body = Solids.named(placed, "insert");
		return Solids.building([body], owned -> {
			var tools = drilling.drillingTools();
			for (tool in tools)
				owned.push(tool);
			return tools.length == 0 ? body : Solids.cut(body, tools);
		});
	}
}
