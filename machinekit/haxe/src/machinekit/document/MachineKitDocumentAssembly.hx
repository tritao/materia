package machinekit.document;

import cadkit.modeling.AssemblyModel;
import cadkit.parametric.DefinitionEvaluatorRegistry;
import cadkit.parametric.DefinitionOutput;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.Placement;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Adds a document instance and its named connector outputs to a kinematics assembly. */
class MachineKitDocumentAssembly {
	public static function add(model:AssemblyModel, id:String, instance:InstanceElement):Void {
		var definition = instance.document.definition(instance.definitionId);
		model.add(id, frame(instance.document.worldPlacement(instance)));
		for (output in definition.outputs()) if (output.purpose == DefinitionOutput.Connector)
			model.connector(id, output.name, frame(DefinitionEvaluatorRegistry.connector(definition, instance, output.name)));
	}

	/** Convert a CadKit placement to the kinematics x,y,z,w frame convention. */
	public static function frame(placement:Placement):AssemblyFrame {
		var plane = placement.location.plane;
		var x = plane.xDirection, y = plane.yDirection, z = plane.normal;
		var m00 = x.x, m01 = y.x, m02 = z.x;
		var m10 = x.y, m11 = y.y, m12 = z.y;
		var m20 = x.z, m21 = y.z, m22 = z.z;
		var qx:Float, qy:Float, qz:Float, qw:Float;
		var trace = m00 + m11 + m22;
		if (trace > 0) {
			var s = Math.sqrt(trace + 1) * 2;
			qw = s / 4; qx = (m21 - m12) / s; qy = (m02 - m20) / s; qz = (m10 - m01) / s;
		} else if (m00 > m11 && m00 > m22) {
			var s = Math.sqrt(1 + m00 - m11 - m22) * 2;
			qw = (m21 - m12) / s; qx = s / 4; qy = (m01 + m10) / s; qz = (m02 + m20) / s;
		} else if (m11 > m22) {
			var s = Math.sqrt(1 + m11 - m00 - m22) * 2;
			qw = (m02 - m20) / s; qx = (m01 + m10) / s; qy = s / 4; qz = (m12 + m21) / s;
		} else {
			var s = Math.sqrt(1 + m22 - m00 - m11) * 2;
			qw = (m10 - m01) / s; qx = (m02 + m20) / s; qy = (m12 + m21) / s; qz = s / 4;
		}
		return {x: plane.origin.x, y: plane.origin.y, z: plane.origin.z, qx: qx, qy: qy, qz: qz, qw: qw};
	}
}
