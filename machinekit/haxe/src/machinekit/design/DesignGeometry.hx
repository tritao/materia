package machinekit.design;

import cadkit.modeling.Part;
import machinekit.component.Solids;
import machinekit.assembly.PosedParts;
import machinekit.design.DesignRequirement.DesignEnvelope;

class DesignGeometry {
	public static function envelope(e:DesignEnvelope):Part {
		validateFrame(e.frame);
		var local=switch(e.volume) {
			case BoxVolume(w,d,h): finite([w,d,h]);if(w<=0 || d<=0 || h<=0) throw "Invalid design box"; Part.box(w,d,h);
			case CylinderVolume(r,a,b): finite([r,a,b]);Part.cylinderSpan(r,a,b);
			case RingVolume(r,bore,a,b):
				finite([r,bore,a,b]);
				if(bore<=0 || bore>=r) throw "Invalid material ring";
				Solids.cut(Part.cylinderSpan(r,a,b),[Part.cylinderSpan(bore,a-1,b+1)]);
			case SocketVolume(r,a,b,mouth,end):
				finite([r,a,b,mouth,end]);
				if(mouth<=0 || mouth>=r || end<=a || end>=b) throw "Invalid coupler mouth";
				Solids.cut(Part.cylinderSpan(r,a,b),[Part.cylinderSpan(mouth,a-1,end)]);
		};
		try {var result=PosedParts.moved(local,e.frame);local.close();return result;} catch(error:Dynamic) {local.close();throw error;}
	}
	static function finite(values:Array<Float>):Void {for(v in values)if(!Math.isFinite(v))throw "Non-finite design envelope";}
	public static function validateFrame(f:materia.assembly.AssemblyRecord.AssemblyFrame):Void {
		finite([f.x,f.y,f.z,f.qx,f.qy,f.qz,f.qw]);
		if(Math.abs(f.qx*f.qx+f.qy*f.qy+f.qz*f.qz+f.qw*f.qw-1)>1e-6)throw "Non-unit design frame";
	}
}
