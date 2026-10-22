package machinekit.structural;

import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.HexNut;
import machinekit.standard.FlatWasher;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

typedef ThroughMountBolt = {
	var screw:SocketHeadCapScrew;
	var washer:FlatWasher;
	var nut:HexNut;
	var screwPose:AssemblyFrame;
	var washerPose:AssemblyFrame;
	var nutPose:AssemblyFrame;
	var gripMm:Float;
	var projectionMm:Float;
}

/** Geometric alternative to a tapped receiver: a real class-8 nut behind a
 * clearance-drilled backing, not a C45 thread policy applied to a thin pad.
 * Bearing/pull-through, preload, capture and tool access still need qualification.
 */
class MountThroughBolts {
	public static function select(id:String,element:MountedElement,seat:AssemblyFrame,
		backFace:AssemblyFrame,preferredLengths:Array<Float>):ThroughMountBolt {
		machinekit.design.DesignGeometry.validateFrame(seat);
		machinekit.design.DesignGeometry.validateFrame(backFace);
		var size=element.thread.designation;
		var nut=HexNut.metric(size),washer=FlatWasher.metric(size);
		if(!nut.thread.matches(element.thread))throw "Mounted element and nut threads differ: "+id;
		var lengths=preferredLengths.copy();lengths.sort(Reflect.compare);
		for(length in lengths)if(!Math.isFinite(length) || length<=0)throw "Invalid through-bolt preferred length";
		for(length in lengths) {
			var screw=SocketHeadCapScrew.metric(size,length,Steel88),bore=element.bore;
			if(screw.spec.headDiameter>bore.counterboreDiameter || screw.spec.headHeight>bore.counterboreDepth
				|| screw.diameter>bore.diameter || washer.innerDiameter<screw.diameter)
				throw "Through-bolt hardware does not fit mounted element: "+id;
			var pose=F.compose(seat,F.inverse(screw.connector("head").frame));
			var bearing=F.compose(pose,screw.bearingFrame()),axis=F.transformVector(bearing,0,0,1);
			var dx=backFace.x-bearing.x,dy=backFace.y-bearing.y,dz=backFace.z-bearing.z;
			var grip=dx*axis.x+dy*axis.y+dz*axis.z;
			var normal=F.transformVector(backFace,0,1,0);
			if(grip<=0 || Math.abs(Math.abs(normal.x*axis.x+normal.y*axis.y+normal.z*axis.z)-1)>1e-6
				|| Math.abs(dx-axis.x*grip)+Math.abs(dy-axis.y*grip)+Math.abs(dz-axis.z*grip)>1e-6)
				throw "Through-bolt back face is not coaxial, perpendicular and behind the head: "+id;
			var threadStart=grip+washer.thickness,projection=length-threadStart-nut.height;
			if(length-screw.threadLength>threadStart+1e-6 || projection<2*screw.thread.pitch-1e-6)continue;
			return {screw:screw,washer:washer,nut:nut,screwPose:pose,
				washerPose:F.compose(bearing,F.translation(0,0,grip)),
				nutPose:F.compose(bearing,F.translation(0,0,threadStart)),gripMm:grip,projectionMm:projection};
		}
		throw "No preferred through-bolt fully engages the nut with two pitches exposed: "+id;
	}
}
