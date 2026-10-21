package materia.casting;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.InsertSpec.InsertBody;
import materia.casting.InterfaceRequirement;
import materia.casting.InterfaceRequirement.InterfaceRequirements;
import materia.casting.InterfaceStock.InterfaceStocks;

/** Geometry only: a replicated layer is not steel and has no invented material. */
typedef InsertFinishedSteel = {
	var body:InsertBody;
	var pose:AssemblyFrame;
	var removedSteel:Array<Box3>;
	var replicatedLayers:Array<{interfaceId:String,region:Box3}>;
}

typedef InsertFinishIssue = {var interfaceIds:Array<String>;var message:String;}

enum InsertFinishResult {
	Resolved(steel:InsertFinishedSteel);
	Invalid(issues:Array<InsertFinishIssue>);
	Unsupported(issues:Array<InsertFinishIssue>);
}

/** One exact stock-to-service realization. Plane removals constrain the finished
 * box; replication thickness is measured from that finished steel, not raw stock.
 * All coordinates remain in the insert's authored frame.
 */
class InsertFinishing {
	public static inline var EPS_MM = 1e-8;

	public static function assess(body:InsertBody,authored:Array<InterfaceRequirement>):InsertFinishResult {
		InsertSpec.InsertBodyCodec.validate(body);
		var requirements=InterfaceRequirements.snapshot(authored),invalid:Array<InsertFinishIssue> = [],unsupported:Array<InsertFinishIssue> = [];
		var raw:Null<Box3> = switch body {
			case Primitive(solid):switch solid {
				case Box(w,d,h):{x:-w/2,y:-d/2,z:0,width:w,depth:d,height:h};
				case Cylinder(_,_):null;
			};
			case CrimpedSocket(_,_):null;
		};
		var low=raw==null ? [0.0,0,0] : [raw.x,raw.y,raw.z];
		var high=raw==null ? [0.0,0,0] : [raw.x+raw.width,raw.y+raw.depth,raw.z+raw.height];
		var removedIds:Array<String> = [];
		for (requirement in requirements) {
			if (requirement.route==InterfaceRoute.Replicated) continue;
			switch InterfaceStocks.assess(body,requirement) {
				case NoLayer:
				case Invalid(reason):invalid.push({interfaceIds:[requirement.id],message:reason});
				case Unsupported(reason):unsupported.push({interfaceIds:[requirement.id],message:reason});
				case Replicate(_):throw "Replication must follow steel finishing";
				case Remove(_):
					var frame=InterfaceRequirements.geometryFrame(requirement.geometry),normal=F.transformVector(frame,0,0,1),axis=CastingBounds.axis(normal);
					if (axis==null) throw "Supported stock layer has no axis";
					var position=[frame.x,frame.y,frame.z],direction=[normal.x,normal.y,normal.z];
					if (direction[axis]>0) high[axis]=Math.min(high[axis],position[axis]);
					else low[axis]=Math.max(low[axis],position[axis]);
					removedIds.push(requirement.id);
			}
		}
		if (invalid.length>0) return Invalid(invalid);
		if (unsupported.length>0) return Unsupported(unsupported);
		var result:InsertFinishedSteel={body:InsertSpec.InsertBodyCodec.snapshot(body),pose:F.identity(),removedSteel:[],replicatedLayers:[]};
		if (removedIds.length>0) {
			if (raw==null) throw "Non-box removal passed the stock qualifier";
			for (axis in 0...3) if (high[axis]-low[axis]<=EPS_MM)
				return Invalid([{interfaceIds:removedIds,message:"Combined finishing leaves no retained steel"}]);
			var service:Box3={x:low[0],y:low[1],z:low[2],width:high[0]-low[0],depth:high[1]-low[1],height:high[2]-low[2]};
			result.body=Primitive(CastingSolid.Box(service.width,service.depth,service.height));
			result.pose=F.translation(service.x+service.width/2,service.y+service.depth/2,service.z);
			result.removedSteel=BoxAlgebra.subtract([raw],[service]);
			for (requirement in requirements) if (requirement.route!=InterfaceRoute.Replicated) switch requirement.geometry {
				case Line(_):
				case Plane(frame):
					var normal=F.transformVector(frame,0,0,1),axis=CastingBounds.axis(normal);
					if (axis==null) continue;
					var direction=[normal.x,normal.y,normal.z],position=[frame.x,frame.y,frame.z];
					var boundary=direction[axis]>0 ? high[axis] : low[axis];
					if (Math.abs(position[axis]-boundary)>EPS_MM)
						invalid.push({interfaceIds:[requirement.id].concat(removedIds),message:"Finishing requirements disagree on the retained steel face"});
			}
		}
		if (invalid.length>0) return Invalid(invalid);
		for (requirement in requirements) if (requirement.route==InterfaceRoute.Replicated) {
			var local=InterfaceRequirements.snapshot([requirement])[0];
			local.geometry=switch local.geometry {
				case Plane(frame):Plane(F.compose(F.inverse(result.pose),frame));
				case Line(frame):Line(F.compose(F.inverse(result.pose),frame));
			};
			switch InterfaceStocks.assess(result.body,local) {
				case Replicate(layer):result.replicatedLayers.push({interfaceId:requirement.id,region:{
					x:layer.x+result.pose.x,y:layer.y+result.pose.y,z:layer.z+result.pose.z,
					width:layer.width,depth:layer.depth,height:layer.height}});
				case Invalid(reason):invalid.push({interfaceIds:[requirement.id],message:reason});
				case Unsupported(reason):unsupported.push({interfaceIds:[requirement.id],message:reason});
				case NoLayer,Remove(_):throw "Replicated requirement has no added layer";
			}
		}
		return invalid.length>0 ? Invalid(invalid) : unsupported.length>0 ? Unsupported(unsupported) : Resolved(result);
	}

	public static function box(steel:InsertFinishedSteel):Null<Box3> return switch steel.body {
		case Primitive(solid):switch solid {
			case Box(w,d,h):{x:steel.pose.x-w/2,y:steel.pose.y-d/2,z:steel.pose.z,width:w,depth:d,height:h};
			case Cylinder(_,_):null;
		};
		case CrimpedSocket(_,_):null;
	};
}
