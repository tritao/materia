package machinekit.structural;

import machinekit.component.MachineComponent;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;
import materia.casting.Box3;
import materia.casting.InsertHole;
import materia.casting.InsertManufacturing;
import materia.casting.HoleClearanceBudget;
import materia.casting.HoleClearanceBudget.HoleClearanceBudgets;
import machinekit.structural.InsertMountPattern.MountHoleErrorAllocation;

typedef DerivedMountHoles = {
	var holes:Array<InsertHole>;
	var requirements:Array<materia.casting.InsertMountHoleRequirement>;
	var routes:Array<{holeId:String,route:InsertHoleRoute}>;
}

/** Shared rail/rack/scale drilling, in the receiver steel's coordinates.
 * Local +Z is the receiver face normal. The receiver may be a cast-in insert
 * or a separate steel member on a welded host; casting is not assumed here.
 */
class MountHoleDerivation {
	public static function derive(stock:Box3, material:materia.casting.InsertMaterial,
		component:MachineComponent, path:String, fingerprint:String, placement:AssemblyFrame,
		connectors:Array<String>, thread:cadkit.threads.ThreadSpec,
		clearance:Null<Float>, reference:Null<String>, permitted:Array<InsertHoleRoute>,
		allocation:MountHoleErrorAllocation):DerivedMountHoles {
		if (connectors.length==0 || StringTools.trim(path)=="" || StringTools.trim(fingerprint)=="")
			throw "Mount holes require placed-element identity and connectors";
		for(value in [stock.x,stock.y,stock.z,stock.width,stock.depth,stock.height])
			if(!Math.isFinite(value)) throw "Receiver steel must have finite dimensions";
		if(stock.width<=0 || stock.depth<=0 || stock.height<=0) throw "Receiver steel must have positive dimensions";
		machinekit.design.DesignGeometry.validateFrame(placement);
		var result:DerivedMountHoles={holes:[],requirements:[],routes:[]};
		var seen:Map<String,Bool>=[];
		var choice=InsertHoles.metricThread(thread);
		var drill=F.fromRotationMatrix(0,0,0,[1,0,0,0,0,-1,0,1,0]);
		for (name in connectors) {
			if (seen.exists(name)) throw "Duplicate element mounting connector: "+path+"/"+name;
			seen.set(name,true);
			var connector=F.compose(placement,component.connector(name).frame);
			var axis=F.transformVector(connector,0,1,0);
			if (Math.abs(axis.x)>1e-9 || Math.abs(axis.y)>1e-9 || Math.abs(axis.z-1)>1e-9)
				throw "Mounting connector is not outward normal to receiver steel: "+path+"/"+name;
			var hole=InsertHoles.records(InsertHoles.tappedBlind(choice,stock.height,material))[0];
			hole.id="mount-"+name;
			hole.frame=F.compose(connector,drill);
			hole.frame.z=stock.z+stock.height;
			materia.casting.InsertStages.validateServiceHole(hole,stock);
			var budget:HoleClearanceBudget={boltDiameterMm:thread.majorDiameter,clearanceDiameterMm:clearance,
				distanceFromDatumMm:Math.sqrt(hole.frame.x*hole.frame.x+hole.frame.y*hole.frame.y),
				placementOffsetMm:allocation.placementOffsetMm,placementAngleRad:allocation.placementAngleRad,
				assemblyAdjustmentMm:allocation.assemblyAdjustmentMm,cureBowMm:allocation.cureBowMm,
				machiningErrorMm:allocation.machiningErrorMm,pitchAccumulationMm:allocation.pitchAccumulationMm,
				requiredMarginMm:allocation.requiredMarginMm};
			result.requirements.push({holeId:hole.id,componentPath:path,componentFingerprint:fingerprint,
				connectorId:name,connectorFrame:connector,clearanceReference:reference,
				permittedRoutes:permitted.copy(),budget:budget});
			switch HoleClearanceBudgets.choose(budget,permitted) {
				case UsePreCast(_):result.routes.push({holeId:hole.id,route:InsertHoleRoute.BeforeCasting});
				case UseTransfer(_),ProposeTransfer(_):result.routes.push({holeId:hole.id,route:InsertHoleRoute.TransferAfterCasting});
				case AwaitInputs(_),ChangeDesign(_):
			}
			result.holes.push(hole);
		}
		// Apply the same lineage/budget schema on cast and welded receivers.
		materia.casting.InsertMountHoleRequirement.InsertMountHoleRequirements.snapshot(result.requirements);
		return result;
	}
}
