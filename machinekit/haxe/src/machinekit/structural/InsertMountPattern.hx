package machinekit.structural;

import machinekit.component.MachineComponent;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;
import materia.casting.HoleClearanceBudget;
import materia.casting.HoleClearanceBudget.HoleClearanceBudgets;
import materia.casting.InsertManufacturing.InsertHoleRoute;

typedef MountHoleErrorAllocation = {
	var placementOffsetMm:Null<Float>;
	var placementAngleRad:Null<Float>;
	var assemblyAdjustmentMm:Null<Float>;
	var cureBowMm:Null<Float>;
	var machiningErrorMm:Null<Float>;
	var pitchAccumulationMm:Null<Float>;
	var requiredMarginMm:Null<Float>;
}

/** Source authoring only. Mounting positions come from placed component
 * connectors; the one stored hole frame is the intersection with finished steel.
 */
class InsertMountPattern {
	public static function derive(source:CastInsert, component:MachineComponent, componentPath:String,
		componentFingerprint:String, componentToInsert:AssemblyFrame, connectorIds:Array<String>,
		thread:cadkit.threads.ThreadSpec, clearanceDiameter:Null<Float>, clearanceReference:Null<String>,
		permitted:Array<InsertHoleRoute>, allocation:MountHoleErrorAllocation):CastInsert {
		var spec=source.insertSpec();
		var manufacturing=switch spec.origin {
			case Fabricated(plan):plan;
			case Catalogue(_):throw "A catalogue insert cannot acquire fabricated mounting holes";
		};
		var finished=switch materia.casting.InsertFinishing.assess(spec.body,spec.interfaces) {
			case Resolved(steel):steel;
			case Invalid(reason),Unsupported(reason):throw "Resolve mounting-hole steel: "+reason;
		};
		var stock=materia.casting.InsertFinishing.box(finished);
		if (stock==null) throw "Unsupported non-box mounting-hole steel: "+componentPath;
		var choice=InsertHoles.metricThread(thread);
		var drill=F.fromRotationMatrix(0,0,0,[1,0,0,0,0,-1,0,1,0]);
		for (index in 0...connectorIds.length) {
			var name=connectorIds[index];
			var connector=F.compose(componentToInsert,component.connector(name).frame);
			var axis=F.transformVector(connector,0,1,0);
			if (Math.abs(axis.x)>1e-9 || Math.abs(axis.y)>1e-9 || Math.abs(Math.abs(axis.z)-1)>1e-9)
				throw "Mounting connector is not normal to the steel face: "+componentPath+"/"+name;
			if (axis.z<=0) throw "Mounting connector points into the steel: "+componentPath+"/"+name;
			var distance=(stock.z+stock.height-connector.z)/axis.z;
			var hole=InsertHoles.records(InsertHoles.tappedBlind(choice,stock.height,source.insertMaterial().grade))[0];
			hole.id="mount-"+name;
			hole.frame=F.compose(connector,drill);
			hole.frame.x=connector.x+axis.x*distance;
			hole.frame.y=connector.y+axis.y*distance;
			hole.frame.z=stock.z+stock.height;
			if (spec.holes.filter(existing -> existing.id==hole.id).length>0)
				throw "Duplicate source mounting hole: "+componentPath+"/"+name;
			materia.casting.InsertStages.validateServiceHole(hole,stock);
			var budget:HoleClearanceBudget={boltDiameterMm:thread.majorDiameter,
				clearanceDiameterMm:clearanceDiameter,
				distanceFromDatumMm:Math.sqrt(hole.frame.x*hole.frame.x+hole.frame.y*hole.frame.y),
				placementOffsetMm:allocation.placementOffsetMm,placementAngleRad:allocation.placementAngleRad,
				assemblyAdjustmentMm:allocation.assemblyAdjustmentMm,cureBowMm:allocation.cureBowMm,
				machiningErrorMm:allocation.machiningErrorMm,pitchAccumulationMm:allocation.pitchAccumulationMm,
				requiredMarginMm:allocation.requiredMarginMm};
			manufacturing.mountingRequirements.push({holeId:hole.id,componentPath:componentPath,
				componentFingerprint:componentFingerprint,connectorId:name,connectorFrame:connector,
				clearanceReference:clearanceReference,permittedRoutes:permitted.copy(),budget:budget});
			switch HoleClearanceBudgets.choose(budget,permitted) {
				case UsePreCast(_):manufacturing.holeRoutes.push({holeId:hole.id,route:InsertHoleRoute.BeforeCasting});
				case UseTransfer(_),ProposeTransfer(_):manufacturing.holeRoutes.push({holeId:hole.id,route:InsertHoleRoute.TransferAfterCasting});
				case AwaitInputs(_),ChangeDesign(_):
			}
			spec.holes.push(hole);
		}
		return new CastInsert(source.values().copy().setToken("holes",InsertHoles.fromNeutral(spec.holes))
			.setToken("origin",haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(spec.origin))));
	}
}
