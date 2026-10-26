package machinekit.hobbing;

import machinekit.structural.CastInsert;
import machinekit.structural.InsertHoles;
import machinekit.structural.InsertMountPattern;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertManufacturing.InsertHoleRoute;

/** Purchased-part connectors own mounting-hole positions. A separate source
 * fixture thread lies between those stations, preserving the undecided service
 * drilling route until the accuracy allocation is supplied. */
class HobberMountingHoles {
	public static function derive(kind:String,rows:Array<CastInsertPlacement>):Array<CastInsertPlacement> {
		var result=rows.copy();
		for (placed in HobberRailPlacement.placements(kind,rows)) {
			var indices=[for (index in 0...result.length) if (result[index].id==placed.insertId) index];
			if (indices.length!=1) throw "Placed bought part needs one source seat: "+placed.insertId;
			var index=indices[0],row=result[index],clearance=placed.rail.mountingHole();
			var toInsert=F.compose(F.inverse(F.compose(FoundationLayout.worldPose(kind),row.pose)),placed.pose);
			var part=InsertMountPattern.derive(row.part,placed.rail,"hobber-"+kind+"/"+placed.memberId,
				HobberRailPlacement.fingerprint(placed),toInsert,
				[for (connector in placed.rail.mountingConnectors()) connector.name],placed.rail.mountingThread(),
				clearance==null ? null : clearance.diameter,clearance==null ? null : clearance.reference,
				[BeforeCasting,TransferAfterCasting],{
					placementOffsetMm:Unresolved("RM0 insert placement offset"),
					placementAngleRad:Unresolved("RM0 insert angular placement"),
					assemblyAdjustmentMm:Unresolved("RM0 assembly lateral adjustment"),
					cureBowMm:Unresolved("RM0 cure bow"),
					machiningErrorMm:Unresolved("RM0 drill and guide accuracy"),
					pitchAccumulationMm:Unresolved("RM0 mounting pitch accumulation"),
					requiredMarginMm:Unresolved("RM0 required clearance margin")
				});
			result[index]={id:row.id,pose:row.pose,openings:row.openings,part:addFixture(row.id,part)};
		}
		return result;
	}

	static function addFixture(id:String,part:CastInsert):CastInsert {
		var spec=part.insertSpec(),stations=[for (hole in spec.holes) hole.frame.y];
		stations.sort(Reflect.compare);
		var gaps:Array<Float>=[];
		for (index in 1...stations.length)
			if (stations[index]-stations[index-1]>1e-7) gaps.push((stations[index]+stations[index-1])/2);
		gaps.sort((a,b)->{
			var distance=Reflect.compare(Math.abs(a),Math.abs(b));
			return distance==0 ? Reflect.compare(a,b) : distance;
		});
		if (gaps.length==0) throw "No inter-station land for the declared fixture thread: "+id;
		var thickness=part.boxDimensions(id).height;
		var fixture=InsertHoles.records(FoundationLayout.fixtureHoles(thickness))[0];
		fixture.frame.y=gaps[0];
		spec.holes.push(fixture);
		switch spec.origin {
			case Fabricated(manufacturing):manufacturing.holeRoutes.push({holeId:fixture.id,route:BeforeCasting});
			case Catalogue(_):throw "A catalogue insert cannot acquire a fabricated fixture thread: "+id;
		}
		return new CastInsert(part.values().copy().setToken("holes",InsertHoles.fromNeutral(spec.holes))
			.setToken("origin",haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(spec.origin))));
	}
}
