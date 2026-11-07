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
  public static function derive(kind:String, rows:Array<CastInsertPlacement>):Array<CastInsertPlacement> {
    var result = rows.copy();
    for (placed in HobberRailPlacement.placements(kind, rows)) {
      var indices = [for (index in 0...result.length) if (result[index].id == placed.insertId) index];
      if (indices.length != 1) throw "Placed bought part needs one source seat: " + placed.insertId;
      var index = indices[0], row = result[index], clearance = placed.rail.mountingHole();
      var toInsert = F.compose(F.inverse(F.compose(FoundationLayout.worldPose(kind), row.pose)), placed.pose);
      var part = InsertMountPattern.derive(
        row.part,
        placed.rail,
        "hobber-" + kind + "/" + placed.memberId,
        HobberRailPlacement.fingerprint(placed),
        toInsert,
        [for (connector in placed.rail.mountingConnectors()) connector.name],
        placed.rail.mountingThread(),
        clearance == null ? null : clearance.diameter,
        clearance == null ? null : clearance.reference,
        [BeforeCasting, TransferAfterCasting],
        {
          placementOffsetMm : Unresolved("RM0 insert placement offset"),
          placementAngleRad : Unresolved("RM0 insert angular placement"),
          assemblyAdjustmentMm : Unresolved("RM0 assembly lateral adjustment"),
          cureBowMm : Unresolved("RM0 cure bow"),
          machiningErrorMm : Unresolved("RM0 drill and guide accuracy"),
          pitchAccumulationMm : Unresolved("RM0 mounting pitch accumulation"),
          requiredMarginMm : Unresolved("RM0 required clearance margin")
        }
      );
      result[index] = {id: row.id, pose: row.pose, openings: row.openings, part: addFixture(row.id, part)};
    }
    // The drive pad bar's screws: the pad declares its pattern (`HobberPadBolts`), the cast insert under it is tapped to suit.
    for (index in 0...result.length) if (result[index].id == "driveSeat") {
      var row = result[index];
      result[index] = {id: row.id, pose: row.pose, openings: row.openings, part: addPadHoles(kind, row)};
    }
    if (kind == "column") for (pad in FoundationLayout.columnSolePads())
      for (index in 0...result.length) if (result[index].id == pad.id) {
        var row=result[index];
        result[index]={id:row.id,pose:row.pose,openings:row.openings,part:addSoleHoles(row,pad)};
      }
    return result;
  }

  /** Reserve the sole joint's transferred service threads before choosing
   * cast anchors. The clamped face owns the pattern; no second hole list. */
  static function addSoleHoles(row:CastInsertPlacement,pad:FoundationLayout.ColumnSolePad):CastInsert {
    var face=HobberSaddle.columnSoleFace(pad);
    var size=switch face.role {
      case Clamped(thread,_,_,_): if (thread==null) throw "Sole joint needs a selected thread"; thread;
      case _: throw "Sole source needs its clamped joint face";
    };
    var choice=InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(Std.parseFloat(size.substr(1))).mating());
    var part=row.part;
    var spec=part.insertSpec();
    var thickness=part.boxDimensions(row.id).height;
    var toInsert=F.compose(F.inverse(F.compose(FoundationLayout.worldPose("column"),row.pose)),face.frame);
    var n=0;
    for (point in face.holes) {
      var q=F.transformPoint(toInsert,point.x,point.y,0);
      if (Math.abs(q.z-thickness)>1e-6) throw "Sole joint source drilling misses insert top: "+row.id;
      var hole=InsertHoles.records(InsertHoles.tappedBlind(choice,thickness))[0];
      hole.id="soleJoint"+(++n);
      hole.frame.x=q.x;hole.frame.y=q.y;
      spec.holes.push(hole);
      switch spec.origin {
        case Fabricated(manufacturing): manufacturing.holeRoutes.push({holeId:hole.id,route:TransferAfterCasting});
        case Catalogue(_): throw "Sole source must be a made insert";
      }
    }
    return new CastInsert(part.values().copy().setToken("holes",InsertHoles.fromNeutral(spec.holes))
      .setToken("origin",haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(spec.origin))));
  }

  /** The tapped holes the drive pad's screws thread into: blind, in the insert's top face, before casting like the fixture thread. */
  static function addPadHoles(kind:String, row:CastInsertPlacement):CastInsert {
    var d = kind == "bed" ? HobberDrives.x() : HobberDrives.z(), part = row.part, spec = part.insertSpec();
    var toInsert = F.inverse(F.compose(FoundationLayout.worldPose(kind), row.pose)), thickness = part.boxDimensions(row.id).height;
    var choice = InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(Std.parseFloat(HobberPadBolts.thread(d).substr(1))).mating());
    var n = 0;
    for (p in HobberPadBolts.points(d)) {
      var q = F.transformPoint(toInsert, p.x, p.y, p.z);
      if (Math.abs(q.z - thickness) > 1e-6) throw "A drive pad screw is not on the top face of its insert: " + kind + "/" + row.id + " z " + q.z + " of " + thickness;
      var hole = InsertHoles.records(InsertHoles.tappedBlind(choice, thickness))[0];
      hole.id = "pad" + (++n);
      hole.frame.x = q.x;
      hole.frame.y = q.y;
      spec.holes.push(hole);
      switch spec.origin {
        case Fabricated(manufacturing):
          manufacturing.holeRoutes.push({holeId: hole.id, route: BeforeCasting});
        case Catalogue(_):
          throw "A catalogue insert cannot acquire the drive pad's threads: " + row.id;
      }
    }
    return new CastInsert(part.values().copy().setToken("holes", InsertHoles.fromNeutral(spec.holes)).setToken(
      "origin",
      haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(spec.origin))
    ));
  }

  static function addFixture(id:String, part:CastInsert):CastInsert {
    var spec = part.insertSpec(), stations = [for (hole in spec.holes) hole.frame.y];
    stations.sort(Reflect.compare);
    var gaps:Array<Float> = [];
    for (index in 1...stations.length) if (stations[index] - stations[index - 1]
      > 1e-7) gaps.push((stations[index] + stations[index - 1]) / 2);
    gaps.sort((a, b) -> {var distance = Reflect.compare(Math.abs(a), Math.abs(b));
    return distance == 0 ? Reflect.compare(a, b) : distance;
    });
    if (gaps.length == 0) throw "No inter-station land for the declared fixture thread: " + id;
    var thickness = part.boxDimensions(id).height;
    var fixture = InsertHoles.records(FoundationLayout.fixtureHoles(thickness))[0];
    fixture.frame.y = gaps[0];
    spec.holes.push(fixture);
    switch spec.origin {
      case Fabricated(manufacturing):
        manufacturing.holeRoutes.push({holeId: fixture.id, route: BeforeCasting});
      case Catalogue(_):
        throw "A catalogue insert cannot acquire a fabricated fixture thread: " + id;
    }
    return new CastInsert(part.values().copy().setToken("holes", InsertHoles.fromNeutral(spec.holes)).setToken(
      "origin",
      haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(spec.origin))
    ));
  }
}
