package hobber;

import machinekit.hobbing.*;

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
    for (placed in HobberSkeleton.reference().rails(kind, rows)) {
      var indices = [for (index in 0...result.length) if (result[index].id == placed.insertId) index];
      if (indices.length != 1) throw "Placed bought part needs one source seat: " + placed.insertId;
      var index = indices[0], row = result[index], clearance = placed.rail.mountingHole();
      var toInsert = F.compose(F.inverse(F.compose(HobberSkeleton.reference().foundationDatum(kind), row.pose)), placed.pose);
      var part = InsertMountPattern.derive(
        row.part,
        placed.rail,
        "hobber-" + kind + "/" + placed.memberId,
        HobberSkeleton.reference().railFingerprint(placed),
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
      result[index] = {id: row.id, purpose:row.purpose, pose: row.pose, openings: row.openings, part: addFixture(row.id, part)};
    }
    if(kind=="column")for(placed in HobberSkeleton.reference().rails(kind,rows))for(index in 0...result.length)if(result[index].id==placed.insertId) {
      var row=result[index];
      result[index]={id:row.id,purpose:row.purpose,pose:row.pose,openings:row.openings,part:addBellowsHoles(row,placed.contact.y<0?-1:1)};
    }
    // The drive pad bar's screws: the pad declares its pattern (`HobberPadBolts`), the cast insert under it is tapped to suit.
    for (index in 0...result.length) if (result[index].id == "driveSeat") {
      var row = result[index];
      result[index] = {id: row.id, purpose:row.purpose, pose: row.pose, openings: row.openings, part: addPadHoles(kind, row)};
    }
    if (kind == "column") for (pad in HobberSkeleton.reference().columnSolePads())
      for (index in 0...result.length) if (result[index].id == pad.id) {
        var row=result[index];
        result[index]={id:row.id,purpose:row.purpose,pose:row.pose,openings:row.openings,part:addSoleHoles(row,pad)};
      }
    return result;
  }

  /** The made upper bracket owns its source drilling pattern. Transfer after
   * indicating it on the existing rail insert; no independent hole list. */
  static function addBellowsHoles(row:CastInsertPlacement,side:Int):CastInsert {
    var mount=HobberSkeleton.reference().zBellowsFixedUpperMount(),face=mount.fasteningFaces().filter(f->f.id=="column")[0];
    var toInsert=F.compose(F.inverse(F.compose(HobberSkeleton.reference().foundationDatum("column"),row.pose)),F.compose(HobberSkeleton.reference().zBellowsPose(side,true,true),face.frame));
    var part=row.part,spec=part.insertSpec(),thickness=part.boxDimensions(row.id).height;
    var size=switch face.role {case Clamped(thread,_,_,_):if(thread==null)throw "Bellows bracket needs its selected thread";thread;case _:throw "Bellows source must follow a clamped face";};
    var choice=InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(Std.parseFloat(size.substr(1))).mating()),n=0;
    for(point in face.holes) {
      var hole=InsertHoles.records(InsertHoles.tappedBlind(choice,thickness))[0];
      hole.id="bellowsUpper"+(++n);
      hole.frame=F.compose(F.compose(toInsert,F.translation(point.x,point.y,0)),F.fromRotationMatrix(0,0,0,[1.0,0,0,0,-1,0,0,0,-1]));
      spec.holes.push(hole);
      switch spec.origin {case Fabricated(m):m.holeRoutes.push({holeId:hole.id,route:TransferAfterCasting});case _:throw "Bellows bracket needs a made insert";}
    }
    return new CastInsert(part.values().copy().setToken("holes",InsertHoles.fromNeutral(spec.holes)).setToken("origin",haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(spec.origin))));
  }

  /** Reserve the sole joint's transferred service threads before choosing
   * cast anchors. The clamped face owns the pattern; no second hole list. */
  static function addSoleHoles(row:CastInsertPlacement,pad:HobberSkeleton.ColumnSolePad):CastInsert {
    var face=HobberXCarriage.columnSoleFace(pad);
    var size=switch face.role {
      case Clamped(thread,_,_,_): if (thread==null) throw "Sole joint needs a selected thread"; thread;
      case _: throw "Sole source needs its clamped joint face";
    };
    var choice=InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(Std.parseFloat(size.substr(1))).mating());
    var part=row.part;
    var spec=part.insertSpec();
    var thickness=part.boxDimensions(row.id).height;
    var toInsert=F.compose(F.inverse(F.compose(HobberSkeleton.reference().foundationDatum("column"),row.pose)),face.frame);
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

  /** Source threads follow each short bar's clamped joint, not a second list.
   * Guard feet have their own threads in the insert, off both ground bars. */
  static function addPadHoles(kind:String,row:CastInsertPlacement):CastInsert {
    var d=kind=="bed"?HobberDrives.x():HobberDrives.z(),part=row.part,spec=part.insertSpec();
    var inverse=F.inverse(F.compose(HobberSkeleton.reference().foundationDatum(kind),row.pose)),thickness=part.boxDimensions(row.id).height;
    for(support in ["fixed","floating"]) {
      var choice=InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(Std.parseFloat(HobberPadBolts.thread(d,support).substr(1))).mating()),n=0;
      for(point in HobberPadBolts.points(d,support)) {
        var q=F.transformPoint(inverse,point.x,point.y,point.z);
        if(Math.abs(q.z-thickness)>1e-6)throw "Ground bar misses insert plane";
        var hole=InsertHoles.records(InsertHoles.tappedBlind(choice,thickness))[0];
        hole.id=support+"Bar"+(++n);hole.frame.x=q.x;hole.frame.y=q.y;spec.holes.push(hole);
        switch spec.origin {case Fabricated(m):m.holeRoutes.push({holeId:hole.id,route:BeforeCasting});case _:throw "Ground bar needs made insert";}
      }
    }
    if(kind=="bed")for(face in HobberChipCover.insertReceivers()) {
      var choice=InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(3).mating());
      var q=F.transformPoint(F.compose(inverse,face.frame),0,0,0),hole=InsertHoles.records(InsertHoles.tappedBlind(choice,thickness))[0];
      hole.id=face.id;hole.frame.x=q.x;hole.frame.y=q.y;spec.holes.push(hole);
      switch spec.origin {case Fabricated(m):m.holeRoutes.push({holeId:hole.id,route:BeforeCasting});case _:throw "Guard needs made insert";}
    }
    return new CastInsert(part.values().copy().setToken("holes",InsertHoles.fromNeutral(spec.holes))
      .setToken("origin",haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(spec.origin))));
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
    var fixture = InsertHoles.records(HobberSkeleton.reference().fixtureHoles(thickness))[0];
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
