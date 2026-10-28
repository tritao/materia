package machinekit.structural;

import machinekit.tooling.InsertAnchorGeometry;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import materia.casting.CastingPatterns;
import materia.casting.CastingFeature;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

typedef CastInsertPlacement = {
  final id:String;
  final part:CastInsert;
  final pose:AssemblyFrame;
  final openings:Array<CastingOpening>;
}

typedef PostCureInsertPlacement = {
  final id:String;
  final part:CastInsert;
  final pose:AssemblyFrame;
}

/** A casting and its separately itemised steel inserts, with matching recesses.
 * Layouts belong to the machine using this assembly, not to the component core.
 */
class MineralFoundation extends MachineAssembly {
  public final body:MineralCasting;

  final inserts:Array<CastInsertPlacement>;
  final bonded:Array<PostCureInsertPlacement> = [];
  // Components declared by this source as permanent during the pour. Parts
  // added later (such as removable levelling plates) belong to service assembly.
  final pouredMembers:Array<String> = ["casting"];

  public function new(
    bodyValues:ComponentValues,
    placements:Array<CastInsertPlacement>,
    ?rootInsert:String,
    ?postCure:Array<PostCureInsertPlacement>
  ) {
    super();
    if (placements == null) throw "Foundation needs an insert list";
    inserts = [];
    var v = machinekit.component.ComponentRecipeSupport.resolve(MineralCasting.parameters(), bodyValues);
    var features = CastingPatterns.decode(v.token("features"));
    var ids:Map<String, Bool> = ["casting" => true];
    for (row in placements) {
      if (row == null || row.id == null || row.id.length == 0
        || ids.exists(row.id) || row.part == null) throw "Foundation inserts need unique ids and steel parts";
      ids.set(row.id, true);
      if (row.openings == null) throw "Foundation insert requires declared recess openings: " + row.id;
      var recess:CastingFeature = {id: 'insert-${row.id}', kind: InsertRecess, geometry: {
        solid: materia.casting.CastingFeature.CastingGeometrySolid.Insert(row.part.bodySpec()),
        pose: F.compose(F.identity(), row.pose)
      }, openings: [for (opening in row.openings) {
        kind:opening.kind, target:opening.target
      }
      ], draft: {degrees: 0, axis: {x: 0, y: 0, z: 1}}, cornerRadius:0, owner:row.id, adjustable:[]};
      // Validate references with the complete feature set, including caller features.
      var copy = recess;
      features.push(copy);
      inserts.push({id: row.id, part: row.part, pose: copy.geometry.pose, openings: copy.openings});
    }
    if (rootInsert != null
      && (rootInsert == "casting" || !ids.exists(rootInsert))) throw "Unknown foundation root insert";
    var embedded:Array<materia.casting.InsertAnchorage.EmbeddedInsertAnchors> = [];
    for (row in inserts) {
      var spec = row.part.insertSpec();
      switch spec.anchorage {
        case Bolted(_, _):
          embedded.push({owner: row.id, pose: row.pose, spec: spec});
        default:
      }
    }
    if (materia.casting.InsertAnchorage.EmbeddedAnchors.snapshot(haxe.Json.parse(v.token("embeddedAnchors"))).length
      > 0) throw "Foundation anchor occupancy is derived from its source inserts, not authored twice";
    v.setToken(
      "embeddedAnchors",
      haxe.Json.stringify(materia.casting.InsertAnchorage.EmbeddedAnchors.records(embedded))
    );
    v.setToken("features", CastingPatterns.encode(features));
    body = new MineralCasting(v);
    validateDatums();
    validateHoleExits();
    addComponent("casting", body);
    for (row in inserts) {
      addComponent(row.id, row.part, row.pose);
      pouredMembers.push(row.id);
      if (row.id == rootInsert) {
        addMemberConnector(row.id, "castingSeat", F.compose(F.inverse(row.pose), body.connector("mount").frame));
        addMate("casting-insert", "fixed", row.id, "castingSeat", "casting", "mount");
      } else {
        addMemberConnector("casting", row.id + "Seat", F.compose(row.pose, row.part.connector("mount").frame));
        addMate(row.id + "-cast-in", "fixed", "casting", row.id + "Seat", row.id, "mount");
      }
      var anchors:Array<InsertAnchorGeometry.InsertAnchorComponent>;
      try anchors = InsertAnchorGeometry.components(row.part.insertSpec()) catch (error:Dynamic) throw "Insert "
        + row.id + ": " + Std.string(error);
      for (hardware in anchors) {
        var id = "anchor-" + haxe.crypto.Sha256.encode(row.id + "/" + hardware.anchorId).substr(0,
          20) + "-" + hardware.kind;
        if (ids.exists(id)) throw "Duplicate foundation anchor member: " + id;
        ids.set(id, true);
        addComponent(id, hardware.part, F.compose(row.pose, hardware.pose));
        pouredMembers.push(id);
        addMemberConnector(row.id, id + "Seat", hardware.pose);
        addMemberConnector(id, "castAnchorSeat", F.identity());
        addMate(id + "-cast-in", "fixed", row.id, id + "Seat", id, "castAnchorSeat");
      }
      exposeConnector(row.id, row.id, "top");
    }
    var bonds = body.postCureBonds();
    var service = postCure == null ? [] : postCure;
    if (bonds.length != service.length) throw "Post-cure receivers and insert placements must agree";
    for (row in service) {
      if (row == null || row.part == null || ids.exists(row.id)) throw "Duplicate or invalid post-cure insert";
      ids.set(row.id, true);
      var matches = bonds.filter(bond -> bond.insertId == row.id);
      if (matches.length != 1) throw "Missing post-cure receiver: " + row.id;
      validateBond(row, matches[0]);
      bonded.push({id: row.id, part: row.part, pose: F.compose(F.identity(), row.pose)});
      addComponent(row.id, row.part, row.pose);
      addMemberConnector("casting", row.id + "Bond", row.pose);
      addMate(row.id + "-bonded-after-cure", "fixed", "casting", row.id + "Bond", row.id, "mount");
      exposeConnector(row.id, row.id, "top");
    }
    if (rootInsert != null) exposeConnector("mount", rootInsert, "mount");
    else exposeConnector("mount", "casting", "mount");
  }

  function validateBond(row:PostCureInsertPlacement, bond:materia.casting.PostCureBond):Void {
    var spec = row.part.insertSpec();
    var diameter:Float, height:Float;
    switch spec.body {
      case Primitive(Cylinder(d, h)):
        diameter = d;
        height = h;
      default:
        throw "Bonded receiver requires a cylindrical insert: " + row.id;
    }
    var hole = bond.receiver;
    var top = F.transformPoint(F.compose(F.inverse(hole.frame), row.pose), 0, 0, height);
    var bottom = F.transformPoint(F.compose(F.inverse(hole.frame), row.pose), 0, 0, 0);
    if (Math.abs(top.x) + Math.abs(top.y) + Math.abs(bottom.x) + Math.abs(bottom.y) > 1e-7
      || top.z > 0 || bottom.z <= 0 || bottom.z >= hole.depth
      || diameter >= hole.diameter) throw "Bonded insert does not fit its declared receiver: " + row.id;
    var scope = new cadkit.modeling.Scope();
    try {
      var raw = new MineralCasting(body.values().copy().setToken(
        "buildStage",
        materia.casting.InsertStages.InsertStage.BeforeCasting
      ));
      var casting = scope.own(raw.geometry());
      var tool = scope.own(CastingCavityCad.receiver(hole));
      var common = scope.own(casting.intersect(tool));
      if (Math.abs(common.volume() - tool.volume())
        > 1e-6) throw "Post-cure receiver breaks into a cavity or outside the casting: " + row.id;
      // The declared back-wall rule is physical source geometry, not evidence.
      var back = materia.casting.InsertHole.InsertHoleCodec.snapshot([hole])[0];
      back.depth += hole.limits.minBack;
      var backTool = scope.own(CastingCavityCad.receiver(back));
      var backCommon = scope.own(casting.intersect(backTool));
      if (Math.abs(backCommon.volume() - backTool.volume())
        > 1e-6) throw "Post-cure receiver lacks its declared back wall: " + row.id;
      var wall = materia.casting.InsertHole.InsertHoleCodec.snapshot([hole])[0];
      wall.diameter += 2 * hole.limits.minWall;
      var wallTool = scope.own(CastingCavityCad.receiver(wall));
      var wallCommon = scope.own(casting.intersect(wallTool));
      if (Math.abs(wallCommon.volume() - wallTool.volume())
        > 1e-6) throw "Post-cure receiver lacks its declared radial wall: " + row.id;
      scope.close();
    } catch (error:Dynamic) {
      scope.close();
      throw error;
    }
  }

  public function postCureLocations():Array<PostCureInsertPlacement> return [for (row in bonded) {
    id:row.id, part:row.part, pose:F.compose(F.identity(), row.pose)
  }
  ];

  function validateDatums():Void {
    var scheme = body.foundationDatumScheme(), interfaces = body.precisionRequirements();
    if (scheme == null) {
      if (interfaces.length > 0
        || inserts.filter(insert -> insert.part.insertSpec().interfaces.length > 0).length > 0) diagnostics.warning(
          "casting.datum.missing",
          "casting",
          "Precision interfaces have no source datum scheme."
        );
      return;
    }
    var unresolved = materia.casting.FoundationDatumScheme.FoundationDatums.validateSource(scheme,
      body.castingEnvelope(), body.castingFeatures(), [for (insert in inserts) {
      var specification = insert.part.insertSpec();
      {
        id:insert.id, pose:insert.pose, holes:specification.holes, interfaces:specification.interfaces
      };
    }
    ], interfaces);
    for (datum in unresolved) diagnostics.warning(
      "casting.datum.unsupported",
      datum.datumId,
      datum.reason + ": " + datum.featureId
    );
  }

  /** A through thread needs an accessible exit, never a closure buried in resin.
	 * The actual finished cavities are authoritative; the insert's own recess is
	 * excluded because a point on its boundary cannot establish access behind it.
	 */
  function validateHoleExits():Void {
    var cavities = body.finishedCavities();
    for (row in inserts) {
      var specification = row.part.insertSpec();
      var before = materia.casting.InsertStages.holes(
        specification,
        materia.casting.InsertStages.InsertStage.BeforeCasting
      );
      for (source in specification.holes) {
        var physical = before.filter(hole -> hole.id == source.id);
        var hole = physical.length == 1 ? physical[0] : source;
        if (hole.extent != Through || hole.thread == null) continue;
        var frame = F.compose(row.pose, hole.frame);
        var exit = F.compose(frame, F.translation(0, 0, hole.depth));
        var thread:materia.casting.InsertHole.InsertThread = cast hole.thread;
        if (materia.casting.CastingHoleExits.intersectsMaterial(
          body.castingEnvelope(),
          cavities,
          body.castingFeatures(),
          exit,
          thread.majorDiameter / 2,
          row.id
        )) throw "Through insert thread exits into casting material: " + row.id + "/" + hole.id;
      }
    }
  }

  /** Physical load-path occupancy in foundation coordinates, before finishing.
	 * Keep resin-only geometry separate for mix quantities, fill and section models.
	 * Permanent steel and anchors occupy material; temporary tooling does not.
	 */
  public function asCastCompositeGeometry():cadkit.modeling.Part {
    var poses = solvedPoses(), members = components(), ids = pouredMembers.copy();
    return machinekit.component.Solids.building([], owned -> {for (member in members) if (ids.indexOf(member.id) >= 0) {
      var part = member.component;
      if (Std.isOfType(part, MineralCasting)) part = new MineralCasting(part.values().copy().setToken(
        "buildStage",
        materia.casting.InsertStages.InsertStage.BeforeCasting
      ));
      else if (Std.isOfType(part, CastInsert)) part = new CastInsert(part.values().copy().setToken(
        "buildStage",
        materia.casting.InsertStages.InsertStage.BeforeCasting
      ));
      var local = part.geometry();
      var placed:cadkit.modeling.Part;
      try placed = machinekit.assembly.PosedParts.moved(local, poses.get(member.id)) catch (error:Dynamic) {
        local.close();
        throw error;
      }
      local.close();
      owned.push(placed);
      placed.shape.setThreadedFaces([]);
    }
    return machinekit.component.Solids.union(owned);
    });
  }

  public function castingMemberIds():Array<String> return pouredMembers.copy();

  public function insertLocations():Array<CastInsertPlacement> return [for (row in inserts) {
    id:row.id, part:row.part, pose:F.compose(F.identity(), row.pose), openings:[for (opening in row.openings) {
      kind:opening.kind, target:opening.target
    }
    ]
  }
  ];

  public function insertRecessLinks():Array<{recessId:String,
    insertId:String}> return [for (feature in body.castingFeatures()) if (feature.kind == InsertRecess) {
    recessId:feature.id, insertId:feature.owner
  }
  ];

  public function castingByWeight(): {material: String, kg: Float} return body.castingByWeight();
}
