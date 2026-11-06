package machinekit.structural;

import machinekit.tooling.InsertAnchorGeometry;
import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.structural.CastingCollisionQualification.CastingCollisionStatus;
import materia.casting.CastingFeature;
import materia.casting.CastingPrism;
import materia.casting.CastingPatterns;
import materia.casting.CastingEnvelope;
import materia.casting.CastingCavities;
import materia.casting.BoxAlgebra;

/** Source design cuts and openings; finished cavities have one neutral derivation. */
class MineralCasting extends MachineComponent {
  public final collisionHullQualification:CastingCollisionQualification;
  final features:Array<CastingFeature>;
  final envelope:Array<CastingPrism>;
  static final recipe = new machinekit.cache.Once<ComponentType>();

  public static function parameters():Array<ComponentParameter> return [
    P.length("width", 650),
    P.length("depth", 550),
    P.length("height", 120),
    P.length("cornerChamfer", 0),
    P.text("features", CastingPatterns.encode([])),
    P.choice("buildStage", materia.casting.InsertStages.tokens(), materia.casting.InsertStages.InsertStage.Service),
    P.text("postCureBonds", "[]"),
    P.text("embeddedAnchors", "[]"),
    P.text("datumScheme", "null"),
    P.text("interfaces", "[]")
  ];

  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType(
      "machinekit.structural.mineral-casting",
      parameters(),
      v -> new MineralCasting(v),
      false,
      false,
      null,
      false
    ).numbered(v -> 'MINERAL-${Dimension.format(v.number("width"))}x${Dimension.format(v.number("depth"))}x${Dimension.format(v.number("height"))}')
      .procured(_ -> Make("mineral casting")));
  }

  public function new(?overrides:ComponentValues) {
    var v = P.resolve(parameters(), overrides), cuts = CastingPatterns.decode(v.token("features"));
    v.setToken("features", CastingPatterns.encode(cuts));
    v.setToken(
      "postCureBonds",
      haxe.Json.stringify(materia.casting.PostCureBond.PostCureBonds
      .records(materia.casting.PostCureBond.PostCureBonds.snapshot(haxe.Json.parse(v.token("postCureBonds")))))
    );
    v.setToken(
      "embeddedAnchors",
      haxe.Json.stringify(materia.casting.InsertAnchorage.EmbeddedAnchors
      .records(materia.casting.InsertAnchorage.EmbeddedAnchors.snapshot(haxe.Json.parse(v.token("embeddedAnchors")))))
    );
    var datumRaw:Dynamic = haxe.Json.parse(v.token("datumScheme"));
    if (datumRaw != null) datumRaw = materia.casting.FoundationDatumScheme
      .FoundationDatums.record(materia.casting.FoundationDatumScheme.FoundationDatums.snapshot(datumRaw));
    v.setToken("datumScheme", haxe.Json.stringify(datumRaw));
    var interfaces =
      materia.casting.InterfaceRequirement.InterfaceRequirements.snapshot(haxe.Json.parse(v.token("interfaces")));
    v.setToken(
      "interfaces",
      haxe.Json.stringify(materia.casting.InterfaceRequirement.InterfaceRequirements.records(interfaces))
    );
    for (name in ["width", "depth", "height"]) if (v.number(name) <= 0) throw "Casting dimensions must be positive";
    if (v.get("material") != null && v.token("material") != "mineral cast") throw "Casting requires mineral cast";
    if (v.number("cornerChamfer") < 0 || 2 * v.number("cornerChamfer") >= Math.min(v.number("width"),
      v.number("depth"))) throw "Casting chamfer consumes body";
    super("MINERAL-CASTING", "Mineral casting (purchase by casting mass, kg)", "mineral cast", false, v);
    addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    features = cuts;
    envelope = CastingEnvelope.build(
      v.number("width"),
      v.number("depth"),
      v.number("height"),
      v.number("cornerChamfer")
    );
    var cavities = finishedCavities(), unsupported = cavities.filter(cavity -> !CastingCavities.supportsBoxes(cavity));
    if ((postCureBonds().length == 0
      || parameterToken("buildStage") == materia.casting.InsertStages.InsertStage.BeforeCasting)
      && parameterNumber("cornerChamfer") == 0 && unsupported.length == 0 && embeddedAnchors().length == 0) {
      var material = materialBoxes();
      addFacet(CollisionHullFacet.fromBoxes([for (box in BoxAlgebra.compact(material)) BoxAlgebra.limits(box)]));
      collisionHullQualification = new CastingCollisionQualification(
        CastingCollisionStatus.Supported,
        "Exact disjoint-box material hulls"
      );
    } else {
      collisionHullQualification = new CastingCollisionQualification(
        CastingCollisionStatus.Unsupported,
        postCureBonds().length > 0 ? "Post-cure receiver bores require non-box collision decomposition: "
        + [for (bond in postCureBonds()) bond.insertId].join(", ") : embeddedAnchors().length
        > 0 ? "Non-box anchor occupancy for inserts: " + [for (row in embeddedAnchors()) row.owner].join(", ")
        : unsupported.length == 0 ? "Chamfered envelopes require non-box collision decomposition"
        : "Unsupported collision geometry for features: " + [for (cavity in unsupported) cavity.featureId].join(", ")
      );
    }
    addFacet(collisionHullQualification);
    if (interfaces.length > 0) addFacet(new machinekit.design.DesignInterfaces([], [], [], interfaces));
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("top", Mount, Solids.axial(0, 0, v.number("height")));
  }

  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;

  override public function setMaterial(spec:String):Void {
    if (spec != "mineral cast") throw "Casting requires mineral cast";
    super.setMaterial(spec);
  }

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var parts:Array<Part> = [];
    try {
      for (p in castingEnvelope()) parts.push(CastingCavityCad.prism(p));
      var body = Solids.union(parts);
      parts = [];
      return Solids.building([body], owned -> {for (cavity in finishedCavities()) owned.push(Solids.named(
        CastingCavityCad.part(cavity),
        cavity.featureId
      ));
      for (part in InsertAnchorGeometry.embeddedParts(embeddedAnchors())) owned.push(part);
      if (parameterToken("buildStage") == materia.casting.InsertStages
        .InsertStage.Service) for (bond in postCureBonds()) owned.push(CastingCavityCad.receiver(bond.receiver));
      return Solids.cut(owned[0], owned.slice(1));
      });
    } catch (error:Dynamic) {
      for (part in parts) part.close();
      throw error;
    }
  }

  /**
	 * The box cavities taken from the envelope's disjoint material boxes in closed form, less the solids box algebra
	 * cannot hold: the non-box cavities (realized and validated as `geometry` realizes them), the embedded anchor
	 * hardware and, in service, the post-cure receivers (`BoxMaterialVolume`). Null for a chamfered envelope or when
	 * those solids overlap one another, and the solid is integrated instead.
	 */
  override public function analyticVolume():Null<machinekit.component.VolumeProperties> {
    var envelope = castingEnvelope();
    for (prism in envelope) if (!CastingCavities.supportsBoxes({
      featureId: prism.id,
      geometry: CastingCavities.FinishedCavityGeometry.Prisms([prism])
    })) return null;
    var cavities = finishedCavities();
    var boxes = materialLess(envelope, cavities.filter(cavity -> CastingCavities.supportsBoxes(cavity)));
    // Each solid is named by its inputs and built only when the integral has not met it before.
    var tools:Array<BoxMaterialVolume.KeyedSolid> = [];
    for (cavity in cavities) if (!CastingCavities.supportsBoxes(cavity)) tools.push({
      key: CastingCavityCad.key(cavity),
      build: () -> CastingCavityCad.part(cavity)
    });
    var screws = new machinekit.assembly.PosedParts();
    try {
      for (solid in InsertAnchorGeometry.embeddedSolids(embeddedAnchors(), screws)) tools.push(solid);
      if (parameterToken("buildStage") == materia.casting.InsertStages
        .InsertStage.Service) for (bond in postCureBonds()) tools.push({
          key: "receiver|" + haxe.Json.stringify(bond.receiver),
          build: () -> CastingCavityCad.receiver(bond.receiver)
        });
      var result = BoxMaterialVolume.ofKeyed(boxes, tools);
      screws.close();
      return result;
    } catch (error:Dynamic) {
      screws.close();
      throw error;
    }
  }

  public function asCastGeometry():Part return new MineralCasting(values().copy().setToken(
    "buildStage",
    materia.casting.InsertStages.InsertStage.BeforeCasting
  )).geometry();

  public function postCureBonds():Array<materia.casting.PostCureBond
    > return materia.casting.PostCureBond.PostCureBonds.snapshot(haxe.Json.parse(parameterToken("postCureBonds")));

  public function embeddedAnchors():Array<materia.casting.InsertAnchorage.EmbeddedInsertAnchors> return materia
    .casting.InsertAnchorage.EmbeddedAnchors.snapshot(haxe.Json.parse(parameterToken("embeddedAnchors")));

  public function foundationDatumScheme():Null<materia.casting.FoundationDatumScheme> {
    var raw:Dynamic = haxe.Json.parse(parameterToken("datumScheme"));
    return raw == null ? null : materia.casting.FoundationDatumScheme.FoundationDatums.snapshot(raw);
  }

  public function precisionRequirements():Array<materia.casting.InterfaceRequirement> {
    var interfaces = machinekit.design.DesignInterfaces.of(this);
    return interfaces == null ? [] : interfaces.precisionRequirements();
  }

  public function castingFeatures():Array<CastingFeature> return materia.casting.CastingFeatureCodec.copy(features);

  public function castingEnvelope():Array<CastingPrism> return [for (prism in envelope) {
    id:prism.id, profile:[for (point in prism.profile) {
      x:point.x, y:point.y
    }
    ], height:prism.height, pose:materia.casting.CastingRecordCodec.frame(prism.pose)
  }
  ];

  public function finishedCavities():Array<CastingCavities.FinishedCastingCavity> return CastingCavities.derive(
    castingEnvelope(),
    features
  );

  public function materialBoxes():Array<materia.casting.Box3> {
    if (postCureBonds().length > 0 && parameterToken("buildStage")
      == materia.casting.InsertStages.InsertStage.Service) throw "Unsupported non-box post-cure receivers: "
      + [for (bond in postCureBonds()) bond.insertId].join(", ");
    var embedded = embeddedAnchors();
    if (embedded.length
      > 0) throw "Unsupported non-box anchor material for inserts: " + [for (row in embedded) row.owner].join(", ");
    return materialLess(castingEnvelope(), finishedCavities());
  }

  /**
   * `BoxAlgebra.materialLess`, by its inputs: sizing integrates many layouts of one envelope whose box cavities repeat. The
   * key is the envelope and the cavities' boxes (whose validation `materialLess` repeats on a miss).
   */
  static function materialLess(envelope:Array<CastingPrism>,
    cavities:Array<CastingCavities.FinishedCastingCavity>):Array<materia.casting.Box3> {
    var cuts = [for (c in cavities) for (b in CastingCavities.boxes(c)) b];
    return materials.get(haxe.Json.stringify(envelope) + "|" + haxe.Json.stringify(cuts), () -> BoxAlgebra.materialLess(
      envelope,
      cavities
    ));
  }

  static final materials = new machinekit.cache.Memo<Array<materia.casting.Box3>>("casting-material-boxes", 512, copyBoxes);

  static function copyBoxes(boxes:Array<materia.casting.Box3>):Array<materia.casting.Box3> {
    var copies:Array<materia.casting.Box3> = [];
    for (box in boxes) copies.push({
      x: box.x,
      y: box.y,
      z: box.z,
      width: box.width,
      depth: box.depth,
      height: box.height
    });
    return copies;
  }

  public function minimumSectionIy(x0:Float, x1:Float):Float return BoxAlgebra.minimumIy(materialBoxes(), x0, x1);

  public function castingByWeight(): {material: String, kg: Float} return {
    material: materialSpec(),
    kg: new MineralCasting(values().copy().setToken(
      "buildStage",
      materia.casting.InsertStages.InsertStage.BeforeCasting
    )).massProperties().mass
  };
}
