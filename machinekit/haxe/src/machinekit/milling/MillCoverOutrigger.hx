package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.BoltedFace.ThreadMaterial;

/** A single machined outrigger, clear of the X motor/connector withdrawal corridor. */
class MillCoverOutrigger extends MachineComponent {
  static final recipe = new machinekit.cache.Once<ComponentType>();
  public static function recipeType():ComponentType {
    return recipe.get(() -> new ComponentType("machinekit.milling.cover-outrigger", [
      ComponentRecipeSupport.choice("side", ["Left", "Right"], "Right"),
      ComponentRecipeSupport.choice("row", ["Front", "Rear"], "Rear")
    ], v -> new MillCoverOutrigger(v.token("side") == "Left" ? -1 : 1,
      v.token("row") == "Front" ? -1 : 1), true).numbered(v -> 'MILL-FRAME-COVER-OUTRIGGER-${v.token("side") == "Left" ? -1 : 1}-${v.token("row") == "Front" ? -1 : 1}'));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues()
    .setToken("side", side < 0 ? "Left" : "Right").setToken("row", ySide < 0 ? "Front" : "Rear")
    .setToken("material", materialSpec());
  final side:Int;
  final ySide:Int;
  public function new(side:Int, ySide:Int) {
    if ((side != -1 && side != 1) || (ySide != -1 && ySide != 1)) throw "Outrigger side and row must be -1 or 1";
    super('MILL-COVER-OUTRIGGER-$side-$ySide', "Machined cover outrigger", "steel", true);
    this.side = side;
    this.ySide = ySide;
    declareDrawingNumber('MILL-FRAME-COVER-OUTRIGGER-$side-$ySide');
    addFacet(new machinekit.component.ProcurementIntent(machinekit.component.Procurement.Make("CNC machining")));
    addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues().setToken("ratingsBasis",
      "Stated").setToken(
        "errorsBasis",
        "Assumed"
      ).setToken(
        "source",
        ""
      )));
  }
  override public function revisionFingerprint():Null<String> return machinekit.component.BomRevision.fingerprint(
    '$side|$ySide|195x24x16|foot14x24x40',
    1
  );
  override public function fasteningFaces():Array<BoltedFace> return [
    BoltedFace.clamped(
      "mount",
      BoltedFace.frameOf({x: 0, y: 0, z: 51}, {x: side, y: 0, z: 0}, {x: 0, y: 1, z: 0}),
      [{x: 0, y: 0}],
      6.6,
      14,
      "M6",
      null,
      false
    ),
    BoltedFace.drilledToSuit(
      "wall",
      BoltedFace.frameOf({x: side * 185, y: ySide * 12, z: 72}, {x: 0, y: -ySide, z: 0}, {x: 1, y: 0, z: 0}),
      20,
      16,
      12,
      24,
      Steel
    )
  ];
  override public function machinable():Bool return true;
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return machined(Solids.union([
    Part.box(195, 24, 16).translated(new Vector(side * 97.5, 0, 64)),
    Part.box(14, 24, 40).translated(new Vector(side * 7, 0, 40))
  ]));
}
