package machinekit.enclosure;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.BomItem;
import machinekit.component.Solids;
import materia.project.MaterialLibrary;

/** Rectangular panel: width along X, layer stack along Y, height from Z=0. */
class EnclosurePanel extends MachineComponent {
  public final width:Float;
  public final height:Float;
  public final construction:LaminatedPanel;
  static var recipe:Null<ComponentType>;

  public function new(width:Float, height:Float, constructionId:String = "fr-acm-4") {
    if (!(width > 0) || !(height > 0) || !Math.isFinite(width + height))
      throw "Enclosure panel needs finite positive dimensions";
    var panel = PanelCatalogue.require(constructionId);
    super('ENCLOSURE-PANEL-$constructionId-${width}x$height', "Enclosure infill panel",
      MaterialLibrary.require(panel.layers[0].materialId).physical.spec);
    this.width = width;
    this.height = height;
    construction = panel;
    var thickness = panel.thicknessMm(), cursor = -thickness / 2, moment = 0.0;
    for (layer in panel.layers) {
      var mass = MaterialLibrary.require(layer.materialId).physical.density * layer.thicknessMm / 1000;
      moment += mass * (cursor + layer.thicknessMm / 2);
      cursor += layer.thicknessMm;
    }
    declareMass(width * height * 1e-6 * panel.arealMassKgM2(), new Vector(0, moment / panel.arealMassKgM2(), height / 2));
    addFacet(machinekit.component.CollisionHullFacet.fromBoxes([[-width/2,-thickness/2,0,width/2,thickness/2,height]]));
    addConnector("mount", Mount, Solids.axial(0, 0, 0));
    addConnector("top", Face, Solids.axial(0, 0, height));
  }

  override public function setMaterial(spec:String):Void {
    if (spec != materialSpec()) throw "Select a panel construction to change its material stack";
  }
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part
    return Solids.named(Part.box(width, construction.thicknessMm(), height), "panel");

  override function get_bom():BomItem {
    var item = super.get_bom();
    item.enclosurePanel = {construction:construction.bomProperties(), widthMm:width, heightMm:height,
      areaM2:width * height * 1e-6, massKg:massProperties().mass};
    return item;
  }

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.enclosure.panel", [
      ComponentRecipeSupport.length("width", 600), ComponentRecipeSupport.length("height", 800),
      ComponentRecipeSupport.choice("construction", [for (panel in PanelCatalogue.all()) panel.id], "fr-acm-4")
    ], v -> new EnclosurePanel(v.number("width"), v.number("height"), v.token("construction")), false, false, null, false);
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues().setNumber("width", width)
    .setNumber("height", height).setToken("construction", construction.id);
}
