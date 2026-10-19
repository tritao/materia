package machinekit.enclosure;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport as R;
import machinekit.component.Solids;
import machinekit.component.CollisionHullFacet;
import machinekit.component.ComponentFacet;
import machinekit.component.BomItem;

/** One through opening in a panel blank: rectangular interface or round extraction port. */
class EnclosureCutPanel extends EnclosurePanel {
  public final openingWidth:Float;
  public final openingHeight:Float;
  public final openingX:Float;
  public final openingZ:Float;
  public final diameter:Float;
  static var cutRecipe:Null<ComponentType>;
  public function new(width:Float,height:Float,construction:String,ow:Float,oh:Float,ox:Float,oz:Float,diameter:Float=0) {
    if(!Math.isFinite(ow+oh+ox+oz+diameter) || diameter<0 ||
      (diameter==0 && (!(ow>0)||!(oh>0))) || (diameter>0 && (ow!=0||oh!=0))) throw "Panel opening needs one valid shape";
    var w=diameter>0?diameter:ow,h=diameter>0?diameter:oh;
    if(Math.abs(ox)+w/2>=width/2 || oz-h/2<=0 || oz+h/2>=height) throw "Panel opening must leave an edge ligament";
    super(width,height,construction);
    openingWidth=ow;openingHeight=oh;openingX=ox;openingZ=oz;this.diameter=diameter;
    var removed=diameter>0?Math.PI*diameter*diameter/4:ow*oh;
    var base=massProperties(),ratio=removed/(width*height),mass=base.mass*(1-ratio);
    declareMass(mass,new Vector(-ox*ratio/(1-ratio),base.centreOfMass.y,(height/2-oz*ratio)/(1-ratio)));
  }
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var t=construction.thicknessMm(),tool:Part;
    if(diameter>0) tool=Part.cylinderAlongY(diameter/2,-t,t,openingX,openingZ);
    else {
      var raw=Part.box(openingWidth,t*2,openingHeight);
      tool=raw.translated(new Vector(openingX,0,openingZ-openingHeight/2));raw.close();
    }
    return Solids.cut(super.geometry(detail),[tool]);
  }
  /** Exact rectangular opening proxy; a round port uses its circumscribed square, matching other plate proxies. */
  override public function facets():Array<ComponentFacet> {
    var result=[for(f in super.facets())if(!Std.isOfType(f,CollisionHullFacet))f];
    var w=diameter>0?diameter:openingWidth,h=diameter>0?diameter:openingHeight,t=construction.thicknessMm()/2;
    var x0=openingX-w/2,x1=openingX+w/2,z0=openingZ-h/2,z1=openingZ+h/2;
    result.push(CollisionHullFacet.fromBoxes([[-width/2,-t,0,x0,t,height],[x1,-t,0,width/2,t,height],
      [x0,-t,0,x1,t,z0],[x0,-t,z1,x1,t,height]]));
    return result;
  }
  override function get_bom():BomItem {
    var item=super.get_bom();
    var data:machinekit.component.BomItem.EnclosurePanelBom=item.enclosurePanel;
    var removed=diameter>0?Math.PI*diameter*diameter/4:openingWidth*openingHeight;
    data.areaM2=(width*height-removed)*1e-6;
    return item;
  }
  public static function cutRecipeType():ComponentType {
    if(cutRecipe==null)cutRecipe=new ComponentType("machinekit.enclosure.cut-panel",[
      R.length("width",600),R.length("height",800),R.choice("construction",[for(p in PanelCatalogue.all())p.id],"steel-2"),
      R.length("openingWidth",100),R.length("openingHeight",100),R.quantity("openingX",0,"mm",null),R.length("openingZ",400),R.length("diameter",0)
    ],v->new EnclosureCutPanel(v.number("width"),v.number("height"),v.token("construction"),v.number("openingWidth"),v.number("openingHeight"),v.number("openingX"),v.number("openingZ"),v.number("diameter")),false,false,null,false);
    return cutRecipe;
  }
  override public function componentType():Null<ComponentType> return cutRecipeType();
  override public function values():ComponentValues return super.values().setNumber("openingWidth",openingWidth).setNumber("openingHeight",openingHeight)
    .setNumber("openingX",openingX).setNumber("openingZ",openingZ).setNumber("diameter",diameter);
}
