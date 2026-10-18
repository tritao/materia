package machinekit.hobbing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Separate designed weldments, with machined mounting datums. Dimensions are mm.
    Strength, weld distortion and modal stiffness remain assumed pending Phase D. */
class HobberSaddle extends MachineComponent {
  static var recipe:Null<ComponentType>;
  public static function parameters():Array<ComponentParameter> return [
    P.choice("design",["Z-saddle","X-bridge","A-yoke","Y-saddle","electrical-panel","X-drive","Z-drive","Y-drive"],"Z-saddle")];
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.saddle",parameters(),v->new HobberSaddle(v.token("design")));
    return recipe;
  }
  public function new(design:String){
    var v=P.resolve(parameters(),new ComponentValues().setToken("design",design));
    super("HOBBER-"+design,"Designed steel "+design+"; machined seats, assumed structural adequacy","steel",false,v);
    addConnector("datum",Mount,F.identity());
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  static function box(x0:Float,x1:Float,y0:Float,y1:Float,z0:Float,z1:Float):Part {
    var raw=Part.box(x1-x0,y1-y0,z1-z0),p=raw.translated(new Vector((x0+x1)/2,(y0+y1)/2,z0));raw.close();return p;
  }
  static function boreX(radius:Float,x0:Float,x1:Float,y:Float,z:Float):Part {
    var p=Part.cylinderSpan(radius,x0,x1);
    var q=machinekit.assembly.PosedParts.moved(p,machinekit.assembly.AxisBuilder.orient(0,y,z,[0.0,0,1],[1.0,0,0]));p.close();return q;
  }
  static function boreY(radius:Float,y0:Float,y1:Float,x:Float,z:Float):Part return Part.cylinderAlongY(radius,y0,y1,x,z);
  override public function geometry(detail:ComponentDetail=Preview):Part {
    return switch values().token("design") {
      // Origin at X=0,Y=0,Z=655: rail block tops Y=152, X rail bases Z=655.
      case "Z-saddle": Solids.union([
        Solids.cut(box(-100,240,-65,142,-20,0),[box(-80,180,67,115,-20.1,0.1)]),box(-100,100,142,152,-55,140),
        box(-255,100,142,152,77,100),
        Solids.cut(box(-255,-185,150,215,67.5,80),[Part.cylinderSpan(8.2,67.4,80.1,-220,180)])]);
      // Origin at the cutter centre with X=0. Beam bottom is the X block top.
      case "X-bridge": Solids.union([
        Solids.cut(box(-85,300,98,275,360,420),[box(-75,290,108,265,370,410)]),
        Solids.cut(box(240,300,120,200,0,380),[box(250,290,130,190,10,370)]),
        box(200,220,-90,160,-90,90),box(220,300,120,160,0,20),
        Solids.cut(box(27.5,47.5,205,275,263,360),[boreX(8.2,27.4,47.6,240,287),box(27.4,47.6,204.9,227,307,360.1)])]);
      // Origin at cutter centre; machined back face X=90 meets swivel head.
      case "A-yoke": Solids.union([
        box(70,90,-100,100,0,172),box(-160,90,-120,120,152,172)]);
      // Ceiling below Y rail blocks; bored saddles seat the actual spindle OD.
      case "Y-saddle": Solids.union([
        box(-60,60,-120,240,104,124),
        Solids.cut(box(-55,55,80,100,-55,104),[boreY(45,79.9,100.1,0,0)]),
        Solids.cut(box(-55,55,200,220,-55,104),[boreY(45,199.9,220.1,0,0)]),
        Solids.cut(box(-45,45,-115,-85,-30,104),[box(-30,30,-115.1,-84.9,-30.1,30)]),
        Solids.cut(box(-145,-55,-45,-27.5,52,104),[boreY(8.2,-45.1,-27.4,-110,77)])]);
      case "X-drive": Solids.union([
        box(-100,220,40,140,565,585),box(-140,220,125,142,585,635),
        Solids.cut(box(-130,-125,35,125,565,655),[boreX(7.1,-130.1,-124.9,80,610)])]);
      case "Z-drive": Solids.union([
        box(-260,-180,115,135,390,950),box(-260,-180,135,155,630,655),box(-260,-180,135,155,922.5,942.5),
        box(-260,-110,135,320,390,410),
        Solids.cut(box(-260,-110,135,320,865,885),[Part.cylinderSpan(8.2,864.9,885.1,-220,180)]),
        Solids.cut(box(-265,-175,135,225,560,565),[Part.cylinderSpan(9.6,559.9,565.1,-220,180)])]);
      case "Y-drive": Solids.union([
        box(-160,-150,-325,-30,355,475),box(-150,-70,-115,-80,355,375),box(-150,-70,-325,-280,355,375),
        Solids.cut(box(-155,-65,-35,-30,355,445),[boreY(7.1,-35.1,-29.9,-110,400)])]);
      case "electrical-panel": Solids.union([box(-210,210,-10,0,0,850),box(-230,230,-150,50,0,20)]);
      default: throw "unknown hobber saddle design";
    };
  }
}
