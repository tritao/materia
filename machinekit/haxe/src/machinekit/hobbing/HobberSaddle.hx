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
    P.choice("design",["GE-X-carriage","GE-Z-saddle","GE-X-drive","GE-Z-drive","GE-chip-tray","GE-guard","GE-enclosure","GE-coolant", "Z-saddle","X-bridge","A-yoke","Y-saddle","electrical-panel","X-drive","Z-drive","Y-drive"],"Z-saddle")];
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
      case "GE-X-carriage": Solids.union([
        box(240,350,-185,185,260,280),box(48,350,-185,-140,260,280),box(48,350,140,185,260,280),
        Solids.cut(box(277.5,297.5,195,275,215,280),[boreX(8.2,277.4,297.6,250,240)]),box(277.5,297.5,165,195,260,280)]);
      case "GE-Z-saddle": Solids.union([
        box(200,220,-110,110,-100,180),box(99,220,-110,-100,-100,180),box(99,220,100,110,-100,180),
        box(99,112,-175,-100,-50,232),box(99,112,100,175,-50,160),
        Solids.cut(box(75,112,-270,-125,214.5,232),[Part.cylinderSpan(8.2,214.4,232.1,100,-250)])]);
      case "GE-X-drive": Solids.union([box(125,620,210,295,210,215),box(125,620,290,295,215,250),
        Solids.cut(box(120,125,205,295,215,285),[boreX(7.1,119.9,125.1,250,240)])]);
      case "GE-Z-drive": Solids.union([box(165,255,-230,-210,295,862),box(55,145,-280,-275,420,862),
        Solids.cut(box(55,220,-295,-210,415,420),[Part.cylinderSpan(9.6,414.9,420.1,100,-250)]),
        box(55,220,-275,-210,857,862),box(35,95,-300,-290,420,862)]);
      case "GE-chip-tray": Solids.union([Part.revolve([{r:95.0,z:220.0},{r:140.0,z:150.0},{r:140.0,z:125.0},{r:138.0,z:125.0},{r:138.0,z:150.0},{r:95.0,z:218.0}]),Solids.cut(Solids.union([box(-245,90,-145,145,121,125),box(-245,90,-149,-145,121,150),box(-245,90,145,149,121,150)]),[box(-91,91,-91,91,120,131)])]);
      case "GE-guard": {
        var pieces:Array<Part> = [];
        for(side in [-1,1])for(span in [[20.0,60.0],[610.0,700.0]]){
          var y=side*165;pieces.push(box(span[0],span[1],y-35,y+35,255,258));
          for(edge in [-1,1]){var cy=y+edge*33.5;pieces.push(box(span[0],span[1],cy-1.5,cy+1.5,220,255));
            var a=edge<0?y-35:y+14,b=edge<0?y-14:y+35;pieces.push(box(span[0],span[1],a,b,220,223));}
        }
        // Four separate end shields are one BOM set; moving bellows remain a reserved provision.
        Solids.union(pieces);
      }
      case "GE-coolant": Solids.cut(box(-46,-34,30,50,80,104),[Part.cylinderSpan(2,79.9,104.1,-40,40)]);
      case "GE-enclosure": Solids.union([box(-270,780,-360,-356,0,950),box(-270,780,356,360,0,950),box(776,780,-360,360,0,950)]);
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
        box(70,90,-100,100,0,172),box(-70,90,-120,120,152,172),box(-160,-70,-45,45,152,172)]);
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
