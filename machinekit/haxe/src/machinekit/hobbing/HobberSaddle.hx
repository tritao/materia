package machinekit.hobbing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Separate designed weldments, with machined mounting datums. Dimensions are mm.
    Strength, weld distortion and modal stiffness remain assumed pending Phase D. */
class HobberSaddle extends MachineComponent {
  final designId:String;
  static var recipe:Null<ComponentType>;
  public static function parameters():Array<ComponentParameter> return [
    P.choice("design",["GE-X-carriage","GE-Z-saddle","GE-X-drive","GE-Z-drive","GE-chip-tray","GE-enclosure","GE-coolant", "GE-swivel-plate","GE-Y-saddle","GE-Y-drive","electrical-panel"],"GE-Z-saddle")];
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.hobbing.saddle",parameters(),v->new HobberSaddle(v.token("design")));
    return recipe;
  }
  public function new(design:String){
    var v=P.resolve(parameters(),new ComponentValues().setToken("design",design));
    super("HOBBER-"+design,"Designed steel "+design+"; machined seats, assumed structural adequacy","steel",false,v);
    designId=design;
    addConnector("datum",Mount,F.identity());
    if(["GE-X-carriage","GE-Z-saddle","GE-X-drive","GE-Z-drive","GE-Y-saddle","GE-Y-drive"].indexOf(design)>=0){
      var parts=pieces();try{addFacet(ExtrusionCollision.pieces(parts));}catch(e:Dynamic){for(p in parts)p.close();throw e;}for(p in parts)p.close();
    }
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
  static function radialPlate(x0:Float,x1:Float,section:Array<{y:Float,z:Float}>):Part {
    var raw=Part.prism([for(p in section)new Vector(p.y,p.z)],x0,x1);
    var shaped=machinekit.assembly.PosedParts.moved(raw,F.fromRotationMatrix(0,0,0,[0.0,0,1,1,0,0,0,1,0]));raw.close();return shaped;
  }
  override public function geometry(detail:ComponentDetail=Preview):Part return Solids.union(pieces());
  function pieces():Array<Part> {
    return switch designId {
      case "GE-X-carriage": [
        box(240,386,-245,245,260,280),box(60,386,-245,-195,260,280),box(60,386,195,245,260,280),
        Solids.cut(box(202.5,222.5,-35,35,155,280),[boreX(8.2,202.4,222.6,0,180)]),box(202.5,260,-35,35,260,280)];
      case "GE-Z-saddle": [
        box(135,148,-175,175,-65,170),box(135,148,-190,-175,140,257),
        Solids.cut(box(111,148,-270,-175,239.5,257),[Part.cylinderSpan(8.2,239.4,257.1,136,-250)])];
      case "GE-swivel-plate": [box(85,100,-115,115,-95,95)];
      case "GE-Y-saddle": [
        radialPlate(45,57,[{y:-130.0,z:-40.0},{y:-65.0,z:-75.0},{y:65.0,z:-75.0},{y:90.0,z:-55.0},{y:90.0,z:95.0},{y:-130.0,z:95.0}]),box(45,57,90,240,55,95),
        Solids.cut(box(-55,57,80,100,-55,55),[boreY(45,79.9,100.1,0,0)]),
        Solids.cut(box(-55,40,200,220,-55,55),[boreY(45,199.9,220.1,0,0)]),box(40,57,200,220,45,55),
        Solids.cut(box(-45,57,-115,-85,-45,45),[box(-30,30,-115.1,-84.9,-30,30)]),
        Solids.cut(box(45,95,-45,-27.5,95,170),[boreY(8.2,-45.1,-27.4,70,145)])];
      case "GE-Y-drive": [
        box(35,45,-160,130,115,190),box(35,100,-115,-95,95,115),
        box(35,115,100,130,185,190),
        Solids.cut(box(35,115,125,130,110,190),[boreY(7.1,124.9,130.1,70,145)])];
      case "GE-X-drive": [box(160,630,-45,45,140,155),box(160,620,40,45,155,190),
        Solids.cut(box(625,630,-45,45,150,220),[boreX(7.1,624.9,630.1,0,180)])];
      case "GE-Z-drive": [box(165,255,-230,-210,295,887),box(55,145,-280,-275,445,887),
        Solids.cut(box(55,220,-295,-210,440,445),[Part.cylinderSpan(9.6,439.9,445.1,100,-250)]),
        box(55,220,-275,-210,882,887),box(35,95,-300,-290,445,887)];
      case "GE-chip-tray": [Part.revolve([{r:142.0,z:225.0},{r:147.0,z:164.0},{r:147.0,z:139.0},{r:145.0,z:139.0},{r:145.0,z:164.0},{r:140.0,z:223.0}]),Solids.cut(Solids.union([box(-245,90,-170,170,135,139),box(-245,90,-174,-170,135,164),box(-245,90,170,174,135,164)]),[box(-141,141,-141,141,134,145)])];
      case "GE-coolant": [Solids.cut(box(33,45,30,50,60,84),[boreX(2,32.9,45.1,40,72)])];
      case "GE-enclosure": [box(-270,780,-360,-356,0,950),box(-270,780,356,360,0,950),box(776,780,-360,360,0,950)];
      case "electrical-panel": [box(-210,210,-10,0,0,850),box(-230,230,-150,50,0,20)];
      default: throw "unknown hobber saddle design";
    };
  }
}
