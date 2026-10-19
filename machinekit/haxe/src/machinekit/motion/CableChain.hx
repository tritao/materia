package machinekit.motion;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Assumed catalogue-class cable chain. Dimensions mm; SI ratings unless stated.
 * Geometry is a nominal layout envelope, not manufacturing or supplier geometry. */
class CableChain extends AssumedCatalogComponent {
  static var recipe:Null<ComponentType>;
  static function inputs():Array<ComponentParameter> return [
    P.length("bendRadius", 75),
    P.length("innerWidth", 40),
    P.length("innerHeight", 25),
    P.length("wall", 3),
    P.length("travel", 2800),
    P.quantity("fillArea", 400, "mm2"),
    P.scalar("maxFill", 0.6, 0, 1)
  ];
  public function new(?v:ComponentValues) {
    super("cable-chain", "cable chain", "PA12 nylon", inputs(), v);
    positive(["bendRadius", "innerWidth", "innerHeight", "wall", "travel", "maxFill"]);
    require(n("bendRadius") > n("innerHeight") / 2 + n("wall"), "bend radius too small for chain height");
    require(n("fillArea") <= n("innerWidth") * n("innerHeight") * n("maxFill"), "chain fill exceeds available area");
    addConnector("fixed", Mount, F.translation(0, 0, 0));
    addConnector("moving", Mount, movingFrame(n("travel")/2));
  }
  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.motion.cable-chain", inputs(), v -> new CableChain(v));
    return recipe;
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function geometry(detail:ComponentDetail = Preview):Part return geometryAt(n("travel")/2,detail);

  /** Hollow continuous link envelope at a travel position; links and pins are nominal. */
  public function geometryAt(position:Float,detail:ComponentDetail = Preview):Part {
    movingFrame(position);
    var h=n("innerHeight")+2*n("wall"),w=n("innerWidth")+2*n("wall"),r=n("bendRadius");
    if(detail==Envelope) {
      var outer=r+h/2, blank=Part.box(w,n("travel")+outer,2*r+h);
      var result=blank.translated(new Vector(0,outer/2,-h/2));blank.close();
      return Solids.named(result,"chain-travel-envelope");
    }
    var delta=position-n("travel")/2,lower=(n("travel")/2+delta)/2;
    var outer:Array<cadkit.modeling.Curve> = [],inner:Array<cadkit.modeling.Curve> = [];
    function section(y:Float,z:Float,a:Float):Void {
      function rectangle(width:Float,height:Float):cadkit.modeling.Curve {
        return cadkit.modeling.Curve.polyline([
          for(corner in [{x:-1,y:-1},{x:1,y:-1},{x:1,y:1},{x:-1,y:1}])
            new Vector(corner.x*width/2,y-corner.y*height/2*Math.sin(a),z+corner.y*height/2*Math.cos(a))
        ],true);
      }
      outer.push(rectangle(w,h));inner.push(rectangle(n("innerWidth"),n("innerHeight")));
    }
    try {
      section(0,0,0);
      if(lower>1e-9) section(lower,0,0);
      for(i in 1...33) {var a=Math.PI*i/32;section(lower+r*Math.sin(a),r*(1-Math.cos(a)),a);}
      if(lower-delta>1e-9) section(delta,2*r,Math.PI);
      var body=Part.loft(outer,true),bore:Part=null;
      try {bore=Part.loft(inner,true);} catch(error:Dynamic) {body.close();throw error;}
      for(c in outer)c.close();outer=[];for(c in inner)c.close();inner=[];
      return Solids.cut(Solids.named(body,"links-envelope"),[Solids.named(bore,"cable-space")]);
    } catch(error:Dynamic) {for(c in outer)c.close();for(c in inner)c.close();throw error;}
  }
  /** Fixed end at mid-travel; both tangents remain nonnegative at travel limits. */
  public function developedLength():Float return n("travel") / 2 + Math.PI * n("bendRadius");
  public function movingFrame(position:Float):materia.assembly.AssemblyRecord.AssemblyFrame {
    require(Math.isFinite(position) && position >= 0 && position <= n("travel"), "moving end outside travel");
    return F.translation(0, position - n("travel") / 2, 2 * n("bendRadius"));
  }
  /** Centreline in the YZ plane, lower run, 180-degree bend, upper run. */
  public function path(position:Float):Array<Vector> {
    movingFrame(position);
    var delta = position - n("travel") / 2, lower =(n("travel") / 2 + delta) / 2, r = n("bendRadius");
    var points = [new Vector(0, 0, 0), new Vector(0, lower, 0)];
    for (i in 1...33) {
      var a = Math.PI * i / 32;
      points.push(new Vector(0, lower + r * Math.sin(a), r *(1 - Math.cos(a))));
    }
    points.push(new Vector(0, delta, 2 * r));
    return points;
  }
}
