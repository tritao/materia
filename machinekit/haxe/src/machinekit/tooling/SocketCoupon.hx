package machinekit.tooling;

import cadkit.modeling.Part;
import materia.assembly.AssemblyFrames as F;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.tooling.casting.SocketFixturePlanner;
import materia.tooling.casting.SocketFixturePlanner.SocketFixturePlan;
import materia.tooling.casting.SocketFixturePlanner.BlindSocketSpec;

/** Cured coupon material, with actual socket exterior cavities and protected blind bores.
 * Its source plan stays separate from the old through-bored-box casting contract.
 */
class SocketCoupon extends MachineComponent {
  static var recipe:Null<ComponentType>;
  static function parameters():Array<ComponentParameter> return [P.length("width",400),P.length("depth",240),P.length("height",70),
    P.text("socket",haxe.Json.stringify(SocketFixturePlanner.reference())),P.text("positions",'[{"x":-80,"y":0},{"x":80,"y":0}]')];
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.tooling.blind-socket-coupon",parameters(),v->new SocketCoupon(v),false,false,null,false);
    return recipe;
  }
  public static function fromPlan(p:SocketFixturePlan):SocketCoupon return new SocketCoupon(new ComponentValues()
    .setNumber("width",p.cavity.width).setNumber("depth",p.cavity.depth).setNumber("height",p.cavity.height)
    .setToken("socket",haxe.Json.stringify(p.socket)).setToken("positions",haxe.Json.stringify([for(s in p.sites)
      {x:s.x-p.cavity.x-p.cavity.width/2,y:s.y-p.cavity.y-p.cavity.depth/2}])));
  public function new(?input:ComponentValues) {
    var v=P.resolve(parameters(),input);
    for(n in ["width","depth","height"])if(!Math.isFinite(v.number(n))||v.number(n)<=0)throw "Coupon dimensions must be positive";
    var sites:Array<Dynamic> = haxe.Json.parse(v.token("positions"));
    if(sites==null||sites.length==0||sites.length>32)throw "Coupon needs socket positions";
    var socket=SocketFixtureCad.spec(v.token("socket"));
    if(socket.length>=v.number("height"))throw "Socket tail exits coupon";
    for(s in sites)if(!Math.isFinite(s.x)||!Math.isFinite(s.y)||Math.abs(s.x)+socket.tailWidth>=v.number("width")/2||Math.abs(s.y)+socket.outsideDiameter>=v.number("depth")/2)
      throw "Socket site outside coupon";
    super("SOCKET-COUPON-"+haxe.crypto.Sha256.encode([Std.string(v.number("width")),Std.string(v.number("depth")),Std.string(v.number("height")),v.token("socket"),v.token("positions")].join("|")).substr(0,12),"Mineral-cast blind-socket test coupon","mineral cast",false,v);
    addConnector("mount",Mount,F.identity());
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var positions:Array<Dynamic> = haxe.Json.parse(parameterToken("positions"));
    var socket=SocketFixtureCad.socket(SocketFixtureCad.spec(parameterToken("socket")));
    var cuts:Array<Part> = [];
    for(s in positions) {
      var exterior=socket.geometry(Envelope);
      cuts.push(PosedParts.moved(exterior,F.fromRotationMatrix(s.x,s.y,0,[1,0,0,0,-1,0,0,0,-1])));exterior.close();
    }
    return Solids.cut(Part.box(parameterNumber("width"),parameterNumber("depth"),parameterNumber("height")),cuts);
  }
}
