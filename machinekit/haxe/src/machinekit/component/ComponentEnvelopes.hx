package machinekit.component;

import cadkit.modeling.Part;

/** Simple envelope geometry only; engineering facts belong to optional facets.
    Geometry methods return a new owned Part for the caller to close. */
class ComponentEnvelopes {
  public static function validateBlock(v:ComponentValues):Void {
    for(k in ["width","depth","height"])if(v.number(k)<=0)throw "invalid block envelope";
  }
  public static function validateAnnular(v:ComponentValues):Void {
    if(v.number("bore")<0||v.number("bore")>=v.number("outside")||v.number("outside")<=0||v.number("height")<=0)
      throw "invalid annular envelope";
  }
  public static function block(v:ComponentValues):Part {
    validateBlock(v);return Solids.named(Part.box(v.number("width"),v.number("depth"),v.number("height")),"body");
  }
  public static function annular(v:ComponentValues):Part {
    validateAnnular(v);
    var body=Solids.named(Part.cylinderSpan(v.number("outside")/2,0,v.number("height")),"body");
    return v.number("bore")==0?body:Solids.cut(body,[Solids.named(Part.cylinderSpan(v.number("bore")/2,-0.1,v.number("height")+0.1),"bore")]);
  }
}
