package machinekit.hobbing;

import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.structural.CastInsert;
import machinekit.structural.MineralCasting;
import machinekit.structural.CastingFeatures;

/** Read compatibility for saved HB5 assemblies; rebuilt parts use structural recipes. */
class LegacyFoundationRecipes {
  public static function register(registry:ComponentRegistry):Void {
    registry.register(new ComponentType("machinekit.hobbing.mineral-casting-body",EngineeringMetadata.parameters().concat([
      P.choice("kind",["bed","column"],"bed"),P.length("width",650),P.length("depth",550),P.length("height",120)]),v -> {
      var kind=v.token("kind"),dims=FoundationLayout.dimensions(kind);
      for(i in 0...3)if(v.number(["width","depth","height"][i])!=dims[i])throw "Legacy casting dimensions differ from its layout";
      var result=FoundationLayout.bodyValues(kind),features=CastingFeatures.decode(result.token("voids"));
      for(row in FoundationLayout.inserts(kind))features.push({id:'insert-${row.id}',width:row.width,depth:row.depth,height:row.height,pose:row.pose});
      result.setToken("voids",CastingFeatures.encode(features));
      if(v.get("material")!=null)result.set("material",v.get("material"));
      return new MineralCasting(result);
    }));
    registry.register(new ComponentType("machinekit.hobbing.cast-in-steel-insert",EngineeringMetadata.parameters().concat([
      P.length("width",35),P.length("depth",400),P.length("height",20),P.length("positionTolerance",0.01),
      P.choice("role",["rail seat","table plate","column plate","threaded bush","lifting point"],"rail seat"),
      P.text("finish","ground after casting")]),v -> {
      var result=new ComponentValues();
      for(name in ["width","depth","height","positionTolerance","role","finish","material"])
        if(v.get(name)!=null)result.set(name,v.get(name));
      return new CastInsert(result);
    }));
  }
}
