import cadkit.modeling.Part;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentDetail;
import machinekit.milling.FlatEndMill;
import machinekit.milling.MillPowerDrawbarPart;

/** Visible rotation details remain connected and within their nominal envelopes. */
class MillVisualDetailChecks {
  static var checks=0;
  static function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
  public static function main():Void {
    for(size in [{d:3.0,f:10.0,s:20.0},{d:6.0,f:22.0,s:35.0},{d:10.0,f:20.0,s:20.0}]) {
      var tool=new FlatEndMill(new ComponentValues().setNumber("diameter",size.d).setNumber("fluteLength",size.f).setNumber("stickout",size.s));
      var preview=tool.geometry(Preview),envelope=tool.geometry(Envelope);
      check(preview.valid(),"fluted cutter is valid");check(preview.solidCount()==1,"flutes leave a connected cutting core");
      check(preview.volume()<envelope.volume()*0.98,"flutes visibly remove material");
      var excess=preview.subtract(envelope);check(excess.volume()<1e-6,"flutes preserve the packaging envelope");excess.close();
      preview.close();envelope.close();
    }
    var holder=new MillPowerDrawbarPart(new ComponentValues().setToken("role","holder")).geometry();
    check(holder.valid() && holder.solidCount()==1,"drive slots leave a valid connected holder");
    var probe=Part.box(2,2,4).translated(new cadkit.modeling.Vector(20,0,-8));
    var removed=holder.intersect(probe);check(removed.volume()<1e-6,"holder drive slot makes orientation visible");
    removed.close();probe.close();holder.close();
    Sys.println('Mill visual details: $checks checks passed');
  }
}
