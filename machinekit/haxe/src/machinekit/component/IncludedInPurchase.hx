package machinekit.component;
/** A physical subpart carried by the separately listed purchase, rather than ordered on its own. */
class IncludedInPurchase implements ComponentFacet {
  public final ownerPartNumber:String;
  public function new(ownerPartNumber:String){if(ownerPartNumber==null||ownerPartNumber=="")throw "Purchase owner required";this.ownerPartNumber=ownerPartNumber;}
  public function check(component:MachineComponent):Void {}
  public function describe():String return 'Included in purchase $ownerPartNumber';
  public static function of(component:MachineComponent):Null<IncludedInPurchase>{for(f in component.facets())if(Std.isOfType(f,IncludedInPurchase))return cast f;return null;}
}
