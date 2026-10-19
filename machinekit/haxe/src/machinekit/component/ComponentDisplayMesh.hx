package machinekit.component;

typedef ComponentDisplayMeshData = {positions:Array<Float>,normals:Array<Float>,indices:Array<Int>};

/** Optional display tessellation; CAD, collision geometry and physical properties remain explicit. */
class ComponentDisplayMesh implements ComponentFacet {
  public final build:Void->ComponentDisplayMeshData;
  public function new(build:Void->ComponentDisplayMeshData)this.build=build;
  public static function of(component:MachineComponent):Null<ComponentDisplayMesh> {
    for(facet in component.facets())if(Std.isOfType(facet,ComponentDisplayMesh))return cast facet;
    return null;
  }
  public function check(component:MachineComponent):Void {}
  public function describe():String return "Parametric display mesh; physical properties come from the component CAD representation";
}
