import materia.project.SceneArtifact.SceneArtifactData;
import materia.presentation.AssemblyPresentation;
import materia.presentation.Presentations;

/** Scene-authored views; the viewer has no hobber-specific presentation behavior. */
class HobberInspectionViews {
  public static function apply(scene:SceneArtifactData,machine:FlexsplineHobber,prefix:String=""):Void {
    var definition=scene.assemblyDefinition;
    if(definition==null)throw "Hobber views require an assembly definition";
    var flat=materia.assembly.AssemblyDefinitionFlattener.flattenView(definition);
    var presentation=Presentations.assembled(definition,-1.1,0.35,"hobber");
    // Resolve the previous producer's name families once into exact occurrence selectors.
    // The stored record never uses prefix matching and will fail on renamed parts.
    var cutaway=["enclosure/"];
    for(id in ["bed","foot","table","column","rail","block","X","Z","Y","swivel","spindle","outboard","drive","chip","coolant","guard"])cutaway.push(prefix+id);
    function rules(families:Array<String>,mode:VisibilityMode):Array<VisibilityRule> {
      return [for(part in flat.occurrences)if(families.filter(f->StringTools.startsWith(part.id,f)).length>0)
        {select:{occurrence:part.id,subtree:false},mode:mode}];
    }
    var enclosure=rules(["enclosure/"],Hidden);
    var scale=scene.metresPerUnit;
    function view(id:String,label:String,anchor:String,offset:Array<Float>,span:Float,yaw:Float,pitch:Float,
        visibility:Array<VisibilityRule>):PresentationView return {id:id,label:label,
      camera:Anchored({occurrence:prefix+anchor,subtree:false},{x:offset[0]/scale,y:offset[1]/scale,z:offset[2]/scale},span/scale,yaw,pitch),
      visibility:visibility,sections:[],explode:0,source:Generated("hobber")};
    presentation.views.push(view("cutting","Tool contact (cutaway)","blank",[0,machine.spec.tipRadius()*0.001,(machine.spec.faceWidth-40)*0.0005],0.04,0.3,-0.4,rules(cutaway,Hidden)));
    presentation.views.push(view("workpiece","Workpiece","blank",[0,0,0],Math.max(0.085,machine.spec.tipRadius()*0.003),-1.0,0.4,enclosure));
    presentation.views.push(view("loading","Loading","mandrel",[0,0,0.02],0.4,-1.0,0.5,enclosure));
    for(choice in [{id:"enclosure-transparent",label:"Enclosure transparent",mode:Transparent},{id:"enclosure-hidden",label:"Enclosure hidden",mode:Hidden}])
      presentation.views.push({id:choice.id,label:choice.label,camera:presentation.views[0].camera,
        visibility:rules(["enclosure/"],choice.mode),sections:[],explode:0,source:Generated("hobber")});
    var external=[for(part in flat.occurrences)if(StringTools.startsWith(part.id,prefix)&&HobberEnclosurePreview.external(part.id.substr(prefix.length)))
      {select:{occurrence:part.id,subtree:false},mode:Hidden}];
    for(entry in presentation.views)entry.visibility=entry.visibility.concat(external);
    scene.presentation=presentation;
  }
}
