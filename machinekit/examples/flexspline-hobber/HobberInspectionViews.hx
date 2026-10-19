import materia.project.SceneArtifact.SceneArtifactData;

/** Scene-authored presets; the editor has no hobber-specific camera code. */
class HobberInspectionViews {
  public static function apply(scene:SceneArtifactData,machine:FlexsplineHobber,prefix:String=""):Void {
    if(scene.project==null)scene.project={dynamicParts:[],motions:[]};
    var project=scene.project;
    var cutaway=["enclosure/"];
    for(id in ["bed","foot","table","column","rail","block","X","Z","Y","swivel","spindle","outboard","drive","chip","coolant","guard"])cutaway.push(prefix+id);
    project.inspectionCameras=[
      {id:"cutting",label:"Tool contact (cutaway)",anchor:prefix+"blank",offset:[0,machine.spec.tipRadius()*0.001,(machine.spec.faceWidth-40)*0.0005],span:0.04,yaw:0.3,pitch:-0.4,hiddenPrefixes:cutaway},
      {id:"workpiece",label:"Workpiece",anchor:prefix+"blank",offset:[0,0,0],span:Math.max(0.085,machine.spec.tipRadius()*0.003),yaw:-1.0,pitch:0.4,hiddenPrefixes:["enclosure/"]},
      {id:"loading",label:"Loading",anchor:prefix+"mandrel",offset:[0,0,0.02],span:0.4,yaw:-1.0,pitch:0.5,hiddenPrefixes:["enclosure/"]}
    ];
  }
}
