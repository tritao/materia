import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyVector;
import materia.presentation.AssemblyPresentation;
import materia.presentation.Presentations;

/** Finger service is ordinary presentation data; supplier screw release remains illustrative. */
class DhGripperPresentation {
  public static function build(definition:AssemblyDefinition,guide:AssemblyVector):AssemblyPresentation {
    var result=Presentations.assembled(definition,-1.1,0.35,"dh-gripper");
    var focus:Array<PartSelector> =[for(part in definition.occurrences)if(part.id!="blank-60") {occurrence:part.id,subtree:false}];
    result.views[0].camera=Frame(focus,-1.1,0.35,1.15);
    var detail:Array<PartSelector> =[{occurrence:"jaw-left",subtree:false},{occurrence:"finger-left",subtree:false}];
    result.views.push({id:"jaw-detail",label:"Jaw detail",camera:Frame(detail,2.0,0.45,1.8),visibility:[],
      sections:[{occurrence:null,point:guide,normal:{x:0,y:0,z:-1}}],explode:0,source:Generated("dh-gripper")});
    result.views.push({id:"finger-service",label:"Finger service",camera:Frame(detail,2.0,0.45,1.8),visibility:[],
      sections:[],explode:0,source:Generated("dh-gripper")});
    for(side in ["left","right"]) {
      for(index in 0...2)result.explode.push({select:{occurrence:'finger-$side-screw-$index',subtree:false},
        direction:{x:0,y:1,z:0},distance:35,stage:1,source:Generated("dh-gripper")});
      result.explode.push({select:{occurrence:'finger-$side',subtree:false},direction:{x:0,y:1,z:0},distance:80,stage:2,source:Generated("dh-gripper")});
    }
    var steps:Array<ProcedureStep> =[{id:"open-jaws",title:"Open the jaws",instruction:"Open the jaws fully before changing the left finger.",
      view:"assembled",highlight:detail,action:SetJoints([{joint:DhGripperJaws.MASTER,value:25}])}];
    function move(id:String,title:String,instruction:String,distance:Float,install:Bool):Void {
      var select:PartSelector={occurrence:id,subtree:false};
      var motion:PresentationMotion={path:[{x:0,y:0,z:0},{x:0,y:distance,z:0}],evidence:Illustrative};
      steps.push({id:(install?"install-":"remove-")+id,title:title,instruction:instruction,view:"finger-service",highlight:[select],
        action:install?Install(select,motion):Remove(select,motion)});
    }
    for(index in 0...2)move('finger-left-screw-$index',"Remove finger screw "+(index+1),"Release the screw thread and withdraw the screw from the finger.",35,false);
    move("finger-left","Withdraw the finger","Support the left finger and withdraw it from the carriage.",80,false);
    move("finger-left","Install the replacement finger","Seat the replacement finger on the same carriage datums.",80,true);
    for(index in 0...2)move('finger-left-screw-$index',"Reinstall finger screw "+(index+1),"Reinstall the screw and tighten to the approved supplier specification.",35,true);
    result.procedures.push({id:"finger-change",label:"Change left finger",kind:Service,start:Assembled,steps:steps,source:Generated("dh-gripper")});
    materia.presentation.PresentationCodec.validate(definition,result);
    return result;
  }
}
