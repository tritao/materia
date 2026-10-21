import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;
import materia.presentation.AssemblyPresentation;
import materia.presentation.PresentationCodec;
import materia.presentation.PresentationEvaluator;

class PresentationTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void { assertions++; if (!value) throw message; }
  static function rejects(action:Void->Void):Void {
    var rejected = false; try action() catch (_:Dynamic) rejected = true;
    check(rejected, "Invalid presentation accepted");
  }
  static function signature(frame:materia.presentation.PresentationEvaluator.PresentationFrame):String {
    var ids = [for (id in frame.poses.keys()) id]; ids.sort(Reflect.compare);
    var values:Array<String> = [];
    for (id in ids) {
      var item = frame.poses.get(id); if (item == null) throw "Missing pose";
      values.push(id + ":" + item.x + ":" + item.y + ":" + item.z + ":" + Std.string(frame.visibility.get(id)));
    }
    return values.join("|") + ":" + frame.highlight.join(",") + ":" + frame.evidence;
  }
  static function pose(frame:materia.presentation.PresentationEvaluator.PresentationFrame):materia.assembly.AssemblyRecord.AssemblyFrame {
    var value = frame.poses.get("unit/a"); if (value == null) throw "Missing pose"; return value;
  }
  public static function run():Void {
    var definition:AssemblyDefinition = {schemaVersion:AssemblyDefinitionCodec.VERSION, id:"test", definitions:[{id:"part",connectors:[]}],
      occurrences:[{id:"unit/a",definition:"part",initialPose:AssemblyFrames.identity()},
        {id:"unit/b",definition:"part",initialPose:AssemblyFrames.identity()}], joints:[]};
    var select:PartSelector = {occurrence:"unit",subtree:true};
    var leaf:PartSelector = {occurrence:"unit/a",subtree:false};
    var motion:PresentationMotion = {path:[{x:0,y:0,z:0},{x:0,y:5,z:0}],evidence:Checked("checker",Failed,"geometry","report")};
    var value:AssemblyPresentation = {schemaVersion:1,definition:"test",defaultView:"assembled",
      views:[{id:"assembled",label:"Assembled",camera:Frame([select],0,0,1),visibility:[],sections:[],explode:0}],
      explode:[{select:select,direction:{x:1,y:0,z:0},distance:10,stage:1},
        {select:leaf,direction:{x:0,y:1,z:0},distance:20,stage:2}],
      procedures:[{id:"remove",label:"Remove",kind:Disassembly,start:Assembled,steps:[
        {id:"a",title:"Remove A",instruction:"Withdraw A",highlight:[leaf],action:Remove(leaf,motion)},
        {id:"b",title:"Note",instruction:"Inspect",highlight:[],action:Note}]}]};
    nestedComposition(value);
    var engineering = AssemblyDefinitionCodec.encode(definition);
    var scene = @:privateAccess ProjectKitTests.sceneData();
    scene.assemblyDefinition = definition; scene.presentation = value;
    var bytes = materia.project.SceneArtifact.encode(scene);
    var decodedScene = materia.project.SceneArtifact.decode(bytes);
    var decodedPresentation = decodedScene.presentation;
    if (decodedPresentation == null) throw "Scene lost presentation";
    check(PresentationCodec.encode(definition,decodedPresentation) == PresentationCodec.encode(definition,value),"Scene presentation round trip");
    check(materia.project.SceneArtifact.encode(decodedScene).compare(bytes) == 0,"Scene re-encodes identically");
    scene.assemblyDefinition = null; rejects(()->materia.project.SceneArtifact.encode(scene));
    var encoded = PresentationCodec.encode(definition,value);
    check(PresentationCodec.encode(definition,PresentationCodec.decode(definition,encoded)) == encoded,"Presentation round trip");
    var poses = new Map<String, materia.assembly.AssemblyRecord.AssemblyFrame>();
    for (part in definition.occurrences) poses.set(part.id,AssemblyFrames.identity());
    var cursor = PresentationEvaluator.reset(value);
    cursor.explode = 0.5;
    var frame = PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry");
    check(pose(frame).x == 10 && pose(frame).y == 0,"First explode stage");
    cursor.explode = 1;
    frame = PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry");
    check(pose(frame).x == 10 && pose(frame).y == 20,"Nested explode offsets compose");
    poses.set("unit",AssemblyFrames.turnY(Math.PI / 2));
    value.explode[1].direction = {x:1,y:0,z:0};
    frame = PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry");
    check(Math.abs(pose(frame).x - 10) < 1e-8 && Math.abs(pose(frame).z + 20) < 1e-8,"Child explode follows rotated parent frame");
    poses.remove("unit"); value.explode[1].direction = {x:0,y:1,z:0};
    value.views[0].visibility = [{select:select,mode:Hidden},{select:leaf,mode:Transparent}];
    frame = PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry");
    check(frame.visibility.get("unit/a") == Transparent && frame.visibility.get("unit/b") == Hidden,"Ordered visibility overrides");
    value.views[0].visibility = [];
    value.views[0].sections = [{occurrence:"unit/a",point:{x:1,y:2,z:3},normal:{x:0,y:1,z:0}}];
    frame = PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry");
    check(frame.sections[0].point.x == 11 && frame.sections[0].point.y == 22,"Section follows displayed occurrence");
    value.views[0].sections = [];
    var original = poses.get("unit/a"); if (original == null) throw "Missing input pose";
    check(original.x == 0,"Solved poses are immutable");
    cursor.procedure = "remove"; cursor.explode = 0;
    frame = PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry");
    check(frame.evidence == "Failed","Negative evidence is visible");
    check(PresentationEvaluator.evaluate(definition,poses,value,cursor,"changed").evidence == "Stale","Changed geometry invalidates evidence");
    var fresh = signature(PresentationEvaluator.evaluate(definition,poses,value,PresentationEvaluator.reset(value),"geometry"));
    cursor.step = 1;
    check(PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry").visibility.get("unit/a") == Hidden,"Completed removal");
    var forward = signature(PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry"));
    cursor.step = 2; cursor.step -= 1;
    check(signature(PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry")) == forward,"Previous equals direct fold");
    check(PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry").visibility.get("unit/a") == Hidden,"Previous recomputes fold");
    cursor.step = 0;
    check(PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry").visibility.get("unit/a") == Shown,"Restart restores assembly");
    cursor.explode = 1;
    check(pose(PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry",true)).x == 0,"Playback suppresses display motion");
    check(signature(PresentationEvaluator.evaluate(definition,poses,value,PresentationEvaluator.reset(value),"geometry")) == fresh,"Reset equals fresh open");
    check(AssemblyDefinitionCodec.encode(definition) == engineering,"Presentation leaves engineering definition byte-identical");
    var reversed = materia.presentation.Procedures.reverse(value.procedures[0],"install","Install",[]);
    check(reversed.start == Disassembled && reversed.kind == Assembly,"Explicit reverse procedure start and kind");
    switch reversed.steps[1].action { case Install(_, _): case _: throw "Reverse removal is an installation"; }
    reversed.steps[1].instruction = "Replacement instruction";
    check(value.procedures[0].steps[0].instruction == "Withdraw A","Reverse owns its records");
    value.procedures.push(reversed);
    cursor.explode = 0; cursor.procedure = "install"; cursor.step = 0;
    check(PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry").visibility.get("unit/a") == Hidden,"Disassembled starts removed");
    cursor.step = 2;
    check(PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry").visibility.get("unit/a") == Shown,"Installation restores part");
    value.schemaVersion = 2; rejects(()->PresentationCodec.validate(definition,value)); value.schemaVersion = 1;
    value.defaultView = "missing"; rejects(()->PresentationCodec.validate(definition,value)); value.defaultView = "assembled";
    value.explode[0].stage = 0; rejects(()->PresentationCodec.validate(definition,value)); value.explode[0].stage = 1;
    value.views[0].visibility = [{select:{occurrence:"missing",subtree:false},mode:Hidden}];
    rejects(()->PresentationCodec.validate(definition,value)); value.views[0].visibility = [];
    rejects(()->PresentationCodec.resolve(definition,{occurrence:"uni",subtree:true}));
    value.views[0].source = Generated(""); rejects(()->PresentationCodec.validate(definition,value)); value.views[0].source = Authored;
    value.views.push(value.views[0]); rejects(()->PresentationCodec.validate(definition,value)); value.views.pop();
    motion.evidence = Checked("checker",Passed,"geometry","report");
    value.procedures[0].source = Authored; rejects(()->PresentationCodec.validate(definition,value));
    motion.evidence = Illustrative;
    var authored = PresentationCodec.decode(definition,PresentationCodec.encode(definition,value));
    check(authored.procedures[0].source == Authored,"Authored procedure source round trip");
    motion.path[1].x = Math.NaN; rejects(()->PresentationCodec.validate(definition,value)); motion.path[1].x = 0;
    value.procedures[0].source = Generated("test");
    motion.evidence = Checked("checker",Passed,"","report"); rejects(()->PresentationCodec.validate(definition,value));
    jointFold();
    Sys.println('Presentation tests passed ($assertions assertions)');
  }
  static function nestedComposition(value:AssemblyPresentation):Void {
    var a = AssemblyFrames.translation(1,0,0), b = AssemblyFrames.translation(0,0,1);
    var definition:AssemblyDefinition = {schemaVersion:AssemblyDefinitionCodec.VERSION,id:"test",definitions:[],
      occurrences:[{id:"unit",definition:"module",assembly:"module",initialPose:AssemblyFrames.translation(100,0,0)}],joints:[],
      assemblies:[{id:"module",definitions:[{id:"part",connectors:[]}],occurrences:[
        {id:"a",definition:"part",initialPose:a},{id:"b",definition:"part",initialPose:b}],joints:[]}]};
    var group = AssemblyFrames.turnY(Math.PI / 2); group.x = 100;
    var solved = new Map<String,materia.assembly.AssemblyRecord.AssemblyFrame>();
    solved.set("unit/a",AssemblyFrames.compose(group,a)); solved.set("unit/b",AssemblyFrames.compose(group,b));
    var cursor = PresentationEvaluator.reset(value); cursor.explode = 1;
    value.explode[1].direction = {x:1,y:0,z:0};
    var frame = PresentationEvaluator.evaluate(definition,solved,value,cursor,"geometry");
    check(Math.abs(pose(frame).x - 110) < 1e-8 && Math.abs(pose(frame).z + 21) < 1e-8,
      "Nested assembly offsets recover the solved parent scope from flattened poses");
    value.explode[1].direction = {x:0,y:1,z:0};
  }
  static function jointFold():Void {
    var definition:AssemblyDefinition = {schemaVersion:AssemblyDefinitionCodec.VERSION,id:"joints",
      definitions:[{id:"part",connectors:[{name:"pin",frame:AssemblyFrames.identity()}]}],
      occurrences:[{id:"base",definition:"part",initialPose:AssemblyFrames.identity()},
        {id:"jaw",definition:"part",initialPose:AssemblyFrames.identity()}],
      joints:[{id:"open",type:"prismatic",role:"tree",parent:"base",parentConnector:"pin",child:"jaw",
        childConnector:"pin",axis:{x:1,y:0,z:0},defaultValue:0,limits:{lower:null,upper:null,velocity:null,effort:null}}]};
    var value:AssemblyPresentation = {schemaVersion:1,definition:"joints",defaultView:"assembled",
      views:[{id:"assembled",label:"Assembled",camera:Frame([{occurrence:"base",subtree:false}],0,0,1),visibility:[],sections:[],explode:0}],explode:[],
      procedures:[{id:"service",label:"Service",kind:Service,start:Assembled,steps:[
        {id:"open",title:"Open",instruction:"Open jaws",highlight:[],action:SetJoints([{joint:"open",value:5}])},
        {id:"inspect",title:"Inspect",instruction:"Inspect jaws",highlight:[],action:Note}]}]};
    var poses = new Map<String,materia.assembly.AssemblyRecord.AssemblyFrame>();
    poses.set("base",AssemblyFrames.identity()); poses.set("jaw",AssemblyFrames.identity());
    var cursor = PresentationEvaluator.reset(value); cursor.procedure = "service"; cursor.step = 1;
    var state:materia.assembly.AssemblyDefinition.AssemblyStateRecord = {schemaVersion:AssemblyDefinitionCodec.VERSION,
      definition:definition.id,jointCoordinates:[{joint:"open",value:0}],rootPoses:[]};
    var before = AssemblyDefinitionCodec.encodeState(definition,state);
    var frame = PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry");
    check(frame.jointOverrides.get("open") == 5,"Joint overrides fold from completed steps");
    check(AssemblyDefinitionCodec.encodeState(definition,state) == before,"Procedure does not modify saved engineering state");
    cursor.step = 0;
    check(!PresentationEvaluator.evaluate(definition,poses,value,cursor,"geometry").jointOverrides.exists("open"),"Previous removes later joint overrides");
    var reverse = materia.presentation.Procedures.reverse(value.procedures[0],"restore","Restore",[{joint:"open",value:0}]);
    switch reverse.steps[1].action {
      case SetJoints(keys): check(keys[0].value == 0,"Reverse restores explicit initial joint coordinate");
      case _: throw "Expected reverse joint action";
    }
    rejects(()->materia.presentation.Procedures.reverse(value.procedures[0],"restore","Restore",[]));
  }

}
