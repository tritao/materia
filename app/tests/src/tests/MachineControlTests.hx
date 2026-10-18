package tests;

import app.MachineControlModel;
import app.ProjectDocumentSession;
import app.SceneObjectData;
import cadkit.modeling.AssemblyModel;
import materia.assembly.AssemblyFrames;

@:access(app.Main.ReferenceEditorApp)
@:access(app.ProjectDocumentSession)
class MachineControlTests {
  static var checks=0;
  static function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
  static function near(a:Null<Float>,b:Float,message:String):Void {if(a==null)throw message+": missing value";check(Math.abs(a-b)<1e-7,message+': $a != $b');}
  static function refused(fn:Void->Void,message:String):Void {var rejected=false;try fn() catch(_:Dynamic)rejected=true;check(rejected,message);}
  static function topology(session:ProjectDocumentSession):cadkit.modeling.CompiledAssembly {
    var state=session.assemblyRuntime;if(state==null)throw "Missing assembly runtime";return state.model;
  }
  static function slidePosition(session:ProjectDocumentSession):Float {
    var object=session.scene.object("project:slide");if(object==null)throw "Missing slide";return object.x;
  }
  static function fixture(unit:String):ProjectDocumentSession {
    var model=new AssemblyModel(unit);
    for(id in ["base","slide","screw","head","rotor"]) {model.add(id);model.connector(id,"axis",AssemblyFrames.identity());}
    model.mateOnAxis("x","prismatic","base","axis","slide","axis",{x:1,y:0,z:0},0,
      {lower:unit=="mm" ? -100 : -0.1,upper:unit=="mm" ? 100 : 0.1,velocity:null,effort:null});
    model.mateOnAxis("screw-turn","continuous","base","axis","screw","axis",{x:0,y:0,z:1});
    model.mateOnAxis("tilt","revolute","slide","axis","head","axis",{x:0,y:0,z:1},0,
      {lower:-Math.PI/2,upper:Math.PI/2,velocity:null,effort:null});
    model.mateOnAxis("spindle/rotor-turn","continuous","base","axis","rotor","axis",{x:0,y:0,z:1});
    model.couple("drive","x","screw-turn",unit=="mm" ? Math.PI*2/5 : Math.PI*2/0.005);
    var data:Array<SceneObjectData> = [for(id in ["base","slide","screw","head","rotor"])
      {id:"project:"+id,label:id,type:"rectangle",x:0.0,y:0.0,z:0.0,width:0.01,height:0.01,depth:0.01,
        collisionEnabled:false,dynamicBody:false,mass:1.0,red:0.5,green:0.5,blue:0.5,visible:true}];
    var centers:Map<String,Array<Float>> = [];for(id in ["base","slide","screw","head","rotor"])centers.set(id,[0,0,0]);
    var session=new ProjectDocumentSession(null,false);
    session.openGeneratedScene(data,null,null,model.definition("machine-controls"),null,centers,unit=="mm" ? 0.001 : 1.0);
    return session;
  }
  static function gestureChecks(session:ProjectDocumentSession):Void {
    var before=MachineControlModel.axes(session).filter(a->a.id=="x")[0].value;
    var history=session.document.history.undoCount,model=topology(session);
    var drag=session.beginAssemblyJointEdit("x");
    for(value in [10.0,20.0,35.0]) {
      drag.update(MachineControlModel.nativePosition(session,"x",value,true));
      check(topology(session)==model,"preview reuses compiled assembly topology");
      near(slidePosition(session),value/1000,"slider follows live preview");
      check(session.document.history.undoCount==history,"pointer moves create no history entries");
    }
    check(drag.commit(),"release commits a drag");
    check(!drag.commit(),"duplicate release is harmless");
    check(session.document.history.undoCount==history+1,"whole drag records one undo entry");
    session.document.undo();near(MachineControlModel.axes(session).filter(a->a.id=="x")[0].value,before,"undo restores pre-drag position");
    session.document.redo();near(slidePosition(session),0.035,"redo restores final position");
    check(topology(session)==model,"commit, undo and redo reuse compiled topology");
    history=session.document.history.undoCount;
    var cancelled=session.beginAssemblyJointEdit("x");cancelled.update(MachineControlModel.nativePosition(session,"x",-50,true));cancelled.cancel();
    near(slidePosition(session),0.035,"cancel restores starting position");check(session.document.history.undoCount==history,"cancel adds no undo entry");
    var noOp=session.beginAssemblyJointEdit("x");check(!noOp.commit(),"unchanged click adds no undo entry");
    var returned=session.beginAssemblyJointEdit("x");returned.update(MachineControlModel.nativePosition(session,"x",70,true));returned.update(MachineControlModel.nativePosition(session,"x",35,true));
    check(!returned.commit(),"returning to starting position is a no-op");
    refused(()->{session.beginAssemblyJointEdit("screw-turn");},"coupled target cannot begin a gesture");
  }
  static function sliderRouting():Void {
    var fonts=haxeon.ui.FontCollection.create();
    var ui=new haxeon.ui.core.UiContext(null,fonts),session=fixture("mm");
    var edit:Null<app.ProjectJointEdit> = null;
    var frame=new haxeon.ui.LayoutFrame(256,100);
    function slider():app.editor.MachineControlSlider {
      var value=MachineControlModel.axes(session).filter(a->a.id=="x")[0].value;
      return new app.editor.MachineControlSlider("drag-test","X",value,-100,100,0.1,
        next->{var current=edit;if(current==null)MachineControlModel.set(session,"x",next,true);else current.update(next);},
        _->{edit=session.beginAssemblyJointEdit("x");},
        commit->{var current=edit;edit=null;if(current!=null){if(commit)current.commit();else current.cancel();}});
    }
    try {
      var view=slider(),root=ui.submit(view,frame),bounds=root.globalBounds();
      var x=bounds.x+10,y=bounds.y+bounds.height/2,width=bounds.width-20;
      var history=session.document.history.undoCount;
      ui.pointerDown(x+width*0.2,y,0);
      root=ui.submit(slider(),frame);
      ui.pointerMove(x+width*0.8,y);
      check(session.document.history.undoCount==history,"routed rebuilt slider does not record moves");
      near(slidePosition(session),0.06,"rebuilt slider keeps preview gesture");
      ui.pointerUp(x+width*0.8,y,0);
      check(session.document.history.undoCount==history+1,"routed release commits one edit");
      session.document.undo();near(slidePosition(session),0,"routed drag undo restores initial pose");
      ui.submit(slider(),frame);ui.pointerDown(x+width*0.7,y,0);ui.pointerCancel(0,x,y);
      near(slidePosition(session),0,"pointer cancellation restores pose");
      check(session.document.history.undoCount==history,"pointer cancellation preserves history");
      view=slider();root=ui.submit(view,frame);ui.focusWidget(root.id);
      ui.key(haxeon.ui.core.UiEventKind.KeyDown,haxeon.ui.core.UiKey.Right);
      check(session.document.history.undoCount==history+1,"keyboard slider edits remain immediate");
      ui.dispose();fonts.dispose();session.dispose();
    } catch(error:Dynamic){ui.dispose();fonts.dispose();session.dispose();throw error;}
  }
  static function angleRouting():Void {
    near(app.editor.MachineAngleControl.deltaAngle(350,10),20,"dial wraps forward across zero");
    near(app.editor.MachineAngleControl.deltaAngle(10,350),-20,"dial wraps backward across zero");
    near(app.editor.MachineAngleControl.deltaAngle(1080,90),90,"accumulated turns retain orientation");
    check(app.editor.MachineControlPanel.rotaryTitle("spindle/rotor-turn")=="Spindle rotation","rotor label is readable");
    check(app.editor.MachineControlPanel.rotaryTitle("mill/spindle/rotor-turn")=="mill · Spindle rotation","multiple machines retain their scope labels");
    var session=fixture("mm"),fonts=haxeon.ui.FontCollection.create();
    var ui=new haxeon.ui.core.UiContext(null,fonts),frame=new haxeon.ui.LayoutFrame(256,100);
    var edit:Null<app.ProjectJointEdit> = null;
    var id="spindle/rotor-turn";
    function value():Float return MachineControlModel.axes(session).filter(a->a.id==id)[0].value;
    function control(dial:Bool,enabled:Bool=true,?minimum:Float,?maximum:Float):app.editor.MachineAngleControl {
      var result=new app.editor.MachineAngleControl("angle-test", "Rotation", value(),dial,
        next->{var current=edit;var native=MachineControlModel.nativePosition(session,id,next,true);if(current==null)MachineControlModel.set(session,id,next,true);else current.update(native);},
        _->{edit=session.beginAssemblyJointEdit(id);},
        commit->{var current=edit;edit=null;if(current!=null){if(commit)current.commit();else current.cancel();}},minimum,maximum);
      result.enabled=enabled;return result;
    }
    try {
      var root=ui.submit(control(false),frame),bounds=root.globalBounds();
      var x=bounds.x+30,y=bounds.y+bounds.height/2,history=session.document.history.undoCount;
      ui.pointerDown(x,y,0);ui.pointerMove(x+40,y);near(value(),40,"scrubber moves one degree per pixel");
      check(session.document.history.undoCount==history,"scrubber previews without history");
      ui.submit(control(false),frame);ui.pointerUp(x+60,y,0);
      near(value(),60,"release uses final pointer position after rebuild");
      check(session.document.history.undoCount==history+1,"scrubber drag records one edit");
      session.document.undo();near(value(),0,"scrubber undo restores starting angle");session.document.redo();near(value(),60,"scrubber redo restores final angle");
      ui.submit(control(false),frame);ui.pointerDown(x,y,0);ui.pointerMove(x+100,y,haxeon.ui.core.UiModifier.Shift);ui.pointerUp(x+100,y,0,haxeon.ui.core.UiModifier.Shift);
      near(value(),70,"Shift scrub gives fine adjustment");
      MachineControlModel.set(session,id,350,true);
      root=ui.submit(control(true),frame);bounds=root.globalBounds();
      var cx=bounds.x+bounds.width/2,cy=bounds.y+bounds.height/2;
      function px(a:Float):Float return cx+24*Math.sin(a*Math.PI/180);
      function py(a:Float):Float return cy-24*Math.cos(a*Math.PI/180);
      history=session.document.history.undoCount;
      ui.pointerDown(px(350),py(350),0);ui.pointerMove(px(10),py(10));near(value(),370,"dial crosses zero without jumping a turn");
      for(a in [90.0,170.0,250.0,330.0,50.0]) {ui.submit(control(true),frame);ui.pointerMove(px(a),py(a));}
      near(value(),770,"dial accumulates multiple turns across rebuilds");ui.pointerUp(px(50),py(50),0);
      check(session.document.history.undoCount==history+1,"multi-turn dial drag records one edit");
      session.document.undo();near(value(),350,"dial undo restores full accumulated angle");
      ui.submit(control(true),frame);ui.pointerDown(px(90),py(90),0);ui.pointerCancel(0,px(90),py(90));near(value(),350,"dial cancellation restores angle");
      ui.submit(control(true),frame);ui.pointerDown(px(350),py(350),0,haxeon.ui.core.UiModifier.Shift);ui.pointerMove(px(10),py(10),haxeon.ui.core.UiModifier.Shift);ui.pointerUp(px(10),py(10),0,haxeon.ui.core.UiModifier.Shift);
      near(value(),352,"Shift dial gives fine adjustment");
      MachineControlModel.set(session,id,0,true);ui.submit(control(false,true,-90,90),frame);ui.pointerDown(x,y,0);ui.pointerMove(x+1000,y);ui.pointerUp(x+1000,y,0);near(value(),90,"bounded angular control clamps at travel limit");
      ui.submit(control(false,false),frame);ui.pointerDown(x,y,0);ui.pointerMove(x+40,y);ui.pointerUp(x+40,y,0);near(value(),90,"disabled rotary control does not move");
      root=ui.submit(control(false),frame);ui.focusWidget(root.id);ui.key(haxeon.ui.core.UiEventKind.KeyDown,haxeon.ui.core.UiKey.Right);near(value(),91,"rotary control supports keyboard adjustment");
      MachineControlModel.set(session,id,3600000,true);ui.submit(control(false),frame);
      ui.pointerDown(x,y,0);ui.pointerMove(x+20,y);ui.pointerUp(x+20,y,0);
      near(value(),3600020,"scrubber retains large accumulated turn counts without integer overflow");
      ui.dispose();fonts.dispose();session.dispose();
    } catch(error:Dynamic){ui.dispose();fonts.dispose();session.dispose();throw error;}
  }
  static function responsiveness(path:String):Void {
    var generated=app.ProjectSourceLoader.load(path),fonts=haxeon.ui.FontCollection.create();
    fonts.add("haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf");
    var editor=new app.Main.ReferenceEditorApp(fonts,null,null,null,null,null,null,null,true);
    var session=editor.session,panel=new app.editor.MachineControlPanel(),frame=new haxeon.ui.LayoutFrame(320,1000);
    session.openGeneratedProject(generated,path);editor.documentChanged();
    editor.ui.submit(panel.build(editor),frame);
    try {
      for(axis in MachineControlModel.axes(session)) if(axis.follows==null) {
        var samples:Array<Float> = [],layouts:Array<Float> = [],start=axis.value;
        var span=axis.minimum!=null && axis.maximum!=null ? (axis.maximum-axis.minimum)*0.1 : 90.0;
        var low=axis.minimum==null ? start-span/2 : Math.max(axis.minimum,start-span/2);
        var high=axis.maximum==null ? start+span/2 : Math.min(axis.maximum,start+span/2);
        var edit=session.beginAssemblyJointEdit(axis.id);
        for(i in 0...110) {
          var value=low+(high-low)*(i%20)/19;
          var began=Sys.time();
          edit.update(MachineControlModel.nativePosition(session,axis.id,value,true));
          var updated=Sys.time();
          editor.ui.submit(panel.build(editor),frame);
          if(i>=10){samples.push((updated-began)*1000);layouts.push((Sys.time()-began)*1000);}
        }
        edit.cancel();samples.sort((a,b)->a<b ? -1 : a>b ? 1 : 0);
        layouts.sort((a,b)->a<b ? -1 : a>b ? 1 : 0);
        var began=Sys.time();MachineControlModel.set(session,axis.id,high,true);
        editor.documentChanged();editor.ui.submit(panel.build(editor),frame);
        var apply=(Sys.time()-began)*1000;session.document.undo();
        trace('Machine response ${axis.id}: preview median=${samples[50]}ms p95=${samples[95]}ms max=${samples[99]}ms; including panel layout p95=${layouts[95]}ms; numeric apply and layout=${apply}ms');
      }
      editor.dispose();fonts.dispose();
    } catch(error:Dynamic){editor.dispose();fonts.dispose();throw error;}
  }
  public static function main():Int {
    var args=Sys.args();if(args.length==2 && args[0]=="--responsiveness"){responsiveness(args[1]);return 0;}
    var empty=new ProjectDocumentSession(null,false);check(MachineControlModel.axes(empty).length==0,"empty documents have no axes");empty.dispose();
    for(unit in ["mm","m"]) {
      var session=fixture(unit);
      try {
        var axes=MachineControlModel.axes(session);check(axes.length==4,"movable joints only");
        var x=axes.filter(a->a.id=="x")[0],turn=axes.filter(a->a.id=="screw-turn")[0],tilt=axes.filter(a->a.id=="tilt")[0];
        near(x.minimum,-100,"linear lower displayed in mm");near(x.maximum,100,"linear upper displayed in mm");
        near(tilt.maximum,90,"rotary limits displayed in degrees");check(turn.follows=="x","coupled targets are read-only");
        var history=session.document.history.undoCount;
        check(MachineControlModel.set(session,"x",75,true),"axis change applies");
        near(slidePosition(session),0.075,"axis moves geometry in metres");
        near(MachineControlModel.axes(session).filter(a->a.id=="screw-turn")[0].value,15*360,"screw follows axis");
        check(session.document.history.undoCount==history+1,"axis changes are undoable");
        check(session.document.undo(),"undo available");near(slidePosition(session),0,"undo restores pose");
        check(session.document.redo(),"redo available");near(slidePosition(session),0.075,"redo restores pose");
        refused(()->{MachineControlModel.set(session,"x",101,true);},"out-of-range rejected");
        refused(()->{MachineControlModel.set(session,"x",Math.NaN,true);},"NaN rejected");
        refused(()->{MachineControlModel.set(session,"x",0,false);},"runtime ownership lock respected");
        refused(()->{MachineControlModel.set(session,"screw-turn",0,true);},"coupling target cannot be driven");
        refused(()->{MachineControlModel.set(session,"missing",0,true);},"unknown joint rejected");
        near(MachineControlModel.axes(session).filter(a->a.id=="x")[0].value,75,"rejected edits leave coordinate unchanged");
        check(MachineControlModel.set(session,"x",-100,true),"lower stop reachable");
        check(MachineControlModel.set(session,"x",100,true),"upper stop reachable");
        check(MachineControlModel.set(session,"tilt",90,true),"rotary stop reachable");
        near(MachineControlModel.axes(session).filter(a->a.id=="tilt")[0].value,90,"rotary position uses degrees");
        gestureChecks(session);
        session.dispose();
      } catch(error:Dynamic) {session.dispose();throw error;}
    }
    sliderRouting();angleRouting();
    near(MachineControlModel.parsePosition(" -1.25e2 "),-125,"numeric entry accepts scientific notation");
    for(text in ["","-","1abc","NaN","Infinity"]) refused(()->{MachineControlModel.parsePosition(text);},"invalid numeric input rejected");
    check(EditorWorkspaceLayoutTests.main()==0,"machine tab layout regression");
    trace('MachineControlTests passed ($checks checks)');return 0;
  }
}
