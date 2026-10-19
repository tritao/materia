package tests;

import app.MachineControlModel;
import app.ProjectDocumentSession;
import app.SceneObjectData;
import cadkit.modeling.AssemblyModel;
import materia.assembly.AssemblyFrames;

@:access(app.Main.ReferenceEditorApp)
@:access(app.ProjectDocumentSession)
@:access(app.EditorScene)
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
  static function fixture(unit:String,?existing:ProjectDocumentSession):ProjectDocumentSession {
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
    var session=existing==null ? new ProjectDocumentSession(null,false) : existing;
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
  static function angleSemantics(node:haxeon.ui.core.RenderNode):haxeon.ui.semantics.Semantics {
    var semantics=node.semantics;if(semantics==null)throw "Missing angle semantics";return semantics;
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
      near(angleSemantics(root).numericValue,10,"dial exposes wrapped orientation while coordinates keep turns");
      near(angleSemantics(root).numericMinimum,0,"continuous dial starts at zero degrees");
      near(angleSemantics(root).numericMaximum,360,"continuous dial exposes a full circle");
      for(a in [90.0,170.0,250.0,330.0,50.0]) {ui.submit(control(true),frame);ui.pointerMove(px(a),py(a));}
      near(value(),770,"dial accumulates multiple turns across rebuilds");
      root=ui.submit(control(true),frame);near(angleSemantics(root).numericValue,50,"rebuilt dial preserves wrapped orientation");
      ui.pointerUp(px(50),py(50),0);
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
      root=ui.submit(control(false),frame);near(angleSemantics(root).numericValue,20,"large turn counts display a single orientation");
      MachineControlModel.set(session,id,-1,true);root=ui.submit(control(false),frame);
      near(angleSemantics(root).numericValue,359,"negative coordinates display positive orientation");
      ui.focusWidget(root.id);ui.key(haxeon.ui.core.UiEventKind.KeyDown,haxeon.ui.core.UiKey.Right);
      near(value(),0,"keyboard crosses zero without adding a reverse revolution");
      near(angleSemantics(root).numericValue,0,"keyboard wraps displayed orientation at zero");
      ui.dispose();fonts.dispose();session.dispose();
    } catch(error:Dynamic){ui.dispose();fonts.dispose();session.dispose();throw error;}
  }
  static function orientationEntry():Void {
    var session=fixture("mm"),id="spindle/rotor-turn";
    function axis():app.MachineControlModel.MachineControlAxis return MachineControlModel.axes(session).filter(a->a.id==id)[0];
    try {
      near(MachineControlModel.wrapDegrees(450),90,"450 degrees displays as 90");
      near(MachineControlModel.wrapDegrees(-30),330,"negative angles wrap positively");
      near(MachineControlModel.wrapDegrees(360),0,"a full revolution displays as zero");
      near(MachineControlModel.wrapDegrees(-720),0,"negative whole revolutions display as zero");
      MachineControlModel.set(session,id,350,true);
      near(MachineControlModel.enteredPosition(axis(),10),370,"numeric entry crosses zero along the shortest rotation");
      MachineControlModel.set(session,id,MachineControlModel.enteredPosition(axis(),10),true);
      MachineControlModel.set(session,id,MachineControlModel.enteredPosition(axis(),450),true);
      near(axis().value,450,"numeric entry keeps accumulated turns");
      near(MachineControlModel.wrapDegrees(axis().value),90,"numeric entry displays normalized orientation");
      session.document.undo();near(axis().value,370,"undo restores the continuous coordinate");
      session.document.redo();near(axis().value,450,"redo restores the continuous coordinate");
      var bounded=MachineControlModel.axes(session).filter(a->a.id=="tilt")[0];
      check(!MachineControlModel.wrapsAngle(bounded),"bounded rotary joints retain authored ranges");
      near(MachineControlModel.enteredPosition(bounded,450),450,"bounded numeric entry does not wrap past its stops");
      refused(()->{MachineControlModel.set(session,"tilt",MachineControlModel.enteredPosition(bounded,450),true);},"bounded rotary stops still reject wrapped equivalents");
      session.dispose();
    }catch(error:Dynamic){session.dispose();throw error;}
  }
  static function orientationPanel():Void {
    var fonts=haxeon.ui.FontCollection.create();
    var font="haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf";
    if(!sys.FileSystem.exists(font))font="../../"+font;
    fonts.add(font);
    var editor=new app.Main.ReferenceEditorApp(fonts,null,null,null,null,null,null,null,true);
    var session=fixture("mm",editor.session),panel=new app.editor.MachineControlPanel(),frame=new haxeon.ui.LayoutFrame(360,1600);
    try {
      editor.documentChanged();MachineControlModel.set(session,"spindle/rotor-turn",350,true);
      var root=editor.ui.submit(panel.build(editor),frame),field=findControl(root,"Spindle rotation (deg)");
      if(field==null)throw "Missing spindle angle field";
      check(angleSemantics(field).value=="350","pane shows normalized starting angle");
      check(editor.ui.accessibilityAction(field.id.value,4,"450",0,0,0),"angle field accepts numeric input");
      root=editor.ui.submit(panel.build(editor),frame);field=findControl(root,"Spindle rotation (deg)");
      if(field==null)throw "Missing edited angle field";
      editor.ui.focusWidget(field.id);editor.ui.key(haxeon.ui.core.UiEventKind.KeyDown,haxeon.ui.core.UiKey.Enter);
      root=editor.ui.submit(panel.build(editor),frame);field=findControl(root,"Spindle rotation (deg)");
      if(field==null)throw "Missing applied angle field";
      check(angleSemantics(field).value=="90","Enter normalizes 450 degrees to 90 in the visible field");
      near(MachineControlModel.axes(session).filter(a->a.id=="spindle/rotor-turn")[0].value,450,"pane keeps the nearest continuous coordinate");
      session.document.undo();root=editor.ui.submit(panel.build(editor),frame);field=findControl(root,"Spindle rotation (deg)");
      if(field==null)throw "Missing undo angle field";
      check(angleSemantics(field).value=="350","undo restores the displayed orientation");
      MachineControlModel.set(session,"spindle/rotor-turn",359.9999,true);
      root=editor.ui.submit(panel.build(editor),frame);field=findControl(root,"Spindle rotation (deg)");
      if(field==null)throw "Missing rounded angle field";
      check(angleSemantics(field).value=="0","rounding at the wrap never displays 360");
      editor.dispose();fonts.dispose();
    }catch(error:Dynamic){editor.dispose();fonts.dispose();throw error;}
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
  static function findControl(node:haxeon.ui.core.RenderNode,label:String):Null<haxeon.ui.core.RenderNode> {
    if(node.semantics!=null && node.semantics.label==label)return node;
    for(child in node.children){var found=findControl(child,label);if(found!=null)return found;}
    return null;
  }
  static function nativePose(scene:app.EditorScene,id:String):Array<Float> {
    var node=scene.snapshot.findNode(scene.runtimeFor("project:"+id).node);
    if(node==null)throw "Missing native node: "+id;
    var transform=node.worldTransform();return [for(i in 0...16)transform.element(i)];
  }
  static function spindleView(path:String):Void {
    var fonts=haxeon.ui.FontCollection.create();fonts.add("haxeon/packages/ui/vendor/skribidi/example/data/IBMPlexSans-Regular.ttf");
    var editor=new app.Main.ReferenceEditorApp(fonts,null,null,null,null,null,null,null,true);
    var session=editor.session,panel=new app.editor.MachineControlPanel(),frame=new haxeon.ui.LayoutFrame(360,2000);
    try {
      session.openGeneratedProject(app.ProjectSourceLoader.load(path),path);editor.documentChanged();
      var host=new haxeon.ui.host.UiHostContext(fonts,new haxeon.platform.NativeKitEvents(),function(){},function(){});
      var viewport=new app.EditorPerspectiveViewport("spindle-section-audit",session.scene,host);
      editor.perspectiveViewport=viewport;
      var housing=nativePose(session.scene,"spindle/spindle");
      for(test in [{id:"spindle/rotor-turn",label:"Spindle rotation scrubber",parts:["rotor","holder","tool","motorShaft","spindlePulley","motorPulley"]},
          {id:"spindle/drawbar-release",label:"spindle/drawbar-release position (mm)",parts:["drawbar","gripper"]},
          {id:"spindle/pistonRod-release",label:"spindle/pistonRod-release position (mm)",parts:["pistonRod","piston"]}]) {
        var sectionRoot=editor.ui.submit(panel.build(editor),frame),inspect=findControl(sectionRoot,"Inspect "+test.id);
        if(inspect==null)throw "Missing axis inspection button: "+test.id;
        var buttonBounds=inspect.globalBounds(),bx=buttonBounds.x+buttonBounds.width/2,by=buttonBounds.y+buttonBounds.height/2;
        var inspectionHistory=session.document.history.undoCount;
        editor.ui.pointerDown(bx,by,0);editor.ui.pointerUp(bx,by,0);
        check(viewport.sectionActive(),"axis Inspect opens a live section: "+test.id);
        check(session.document.history.undoCount==inspectionHistory,"Inspect does not change machine history");
        var before=[for(part in test.parts)nativePose(session.scene,"spindle/"+part)];
        var visual=session.scene.visualRevision,history=session.document.history.undoCount;
        var root=editor.ui.submit(panel.build(editor),frame),control=findControl(root,test.label);
        if(control==null)throw "Missing machine control: "+test.label;
        var bounds=control.globalBounds(),x=bounds.x+10,y=bounds.y+bounds.height/2;
        var rotary=test.id=="spindle/rotor-turn";
        if(!rotary)x=bounds.x+10+(bounds.width-20)*0.2;
        editor.ui.pointerDown(x,y,0);
        var end=rotary ? x+47 : bounds.x+10+(bounds.width-20)*0.8;
        editor.ui.pointerMove(end,y);editor.ui.submit(panel.build(editor),frame);editor.ui.pointerUp(end,y,0);
        check(session.scene.visualRevision>visual,"spindle gesture invalidates rendered scene: "+test.id);
        check(session.document.history.undoCount==history+1,"spindle pane gesture records one edit: "+test.id);
        var axis=MachineControlModel.axes(session).filter(a->a.id==test.id)[0];
        near(axis.value,rotary ? 47 : test.id=="spindle/drawbar-release" ? 3.6 : 5.2,"spindle pane changes its own joint");
        for(i in 0...test.parts.length) {
          var after=nativePose(session.scene,"spindle/"+test.parts[i]),changed=false;
          for(j in 0...16)if(Math.abs(after[j]-before[i][j])>1e-8)changed=true;
          check(changed,"native rendered transform moves: "+test.parts[i]);
          if(!rotary)near(after[14]-before[i][14],-axis.value/1000,"native release travel is in metres");
        }
        var fixed=nativePose(session.scene,"spindle/spindle");
        for(i in 0...16)near(fixed[i],housing[i],"spindle housing remains stationary");
        Sys.println('Spindle view: ${test.id}=${axis.value}${axis.unit}; native transforms updated for ${test.parts.join(", ")}');
        session.document.undo();
        for(i in 0...test.parts.length) {
          var restored=nativePose(session.scene,"spindle/"+test.parts[i]);
          for(j in 0...16)near(restored[j],before[i][j],"undo restores native spindle transform");
        }
      }
      var closeRoot=editor.ui.submit(panel.build(editor),frame),close=findControl(closeRoot,"Close section");
      if(close==null)throw "Missing close section action";
      var closeBounds=close.globalBounds();editor.ui.pointerDown(closeBounds.x+8,closeBounds.y+8,0);editor.ui.pointerUp(closeBounds.x+8,closeBounds.y+8,0);
      check(!viewport.sectionActive(),"Machine pane closes inspection");
      editor.dispose();fonts.dispose();Sys.println('Spindle view: $checks checks passed');
    }catch(error:Dynamic){editor.dispose();fonts.dispose();throw error;}
  }
  static function posePersistenceChecks():Void {
    var document=new haxeon.ui.editing.EditorDocument("persistence"),value=0;
    function operation(persistent:Bool,from:Int,to:Int):haxeon.ui.editing.EditOperation
      return new haxeon.ui.editing.EditOperation("Change",()->value=to,()->value=from,null,null,null,256,persistent);
    document.apply(operation(false,0,1));check(!document.isDirty,"session edits preserve clean savepoint");
    document.undo();check(!document.isDirty && value==0,"session undo remains clean");
    document.redo();check(!document.isDirty && value==1,"session redo remains clean");
    document.apply(operation(true,1,2));check(document.isDirty,"design edit requires save");
    document.apply(operation(false,2,3));document.undo();check(document.isDirty,"session undo cannot hide a design edit");
    document.undo();check(!document.isDirty,"undo design edit returns to saved content through session history");
    document.redo();document.redo();document.markSaved();
    document.undo();check(!document.isDirty,"undo session step across a savepoint remains clean");
    document.undo();check(document.isDirty,"undo saved design content requires save");
    document.apply(operation(true,1,4));check(document.isDirty,"branched design history cannot collide with saved token");
    document.markSaved();
    var transaction=document.begin("Session transaction");transaction.apply(operation(false,4,5));transaction.commit();
    check(!document.isDirty,"session-only transaction remains clean");
    transaction=document.begin("Mixed transaction");transaction.apply(operation(false,5,6));transaction.apply(operation(true,6,7));transaction.commit();
    check(document.isDirty,"mixed transaction affects persistence");document.undo();check(!document.isDirty,"mixed transaction undo restores savepoint");
    document.markSaved();
    var count=document.history.undoCount;
    transaction=document.begin("Travel drag","travel");transaction.apply(operation(false,5,6));transaction.commit();
    transaction=document.begin("Travel drag","travel");transaction.apply(operation(false,6,7));transaction.commit();
    check(document.history.undoCount==count+1 && !document.isDirty,"coalesced session transactions remain clean");
    document.undo();check(value==5 && !document.isDirty,"coalesced session undo restores its start");
    var session=fixture("mm");
    try {
      session.document.markSaved();MachineControlModel.set(session,"x",25,true);
      check(!session.isDirty(),"travel does not trigger close prompt");
      var closed=false;
      var controller=new app.SceneDocumentController(session,function(_,_,_){throw "Unexpected path dialog";},function(){});
      controller.requestClose(()->closed=true);
      check(closed && !controller.needsConfirmation(),"native close workflow skips confirmation after travel only");
      var initial=session.initialAssemblyState;
      check(initial!=null,"initial assembly pose retained");
      near(session.initialJointCoordinate("x",-1),0,"preview leaves initial pose unchanged");
      var gesture=session.beginAssemblyJointEdit("x");gesture.update(50);gesture.commit();
      check(!session.isDirty(),"slider gesture remains session-only");
      session.document.undo();check(!session.isDirty(),"slider undo remains clean");
      session.document.redo();check(!session.isDirty(),"slider redo remains clean");
      check(session.saveCurrentPoseAsInitial(),"explicit initial pose change applies");
      check(session.isDirty(),"explicit initial pose requires saving");
      closed=false;controller.requestClose(()->closed=true);
      check(!closed && controller.needsConfirmation(),"native close workflow protects an authored initial pose");
      controller.resolve("cancel");
      near(MachineControlModel.axes(session).filter(a->a.id=="x")[0].initial,50,"Initial button uses authored pose");
      session.document.markSaved();MachineControlModel.set(session,"x",75,true);
      check(!session.isDirty(),"travel after saving initial pose remains clean");
      // Ordinary saves serialize the initial pose, not the live preview.
      session.projectReference=sys.FileSystem.fullPath("haxeon.json");
      session.projectBaseline=session.scene.recordsForSave();
      var saved=session.projectSaveData("/tmp/materia-pose-persistence.materia");
      var definition=session.projectAssemblyDefinition;if(definition==null)throw "Missing fixture definition";
      var encoded=saved.record.assemblyState;if(encoded==null)throw "Missing saved initial pose";
      var restored=materia.assembly.AssemblyDefinitionCodec.decodeState(definition,encoded);
      for(joint in restored.jointCoordinates)if(joint.joint=="x")near(joint.value,50,"ordinary save excludes live preview");
      session.document.undo();check(!session.isDirty(),"undo travel after save stays clean");
      session.document.undo();check(session.isDirty(),"undo saved initial pose is a persistent edit");
      near(session.initialJointCoordinate("x",-1),0,"undo initial pose restores previous authoring state");
      session.dispose();
    }catch(error:Dynamic){session.dispose();throw error;}
  }
  public static function main():Int {
    var args=Sys.args();if(args.length==2 && args[0]=="--responsiveness"){responsiveness(args[1]);return 0;}
    if(args.length==2 && args[0]=="--spindle-view"){spindleView(args[1]);return 0;}
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
    posePersistenceChecks();
    sliderRouting();angleRouting();orientationEntry();orientationPanel();
    near(MachineControlModel.parsePosition(" -1.25e2 "),-125,"numeric entry accepts scientific notation");
    for(text in ["","-","1abc","NaN","Infinity"]) refused(()->{MachineControlModel.parsePosition(text);},"invalid numeric input rejected");
    check(EditorWorkspaceLayoutTests.main()==0,"machine tab layout regression");
    trace('MachineControlTests passed ($checks checks)');return 0;
  }
}
