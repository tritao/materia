package machinekit.enclosure;

import machinekit.assembly.MachineAssembly;
import machinekit.structural.BlockBracket;
import machinekit.component.MachineComponent;
import machinekit.standard.DeepGrooveBearing;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

/** Captured roller suspension with lower anti-sway shoes. Drive coupling is ideal; actuator hardware is not modeled. */
class OpposedSlidingDoor extends MachineAssembly {
  public final stroke:Float;
  public function new(width:Float,height:Float,constructionId:String="polycarbonate-6",gap:Float=10) {
    if(!(width>0&&height>0&&gap>0)||!Math.isFinite(width+height+gap))throw "Invalid opposed door dimensions";
    super();stroke=width/2+gap;
    var railPose=F.identity(),half=width/2+stroke+20;
    // Downward-facing 8 mm slot; twin roller treads run on the two lower lips.
    addComponent("rail",new BlockBracket("door captured roller track",[
      {min:[-half,-22,height+10],max:[half,-20,height+44]},
      {min:[-half,20,height+10],max:[half,22,height+44]},
      {min:[-half,-20,height+40],max:[half,20,height+44]},
      {min:[-half,-20,height+10],max:[half,-4,height+14]},
      {min:[-half,4,height+10],max:[half,20,height+14]}
    ],"steel"));
    function fixed(id:String,part:MachineComponent,parent:String,pose:AssemblyFrame):Void {
      addComponent(id,part);
      addMemberConnector(parent,id,pose);addMemberConnector(id,"doorMount",F.identity());
      addMate(id+"-mount","fixed",parent,id,id,"doorMount");
    }
    for(side in [-1,1]) {
      var mount=side*(half-10);
      fixed('trackMount$side',new BlockBracket("door track frame standoff",[
        {min:[mount-10,22,height+16],max:[mount+10,40,height+36]}
      ],"steel"),"rail",F.identity());
      var end=side*(width/2-width/8+stroke+11);
      fixed('stop$side',new BlockBracket("door end stop",[
        {min:[side<0?end-10:end,-18,height+14],max:[side<0?end:end+10,18,height+40]}
      ],"steel"),"rail",F.identity());
      // Two fixed shoes overlap in engagement as each short leaf traverses them.
      for(x in [side*(width/2-30),side*(stroke+30)])
        fixed('guide$side-$x',new BlockBracket("door lower guide shoe",[
          {min:[x-8,-5,-24],max:[x+8,40,-20]},
          {min:[x-3,-3,-20],max:[x+3,3,-5]},
          {min:[x-8,40,-50],max:[x+8,44,-20]}
        ],"UHMW polyethylene"),"rail",F.identity());
    }
    fixed("headerCover",new BlockBracket("door removable splash hood",[
      {min:[-half-5,-26,height-5],max:[half+5,-24,height+52]},
      {min:[-half-5,-24,height+50],max:[half+5,24,height+52]}
    ],"steel"),"rail",F.identity());
    for(left in [false,true]) {
      var id=left?"leftLeaf":"leaf",joint=left?"leftSlide":"slide",pose=F.translation((left?-1:1)*width/4,0,0);
      var leaf=new EnclosurePanel(width/2,height,constructionId);
      addComponent(id,leaf,pose);
      addMemberConnector("rail",joint,F.compose(F.inverse(railPose),F.compose(pose,leaf.connector("mount").frame)));
      addMateOnAxis(joint,"prismatic","rail",joint,id,"mount",{x:1,y:0,z:0},0,
        {lower:left?-stroke:0,upper:left?0:stroke,velocity:SlidingDoor.SPEED_MM_PER_SECOND,acceleration:400,effort:null,overtravel:SlidingDoor.SWITCH_TOLERANCE_MM});
      var prefix=left?"left":"right";
      fixed(prefix+"LowerChannel",new BlockBracket("door lower anti sway channel",[
        {min:[-width/4,-8,-12],max:[width/4,-5,0]},
        {min:[-width/4,5,-12],max:[width/4,8,0]},
        {min:[-width/4,-5,-3],max:[width/4,5,0]}
      ],"steel"),id,F.identity());
      for(hanger in [-1,1]) {
        var x=hanger*width/8,name=prefix+'Hanger$hanger';
        fixed(name,new BlockBracket("door roller hanger",[
          {min:[x-12,3,height-30],max:[x+12,7,height+4]},
          {min:[x-4,-3,height],max:[x+4,3,height+29]},
          {min:[x-4,-3,height],max:[x+4,7,height+4]}
        ],"steel"),id,F.identity());
        fixed(name+"Axle",new machinekit.motion.SteppedShaft([
          {diameter:12,length:2},{diameter:8,length:30.2}
         ],null,null,null,{outputThread:{diameter:6,pitch:1,length:7}}),id,F.fromRotationMatrix(x,-13.6,height+25,[1,0,0,0,0,1,0,-1,0]));
        for(y in [-4.6,3.0])fixed(name+'Spacer$y',machinekit.standard.FlatWasher.metric("M8"),id,
          F.fromRotationMatrix(x,y,height+25,[1,0,0,0,0,1,0,-1,0]));
        var washer=machinekit.standard.FlatWasher.metric("M6");
        fixed(name+"Washer",washer,id,F.fromRotationMatrix(x,11.6,height+25,[1,0,0,0,0,1,0,-1,0]));
        fixed(name+"Nut",machinekit.standard.HexNut.metric("M6"),id,
          F.fromRotationMatrix(x,11.6+washer.spec.thickness,height+25,[1,0,0,0,0,1,0,-1,0]));
        for(y in [-11.6,4.6]) {
          var wheel=DeepGrooveBearing.metric("608");
          fixed(name+'Roller$y',wheel,id,F.fromRotationMatrix(x,y,height+25,[1,0,0,0,0,1,0,-1,0]));
        }
      }

    }
    addCoupling("opposite-belt-runs","slide","leftSlide",-1);
    addSwitch("closed","slide",{lower:-SlidingDoor.SWITCH_TOLERANCE_MM,upper:SlidingDoor.SWITCH_TOLERANCE_MM,velocity:null,effort:null});
    addSwitch("open","slide",{lower:stroke-SlidingDoor.SWITCH_TOLERANCE_MM,upper:stroke+SlidingDoor.SWITCH_TOLERANCE_MM,velocity:null,effort:null});
    addMemberConnector("rail","mount",F.inverse(railPose));
    exposeConnector("mount","rail","mount");
  }
}
