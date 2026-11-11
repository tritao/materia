import hobber.*;
import cadkit.modeling.AssemblyState;
import gearkit.*;
import machinekit.assembly.*;
import machinekit.assembly.AxisBuilder;
import machinekit.component.*;
import machinekit.control.BissMaster;
import machinekit.design.DesignRequirement;
import machinekit.motion.*;
import machinekit.hobbing.*;
import machinekit.power.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

class HobberZAxis extends HobberUnit implements machinekit.cover.WayCover.CoverMembers {
  public final zBellows:Array<machinekit.cover.WayCover>=[];
  public final zBellowsMounts:Array<String>=[];
  public function new(skeleton:HobberSkeleton) {
    super("zAxis", skeleton);
    var centre=0.0;
    for (placed in layout.rails("column", layout.foundationPlacements("column"))) {
      var insert = "column/" + placed.insertId;
      externalPart(placed.memberId, placed.rail, HobberFrame.sourcePose(placed.pose), "foundation", "column-"+placed.insertId, HobberFrame.sourcePose(placed.contact), true);
    }
    for (side in [-1, 1]) for (row in [0.0, layout.zBlockSpacing]) {
      var id = "blockZ" + (side < 0 ? "Left" : "Right") + (row == 0 ? "" : "Upper");
      var block = LinearRailBlock.metric("HGH15CA", LinearRailBlock.SUPPLIER);
      var pose = AxisBuilder.orient(-side * layout.columnRailCentre, centre + heights.zGuideOriginY, heights.hobCentreZ + row, [0.0, -1, 0], [0.0, 0, 1]);
      var face = F.alongY(-side * layout.columnRailCentre, centre + heights.zBlockFaceY, heights.hobCentreZ + row, 0, -1, 0);
      if (id == "blockZLeft") {
        addComponent(id, block, pose);
        moving("Z", Prismatic, "railZLeft", id, HobberFrame.partAt(0, centre + heights.zGuideOriginY, heights.hobCentreZ), [0.0, 0, 1], 0, layout.zTravelMin, layout.zTravelMax);
        fixedPart("Zplate", new HobberZSaddle(), HobberFrame.partAt(0, centre, heights.hobCentreZ), id, face);
      } else fixedPart(id, block, pose, "Zplate", face);
    }
    host("saddle","Zplate");
    group("Drive");
    var axis=HobberDrives.z();
      for(support in ["fixed","floating"]) {
        var id=HobberDrives.barMember(axis.id,support),face=HobberPadBolts.face(axis,support);
        externalPart(id,new HobberDriveBar(axis.id,support),HobberFrame.partAt(0, 0, HobberFrame.PART_DATUM_Z),"foundation",axis.id=="X"?"bed-driveSeat":"column-driveSeat",HobberFrame.sourcePose(face.frame),true);
      }
    drive(axis,HobberDrives.barMember("Z","fixed"),HobberDrives.barMember("Z","floating"),"Zplate",0);
    var scaleZ=HobberScales.z();
    group("Scale");
    externalPart("ZscaleSeat", new HobberScaleSeat("Z"), HobberFrame.partAt(0, 0, HobberFrame.PART_DATUM_Z), "foundation", "column-casting", scaleBarContact(scaleZ));
    var armFace = HobberScales.headFace(scaleZ);
    fixedPart("ZscaleArm", new HobberZScaleArm(), HobberFrame.partAt(0, 0, heights.hobCentreZ), "Zplate",
      F.alongY(-(armFace[1] - HobberScales.Z_ARM_THICKNESS / 2), 148, HobberFrame.PART_DATUM_Z + armFace[2], 0, 1, 0));
    scale(scaleZ,"ZscaleSeat","ZscaleArm");
    installZBellows();

  }
  /** Thin nested cloth is internal to one bought cover, with its own radial
   * clearance rather than the machine's 2 mm running gap. Only grant those
   * internal pairs after actual-solid zero-overlap checks at both overtravel
   * limits and mid travel. External guard clearances remain machine checks. */
  public function internalContacts(machine:MachineAssembly):Array<{a:String,b:String}> {
    var out:Array<{a:String,b:String}>=[],poses=machine.solvedPoses(),entries=machine.components();
    PosedParts.scope(cache->{
      for(cover in zBellows) {
        var ids=cover.members();
        for(q in [layout.zBellowsLowerLimit(),0.0,layout.zBellowsUpperLimit()]) {
          var shapes:Array<cadkit.modeling.Part>=[];
          for(i in 0...ids.length) {
            var id=ids[i],entry=entries.filter(m->m.id==id)[0],pose=poses.get(id);
            if(pose==null)throw "Missing Z bellows pose: "+id;
            var ratio=i==0?0.0:i==ids.length-1?1.0:i*1.0/cover.plan.levels;
            shapes.push(cache.posed(entry.component,F.compose(F.translation(0,0,ratio*q),pose)));
          }
          for(i in 0...shapes.length)for(j in i+1...shapes.length) {
            var volume=PosedParts.commonVolume(shapes[i],PosedParts.boxOf(shapes[i]),shapes[j],PosedParts.boxOf(shapes[j]));
            if(volume>1e-5)throw 'Z bellows internal overlap: ${ids[i]} / ${ids[j]} at $q: $volume mm3';
          }
          PosedParts.closeAll(shapes);
        }
        for(i in 0...ids.length)for(j in i+1...ids.length)out.push({a:ids[i],b:ids[j]});
      }
    });
    return out;
  }

  /** Bellows flanges and loose skins use the shared coupled-cover builder. */
  function installZBellows():Void {
    var p=layout.zBellowsPlan();
    for(side in [-1,1])for(upper in [false,true]) {
      var id="Zbellows"+(side<0?"Left":"Right")+(upper?"Upper":"Lower"),moving=id+"MovingMount",fixed=id+"FixedMount";
      group("Protection/"+id+"/Mounts");
      var movingPose=HobberFrame.sourcePose(layout.zBellowsPose(side,upper,false));
      fixedPart(moving,layout.zBellowsMovingMount(),movingPose,"Zplate");zBellowsMounts.push(moving);
      if(upper) {
        var mount=layout.zBellowsFixedUpperMount(),pose=HobberFrame.sourcePose(layout.zBellowsPose(side,true,true));
        var face=mount.fasteningFaces().filter(f->f.id=="column")[0];
        var contact=F.compose(pose,face.frame);
        var rail=layout.rails("column",layout.foundationPlacements("column")).filter(r->side*r.contact.y>0)[0];
        externalPart(fixed,mount,pose,"foundation","column-"+rail.insertId,contact);zBellowsMounts.push(fixed);
      }
      var origin=HobberFrame.sourcePose(layout.zBellowsPose(side,upper,true)),cover=machinekit.cover.WayCover.installOn(this,this,{
        id:id,row:p.row.id,fixedParent:upper?fixed:"Xbridge",movingParent:moving,
        leader:{id:"Z",lower:layout.zBellowsLowerLimit(),upper:layout.zBellowsUpperLimit(),initial:0.0},
        origin:[origin.x,origin.y,origin.z],along:[0.0,0,upper?-1.0:1.0],up:[0.0,-1,0],width:p.width,height:p.height,space:p.space,shape:p.shape});
      zBellows.push(cover);
      var last=cover.skinMembers[cover.skinMembers.length-1];
      addMemberConnector(last,"end-guide",F.compose(F.inverse(constructionPose(last)),movingPose));
      addMemberConnector(cover.movingMember,"end-guide",F.compose(F.inverse(constructionPose(cover.movingMember)),movingPose));
      addConstraintOnAxis(id+"-end-guide","prismatic",last,"end-guide",cover.movingMember,"end-guide",{x:0.0,y:0,z:1});
    }
  }
  public function axisDirection(id:String):Array<Float> {if(id!="Z")throw "Unknown cover axis: "+id;return [0.0,0,1];}
  public function attach(id:String,component:MachineComponent,pose:AssemblyFrame,parent:String):Void {
    if(Std.isOfType(component,machinekit.cover.WayCoverSection)) {
      group("Protection/"+id.substr(0,id.lastIndexOf("-"))+"/"+(StringTools.endsWith(id,"-fixed")?"Fixed end":"Moving end"));
      component=new HobberZBellowsPart(component.values());
    }
    if(parent=="Xbridge") externalPart(id,component,pose,"xAxis","carriage");
    else fixedPart(id,component,pose,parent);
  }
  public function hang(id:String,component:MachineComponent,pose:AssemblyFrame,parent:String):Void {
    if(Std.isOfType(component,machinekit.cover.WayCoverSection)) {
      group("Protection/"+id.substr(0,id.lastIndexOf("-"))+"/Folds");
      component=new HobberZBellowsPart(component.values());
    }
    addComponent(id,component,pose);
    var meeting=F.translation(pose.x,pose.y,pose.z);
    addMemberConnector(parent,'to-$id',F.compose(F.inverse(constructionPose(parent)),meeting));
    addMemberConnector(id,'attach-$id',F.compose(F.inverse(pose),meeting));
  }


}
