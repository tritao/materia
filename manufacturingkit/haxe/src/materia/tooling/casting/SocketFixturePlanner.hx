package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelJoints.PanelJointPair;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelPlan.PanelOperationKind;
import materia.tooling.ToolingValidation;

/** Explicit blind-socket contract. No CAD dependency and no reinterpretation as a through-bored box. */
typedef BlindSocketSpec = {
  var length:Float; var outsideDiameter:Float; var boreDepth:Float; var threadLength:Float;
  var tailWidth:Float; var tailThickness:Float; var tailWave:Float;
  var thread:String; var source:String; var exteriorVerified:Bool;
}
typedef SocketFixtureSite = {
  var id:String; var x:Float; var y:Float;
  var mouth:AssemblyFrame;
  var screwLength:Float; var insertionMm:Float;
  var clearanceOperationId:String; var accessOperationId:String;
  var seal:{kind:String,footprintOutsideDiameter:Float,clearanceDiameter:Float,materialSpec:Null<String>,qualified:Bool};
}
typedef SocketFixturePlan = {
  var revision:Int; var socket:BlindSocketSpec; var plates:PanelJointPair;
  var cavity:{x:Float,y:Float,z:Float,width:Float,depth:Float,height:Float};
  var sites:Array<SocketFixtureSite>;
  var primary:{x:Float,y:Float}; var secondary:{x:Float,y:Float};
  var clampPoints:Array<{x:Float,y:Float}>;
  var pinDiameter:Float; var plateThickness:Float; var mouthZ:Float; var washerThickness:Float;
  var positionToleranceMm:Null<Float>;
  var validation:Array<ToolingValidation>;
}

class SocketFixturePlanner {
  public static function reference():BlindSocketSpec return {
    length:40,outsideDiameter:8,boreDepth:25,threadLength:6,tailWidth:11,tailThickness:2.2,tailWave:1.4,
    thread:"M6",source:"PFEIFER 05.250.064.040; outside diameter and crimp dimensions nominal",exteriorVerified:false
  };
  public static function coupon(?socket:BlindSocketSpec,thickness:Float=18,screwLength:Float=30,washerThickness:Float=1.6):SocketFixturePlan {
    var s=socket==null?reference():socket;
    for(n in [s.length,s.outsideDiameter,s.boreDepth,s.threadLength,s.tailWidth,s.tailThickness,s.tailWave,thickness,screwLength,washerThickness])
      if(!Math.isFinite(n)||n<=0)throw "Socket fixture dimensions must be positive";
    if(s.thread!="M6"||s.outsideDiameter<=6.4||s.threadLength>=s.boreDepth||s.boreDepth>=s.length||s.length>=70||
      s.tailWidth<s.outsideDiameter||s.tailThickness>=s.outsideDiameter)throw "Unsupported/inconsistent blind socket";
    var insertion=screwLength-thickness-washerThickness;
    if(insertion<s.threadLength||insertion>s.boreDepth-1)throw "Positioning screw has insufficient engagement or bottoms in blind socket";
    var primary={x:80.0,y:20.0},secondary={x:420.0,y:20.0},clamps=[{x:80.0,y:320.0},{x:420.0,y:320.0}];
    var pair=PanelJoints.pinBolt("socket-fixture",PanelJoints.stock(thickness),6,0.15,2,7,6,6,
      {width:500,height:340,primary:primary,secondary:secondary,bolt:clamps[0]});
    for(p in [pair.fixed,pair.moving]) {
      var op=PanelJoints.operation("second-closure-bolt",Drill,PanelContours.circle(clamps[1].x,clamps[1].y,3.5),thickness,7);
      op.through=true;p.operations.push(op);
    }
    var sites:Array<SocketFixtureSite> = [];
    for(i in 0...2) {
      var x=i==0?170.0:330.0,y=170.0,id="socket"+i,hole=id+"-fixing",access=id+"-driver-access";
      var fixing=PanelJoints.operation(hole,Drill,PanelContours.circle(x,y,3.2),thickness,6.4);fixing.through=true;
      pair.moving.operations.push(fixing);pair.moving.contactOpenings.push(hole);
      var clearance=PanelJoints.operation(access,Drill,PanelContours.circle(x,y,7),thickness,14);clearance.through=true;
      pair.fixed.operations.push(clearance);
      // Mouth faces the mould bed; closed tail points into the casting (+world Z).
      sites.push({id:id,x:x,y:y,mouth:F.fromRotationMatrix(x,y,2*thickness,[1,0,0,0,-1,0,0,0,-1]),
        screwLength:screwLength,insertionMm:insertion,clearanceOperationId:hole,accessOperationId:access,
        seal:{kind:"thin mouth-face/bolt-path seal; film not assigned a fictitious thickness",footprintOutsideDiameter:s.outsideDiameter,
          clearanceDiameter:6.4,materialSpec:null,qualified:false}});
    }
    var checks=[
      ToolingReadiness.pending("socket-exterior",["socket0","socket1"],"Supplier outside diameter/crimp dimensions remain nominal"),
      ToolingReadiness.pending("socket-seal",["socket0","socket1"],"Select and trial mouth seal/release system; no resin-side plug on closed tail"),
      ToolingReadiness.pending("locator-accuracy",["socket-fixture"],"Nominal 6 mm pins with 0.15 mm diametral fit clearance; physical accuracy/tolerance budget unqualified"),
      ToolingReadiness.pending("mould-closure",["coupon-walls"],"Wall clamping, resin pressure and leak qualification remain pending"),
      ToolingReadiness.pending("process-retention",["socket0","socket1"],"Validate compaction, cure and mineral-cast retention with actual materials")
    ];
    return {revision:1,socket:s,plates:pair,cavity:{x:50,y:50,z:2*thickness,width:400,depth:240,height:70},
      sites:sites,primary:primary,secondary:secondary,clampPoints:clamps,pinDiameter:6,plateThickness:thickness,
      mouthZ:2*thickness,washerThickness:washerThickness,positionToleranceMm:null,validation:checks};
  }
}
