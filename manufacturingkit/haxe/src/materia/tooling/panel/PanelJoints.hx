package materia.tooling.panel;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ToolingEnums.CoatedFace;
import materia.tooling.ToolingEnums.StockDirection;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.ToolingEnums.ContactFace;
import materia.tooling.panel.PanelPlan.PanelOperation;
import materia.tooling.panel.PanelPlan.PanelOperationKind;
import materia.tooling.panel.PanelPlan.PanelSegment;
import materia.tooling.ToolingPrism.ToolingPoint2;

enum abstract PanelJointKind(String) to String {var TabSlot="tab-slot";var Rebate="rebate";var PinBolt="pin-bolt";}
enum abstract PanelClosure(String) to String {var ExternalClamps="external-clamps";var ThroughBolt="through-bolt";}
typedef PanelJoint = {
    var id:String;var kind:PanelJointKind;var fixedId:String;var movingId:String;
    var fixedFrame:AssemblyFrame;var movingFrame:AssemblyFrame;
    /** Total finished opening minus tab/pin size, not a per-side allowance. */
    var clearance:Float;var closure:PanelClosure;
}
typedef PanelJointPair={var fixed:PanelPlan;var moving:PanelPlan;var joint:PanelJoint;}
typedef TabJointSpec={
    var id:String;var fixedStock:PanelStockSpec;var movingStock:PanelStockSpec;
    var fixedWidth:Float;var fixedHeight:Float;var movingWidth:Float;var bodyHeight:Float;
    var offsetX:Float;var slotY:Float;var tabs:Array<{center:Float,width:Float}>;
    var clearance:Float;var routerDiameter:Float;var minimumLigament:Float;
}
typedef PinBoltLayout={
    var width:Float;var height:Float;
    var primary:ToolingPoint2;var secondary:ToolingPoint2;var bolt:ToolingPoint2;
}

/** One constructor owns both finished mating geometries and their assembly datums. */
class PanelJoints {
    public static function stock(thickness:Float=18):PanelStockSpec return {
        materialSpec:"birch plywood",nominalThickness:18,measuredThickness:thickness,thicknessVariation:0.1,
        width:2440,height:1220,coatedFace:CoatedFace.Both,allowedRotations:[0,90],directionConstraint:StockDirection.Free};

    public static function operation(id:String,kind:PanelOperationKind,contour:Array<PanelSegment>,depth:Float,tool:Float,
            side:MachiningSide=MachiningSide.PositiveZ,through:Bool=true,?relief:Float):PanelOperation
        return {id:id,kind:kind,contour:contour,side:side,setupId:side==MachiningSide.PositiveZ?"front":"back",
            depth:depth,through:through,toolDiameter:tool,reliefToolDiameter:relief};

    public static function blank(id:String,width:Float,height:Float,stock:PanelStockSpec,router:Float=6,ligament:Float=6):PanelPlan
        return PanelCodec.snapshot({id:id,revision:1,stock:stock,
            operations:[operation("outline",OutsideProfile,PanelContours.rectangle(0,0,width,height),stock.measuredThickness,router)],
            minimumLigament:ligament,contactFace:ContactFace.None,labelSide:MachiningSide.PositiveZ,labelX:width/2,labelY:height-12});

    public static function tabSlot(s:TabJointSpec,roundedTips:Bool=false):PanelJointPair {
        if(s.tabs==null || s.tabs.length<1 || s.tabs.length>16 || !Math.isFinite(s.clearance) || s.clearance<0)throw "Invalid tab joint";
        var fixed=blank(s.id+"-bed",s.fixedWidth,s.fixedHeight,s.fixedStock,s.routerDiameter,s.minimumLigament);
        var t=s.fixedStock.measuredThickness,mt=s.movingStock.measuredThickness;
        var moving=blank(s.id+"-wall",s.movingWidth,s.bodyHeight+t,s.movingStock,s.routerDiameter,s.minimumLigament);
        var tabs=s.tabs.copy();tabs.sort((a,b)->Reflect.compare(a.center,b.center));
        var points=[{x:0.0,y:t}],last=0.0;
        for(i in 0...tabs.length) {
            var tab=tabs[i],left=tab.center-tab.width/2,right=tab.center+tab.width/2;
            if(!Math.isFinite(tab.center) || !Math.isFinite(tab.width) || tab.width<s.routerDiameter+s.minimumLigament ||
                left-last<s.minimumLigament+s.routerDiameter || s.movingWidth-right<s.minimumLigament+s.routerDiameter)
                throw "Tab leaves insufficient root or edge ligament";
            points.push({x:left,y:t});points.push({x:left,y:0});points.push({x:right,y:0});points.push({x:right,y:t});last=right;
            fixed.operations.push(operation('slot-$i',InsideProfile,
                PanelContours.relievedRectangle(s.offsetX+tab.center,s.slotY,tab.width+s.clearance,mt+s.clearance,s.routerDiameter/2),
                t,s.routerDiameter,MachiningSide.PositiveZ,true,s.routerDiameter));
        }
        points.push({x:s.movingWidth,y:t});points.push({x:s.movingWidth,y:t+s.bodyHeight});points.push({x:0,y:t+s.bodyHeight});
        moving.operations=[operation("outline",OutsideProfile,PanelContours.relievedPolygon(points,s.routerDiameter/2,roundedTips),mt,
            s.routerDiameter,MachiningSide.PositiveZ,true,s.routerDiameter)];
        var joint:PanelJoint={id:s.id,kind:TabSlot,fixedId:fixed.id,movingId:moving.id,clearance:s.clearance,closure:ExternalClamps,
            fixedFrame:F.fromRotationMatrix(s.offsetX+tabs[0].center,s.slotY,t/2,[1,0,0, 0,0,-1, 0,1,0]),
            movingFrame:F.translation(tabs[0].center,t/2,mt/2)};
        return {fixed:PanelCodec.snapshot(fixed),moving:PanelCodec.snapshot(moving),joint:joint};
    }
    public static function rebate(id:String,stock:PanelStockSpec,width:Float=160,height:Float=100,wallHeight:Float=90,
            clearance:Float=0.2,router:Float=6,ligament:Float=6):PanelJointPair {
        if(!Math.isFinite(clearance) || clearance<0)throw "Invalid rebate clearance";
        var t=stock.measuredThickness,depth=t/2;
        if(depth<ligament || t<router)throw "Rebate leaves insufficient shelf";
        var fixed=blank(id+"-shelf",width,height,stock,router,ligament);
        fixed.operations.push(operation("edge-rebate",Rebate,PanelContours.rectangle(width-t-clearance,0,t+clearance,height),depth,router,MachiningSide.PositiveZ,false));
        fixed.contactFace=ContactFace.NegativeZ;
        var moving=blank(id+"-return",height,wallHeight,stock,router,ligament);
        var joint:PanelJoint={id:id,kind:PanelJointKind.Rebate,fixedId:fixed.id,movingId:moving.id,clearance:clearance,closure:ExternalClamps,
            fixedFrame:F.fromRotationMatrix(width-t,0,depth,[0,0,1, 1,0,0, 0,1,0]),movingFrame:F.identity()};
        return {fixed:PanelCodec.snapshot(fixed),moving:PanelCodec.snapshot(moving),joint:joint};
    }
    public static function pinBolt(id:String,stock:PanelStockSpec,pinDiameter:Float=6,clearance:Float=0.15,
            secondaryTravel:Float=2,boltHole:Float=7,router:Float=6,ligament:Float=6,?layout:PinBoltLayout):PanelJointPair {
        if(!Math.isFinite(clearance) || clearance<0 || !Math.isFinite(secondaryTravel) || secondaryTravel<=0 ||
            !Math.isFinite(pinDiameter) || pinDiameter<=0 || !Math.isFinite(boltHole) || boltHole<=0)throw "Invalid pin/bolt fit";
        var l=layout==null?{width:180.0,height:110.0,primary:{x:40.0,y:35.0},secondary:{x:140.0,y:35.0},bolt:{x:90.0,y:65.0}}:layout;
        var dx=l.secondary.x-l.primary.x,dy=l.secondary.y-l.primary.y;
        if(!Math.isFinite(dx) || !Math.isFinite(dy) || Math.sqrt(dx*dx+dy*dy)<1e-7)throw "Pin centers must be distinct";
        var angle=Math.atan2(dy,dx);
        var fixed=blank(id+"-lower",l.width,l.height,stock,router,ligament),moving=blank(id+"-upper",l.width,l.height,stock,router,ligament),t=stock.measuredThickness;
        for(p in [fixed,moving]) {
            var d=pinDiameter+clearance;
            p.operations.push(operation("primary-pin",Drill,PanelContours.circle(l.primary.x,l.primary.y,d/2),t,d));
            p.operations.push(operation("secondary-pin",p==fixed?Drill:InsideProfile,p==fixed?PanelContours.circle(l.secondary.x,l.secondary.y,d/2):
                PanelContours.placed(PanelContours.obround(0,0,d+secondaryTravel,d),angle,l.secondary.x,l.secondary.y),t,p==fixed?d:router));
            p.operations.push(operation("closure-bolt",Drill,PanelContours.circle(l.bolt.x,l.bolt.y,boltHole/2),t,boltHole));
        }
        var joint:PanelJoint={id:id,kind:PinBolt,fixedId:fixed.id,movingId:moving.id,clearance:clearance,closure:ThroughBolt,
            fixedFrame:F.translation(l.primary.x,l.primary.y,t),movingFrame:F.translation(l.primary.x,l.primary.y,0)};
        return {fixed:PanelCodec.snapshot(fixed),moving:PanelCodec.snapshot(moving),joint:joint};
    }
    public static function movingPose(joint:PanelJoint,fixedPose:AssemblyFrame):AssemblyFrame
        return F.compose(fixedPose,F.compose(joint.fixedFrame,F.inverse(joint.movingFrame)));
}
