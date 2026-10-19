package materia.tooling.panel;

import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.panel.PanelPlan.PanelOperationKind;

typedef PanelCouponPlan={var receiver:PanelPlan;var plug:PanelPlan;var clearances:Array<Float>;}
/** Neutral fit coupon: a shared plug and slots marked by total fit allowance. */
class PanelCoupon {
    public static function make(stock:PanelStockSpec,clearances:Array<Float>,router:Float=6):PanelCouponPlan {
        if(clearances==null || clearances.length<1 || clearances.length>8)throw "Invalid coupon clearance count";
        var pair=PanelJoints.tabSlot({id:"coupon",fixedStock:stock,movingStock:stock,
            fixedWidth:clearances.length*50+20,fixedHeight:100,movingWidth:60,bodyHeight:50,
            offsetX:5,slotY:40,tabs:[{center:30.0,width:30.0}],clearance:clearances[0],routerDiameter:router,minimumLigament:6});
        pair.fixed.operations=pair.fixed.operations.filter(o->o.kind==OutsideProfile);
        var used:Array<Float> = [];
        for(i in 0...clearances.length) {
            var c=clearances[i];if(!Math.isFinite(c) || c<0 || used.indexOf(c)>=0)throw "Invalid or duplicate coupon clearance";
            used.push(c);
            pair.fixed.operations.push(PanelJoints.operation('fit-$i',InsideProfile,
                PanelContours.relievedRectangle(35+50*i,40,30+c,stock.measuredThickness+c,router/2),
                stock.measuredThickness,router,MachiningSide.PositiveZ,true,router));
        }
        return {receiver:PanelCodec.snapshot(pair.fixed),plug:pair.moving,clearances:used};
    }
}
