import machinekit.assembly.AssemblyPreview;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames as F;

/** Geometry and physical assembly acceptance for both full-sheet bed lengths. */
class SheetRouterChecks {
    static var assertions=0;
    static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw 'Sheet router: '+label;}
    static function near(a:Float,b:Float,label:String):Void check(Math.isFinite(a)&&Math.abs(a-b)<1e-6,label+': '+a+' vs '+b);
    static function mesh():Void {
        var machine=new SheetRouter(),definition=machine.definition(),state=new AssemblyState(definition);
        var wheel=machine.component("pinionX"),longRack=machine.component("rackX");
        var rack=new machinekit.transmission.HelicalRack(longRack.values().setInteger("teeth",80));
        var location=machine.rackPlacement({id:"x",lower:0.0,upper:0.0,initial:0.0},wheel,machine.zeroPose("pinionX"),[0.0,0.0,-1.0],rack);
        var pinion=wheel.geometry(),bar=rack.geometry();
        var placedRack=machinekit.assembly.PosedParts.moved(bar,location.pose);bar.close();
        try {
            for(step in 0...11) {
                state.setJoint("x",step*0.5);state.forwardKinematics();
                var placed=machinekit.assembly.PosedParts.moved(pinion,state.worldPose("pinionX")),common=placed.intersect(placedRack);
                var volume=common.volume();common.close();placed.close();
                Sys.println("SR3 tooth sweep "+step+": "+volume+" mm3");Sys.stdout().flush();
                check(volume<0.05,"helical tooth clearance over one pitch: "+volume);
            }
            pinion.close();placedRack.close();
        } catch(error:Dynamic) {pinion.close();placedRack.close();throw error;}
    }
    public static function run():Void {
        var short=new machinekit.transmission.HelicalRack(new machinekit.component.ComponentValues().setInteger("teeth",80)).geometry();
        var segmented=new machinekit.transmission.HelicalRack(new machinekit.component.ComponentValues().setInteger("teeth",81)).geometry();
        check(segmented.valid(),"segmented full-length rack is a valid compound");
        near(segmented.volume()/81,short.volume()/80,"whole-pitch rack segmentation preserves volume");short.close();segmented.close();
        for(length in [2600.0,3150.0]) {
            Sys.println('SR3 building '+length);Sys.stdout().flush();
            var machine=new SheetRouter(new SheetRouterSpec(length));
            machine.check().throwIfErrors();
            check(machine.component('blockYLeftRear')!=null&&machine.component('blockYRightRear')!=null,'two Y blocks per side');
            check(machine.component('railPadYLeft').materialSpec()=='steel','steel replication rail pads');
            var definition=machine.definition(),state=new AssemblyState(definition);
            var start=state.worldConnector('spindle','gauge');
            var samples:Array<Array<Float>> = [];
            for(x in [0.0,800.0,1600.0])for(y in [0.0,length/2,length,length+300])for(z in [0.0,125.0,250.0]) {
                state.setJoint('x',x);state.setJoint('y',y);state.setJoint('z',z);state.forwardKinematics();
                var gauge=state.worldConnector('spindle','gauge');
                near(gauge.x-start.x,x,'corner X');near(gauge.y-start.y,y,'corner Y');near(gauge.z-start.z,-z,'corner Z');
                if(y<=length)samples.push([x,y,z]);
            }
            for(i in 0...8) {
                var pocket=F.compose(machine.zeroPose('fork${i+1}'),machine.component('fork${i+1}').connector('pocket').frame);
                // Holder fork seat is 8 mm below gauge; travel from the open side at the same Z.
                var gauge=F.translation(pocket.x,pocket.y,pocket.z+8);
                var q=machine.gaugeCoordinates(gauge);
                state.setJoint('x',q[0]);state.setJoint('y',q[1]);state.setJoint('z',q[2]);state.forwardKinematics();
                var shoe=state.worldPose('shoe');
                check(shoe.z-machine.shoe.n('brushLength')>pocket.z,'raised shoe clears fork');
                samples.push(q);
                check(q[0]>=0&&q[0]<=1600&&q[1]>=0&&q[1]<=length+300&&q[2]>=0&&q[2]<=250,'rack pocket reachable '+i);
                var approach=F.compose(machine.zeroPose('fork${i+1}'),machine.component('fork${i+1}').connector('approach').frame);
                near(approach.x,pocket.x,'fork opens along Y');check(approach.y<pocket.y&&approach.y>machine.workOrigin.y+length+20,'fork approach stays beyond the full sheet');
                var entry=machine.gaugeCoordinates(F.translation(approach.x,approach.y,gauge.z));
                samples.push(entry);samples.push([q[0],(entry[1]+q[1])/2,q[2]]);samples.push([entry[0],entry[1],q[2]/2]);
            }
            check(machine.gaugeZero.z-machine.routerSpec.longestTool>machine.boardTop+machine.routerSpec.sheetThickness,'longest tool clears full sheet when raised');
            var beam=machine.zeroPose('beam');check(beam.z-100-machine.boardTop>=150,'gantry clears board by at least 150 mm');
            for(binding in machine.cableChainBindings) {
                check(binding.chain.n('travel')==(binding.axis=='x'?1600:length+300),'chain covers axis travel');
                for(position in [0.0,binding.chain.n('travel')]) {
                    var part=machine.cableChainGeometry(binding.axis,position);check(part.valid()&&part.volume()>0,'posed chain is valid');part.close();
                }
            }
            Sys.println('SR3 layout verified '+length);Sys.stdout().flush();
            if(Sys.args().indexOf("--layout-only")>=0)continue;
            var scene=AssemblyPreview.scene(machine,'sheet-router-$length',256);
            var compiled=scene.assemblyDefinition;if(compiled==null)throw "Sheet router has no compiled assembly";
            var model=cadbridge.AssemblySimulationBridge.toRobotModel(compiled,cadbridge.AssemblyPhysicalPartView.fromSceneArtifact(scene),scene.assemblyState).model;
            check(model.validate().length==0,"full physical model validates");
            Sys.println('SR3 scene compiled '+length);Sys.stdout().flush();
            if(Sys.args().indexOf("--skip-clearance")<0) {
                var clearance=new GantryClearanceChecks(machine,compiled,[
                    {a:"shoeLiftCylinder",b:"shoeLiftRod"},{a:"rackCoverLiftCylinder",b:"rackCoverLiftRod"}]);
                try {for(q in samples) {if(Sys.getEnv("MACHINEKIT_PREVIEW_TRACE")=="1") {Sys.println("SR3 clearance "+length+" "+q.join(","));Sys.stdout().flush();} state.setJoint('x',q[0]);state.setJoint('y',q[1]);state.setJoint('z',q[2]);state.forwardKinematics();clearance.check(state);assertions++;} clearance.close();}
                catch(error:Dynamic) {clearance.close();throw error;}
            }
            var mass=machine.massProperties(state);check(mass.mass>0&&Math.isFinite(mass.mass),'finite assembly mass');
            check(machine.billOfMaterials().lines().length>50,'complete assembly BOM');
            Sys.println('SR3 '+length+' mass '+mass.mass+' kg, BOM '+machine.billOfMaterials().lines().length+' lines');
        }
        Sys.println('Sheet router SR3: '+assertions+' assertions passed');
    }
    public static function main():Void {if(Sys.args().indexOf("--mesh-only")>=0)mesh();else run();}
}
