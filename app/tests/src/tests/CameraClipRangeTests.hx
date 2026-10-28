package tests;

import nativekit.scene.PerspectiveCamera;

/** Off-screen exploded parts must not consume the visible model's depth precision. */
class CameraClipRangeTests {
  static var assertions=0;
  static function check(value:Bool,message:String):Void {
    assertions++;if(!value)throw message;
  }
  public static function main():Int {
    var camera=new PerspectiveCamera();
    for(yaw in [0.0,0.7,13.241878906249955])for(pitch in [-0.8,0.0,0.46328857421874864]) {
      camera.setAngle(yaw,pitch);camera.setTarget(-0.838,0.073,0.176,0.05);
      var eye=camera.eyePosition(),back=camera.viewDirection(),right=[-Math.sin(yaw),Math.cos(yaw),0.0];
      function box(depth:Float,side:Float,radius:Float):Array<Float> {
        var center=[for(i in 0...3)eye[i]-back[i]*depth+right[i]*side];
        return [center[0]-radius,center[1]-radius,center[2]-radius,
          center[0]+radius,center[1]+radius,center[2]+radius];
      }
      var visible=box(20,0,0.5),crossing=box(0,100,0.5),distant=box(1000,10000,0.5);
      for(aspect in [0.5,1.0,2.0]) {
        camera.fitClipRange([visible],aspect);var near=camera.clipNear,far=camera.clipFar;
        camera.fitClipRange([visible,crossing,distant,box(-10,0,0.5)],aspect);
        check(Math.abs(camera.clipNear-near)<1e-9&&Math.abs(camera.clipFar-far)<1e-9,
          "off-screen/behind-eye boxes preserve the visible depth range");
      }
      camera.fitClipRange([visible,crossing]);
      check(camera.clipNear<0.00001,"fixture straddles the eye in depth without entering the view");
      var side=box(10,8,0.2);
      camera.fitClipRange([visible,side],1);var narrowNear=camera.clipNear;
      camera.fitClipRange([visible,side],2);
      check(camera.clipNear<narrowNear*0.75,"wide aspect retains side geometry");
      camera.fitClipRange([visible,side],1,90);
      check(camera.clipNear<narrowNear*0.75,"inspection lens FOV retains side geometry");
      camera.fitClipRange([visible,box(10,Math.tan(25*Math.PI/180)*10,1)],1);
      check(camera.clipNear<narrowNear*0.75,"boxes crossing a side plane stay included");
      camera.fitClipRange([visible,box(0,0,0.5)],1);
      check(camera.clipNear<0.00001,"camera inside an on-screen box retains nearby geometry");
      camera.fitClipRange([crossing],1);
      check(camera.clipNear>0&&camera.clipFar>camera.clipNear,"fully off-screen scene keeps valid clipping");
    }
    // The saved column-mold camera, its off-screen bed, and two visible former bounds.
    camera.setAngle(13.241878906249955,0.46328857421874864);
    camera.setTarget(-0.8380172100843716,0.0726464048317293,0.1761653914041284,0.05);
    camera.fitClipRange([
      [-0.266,-0.879,-0.022,0.266,0.046,-0.004],
      [-1.0304913847,-0.0899,0.0271,-0.9506913847,-0.0101,0.0929],
      [-1.0304913847,-0.0899,0.1671,-0.9506913847,-0.0101,0.2329]
    ],1085.0/689.0);
    check(camera.clipNear>0.1&&camera.clipFar<0.5,"saved mold view keeps its useful depth precision");
    Sys.println('Camera clip range: $assertions assertions passed');return 0;
  }
}
