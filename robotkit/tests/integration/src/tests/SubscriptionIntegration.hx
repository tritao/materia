package tests;

import robotkit.client.RobotClient;
import NativeKitRuntime;

/** Two observers request different RKF1 families from one real robotd. */
class SubscriptionIntegration {
  public static function run(host:String, port:Int):Void {
    var runtime = NativeKitRuntime.start();
    var numeric = new RobotClient("numeric-only", "observer");
    numeric.subscribeCamera = false;
    var images = new RobotClient("camera-reader", "observer");
    images.subscribeCamera = true;
    var numericFrames = 0;
    var numericCameras = 0;
    var imageCameras = 0;
    numeric.sensorListener = function(_) numericFrames++;
    numeric.cameraListener = function(_) numericCameras++;
    images.cameraListener = function(_) imageCameras++;
    var failure:Dynamic = null;
    try {
      numeric.connectWithEvents(host, port, runtime.events);
      images.connectWithEvents(host, port, runtime.events);
      var deadline = Sys.time() + 5.0;
      while (Sys.time() < deadline && (numericFrames == 0 || imageCameras == 0)) {
        numeric.poll();
        images.poll();
        numeric.wait(0.01);
        images.wait(0.01);
      }
      // Continue polling so an incorrectly delivered camera frame cannot hide
      // behind the first numeric frame.
      var settle = Sys.time() + 0.2;
      while (Sys.time() < settle) {
        numeric.poll();
        images.poll();
        numeric.wait(0.01);
      }
      if (numericFrames == 0 || imageCameras == 0 || numericCameras != 0)
        throw 'RKF1 subscription filtering failed: numeric=$numericFrames camera=$imageCameras filtered=$numericCameras';
      var welcome = numeric.welcome;
      if (welcome == null || welcome.capabilities.indexOf("essential") < 0 ||
          welcome.capabilities.indexOf("sensor") < 0 ||
          welcome.capabilities.indexOf("camera") < 0)
        throw "RKF1 Welcome stream capabilities are incomplete";
      Sys.println("robotd subscriptions filtered camera per observer and reported capabilities");
    } catch (error:Dynamic) failure = error;
    images.close();
    numeric.close();
    runtime.dispose();
    if (failure != null) throw failure;
  }
}
