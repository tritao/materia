package processkit.moonraker;

import materia.assembly.BeltCalibration.ResonanceMeasurement;
import processkit.calibration.KlipperResonance;

/** Read recorded artifacts through Moonraker. Acquisition/movement and publishing /tmp files are explicit external steps.
 * Never run SHAPER_CALIBRATE or SAVE_CONFIG as a side effect of importing measurements. */
class ResonanceImport {
  public static function read(client:MoonrakerClient,root:String,spectrum:String,decay:String,axis:String,
      start:Float,end:Float,complete:ResonanceMeasurement->Void):Void {
    client.downloadFile(root,spectrum,psd -> {
      client.downloadFile(root,decay,raw -> complete(KlipperResonance.measure(axis,psd,raw,start,end,
        root+"/"+spectrum,root+"/"+decay)));
    });
  }
}
