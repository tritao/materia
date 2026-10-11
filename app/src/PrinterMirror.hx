package app;

import processkit.moonraker.MoonrakerStatus;

/**
 * The digital twin of a real printer: the project's printer assembly shown where the machine's nozzle is. Klipper's
 * `motion_report.live_position` gives the nozzle's x, y and z in millimetres, which are the assembly's axis joints in its
 * units, and the extruder's position as millimetres of filament, which the extruder's gear turns into an angle. It is
 * the live position, not `toolhead.position`: that is the end of the last move queued, ahead of the nozzle.
 */
class PrinterMirror {
  /** Sets the printer's axes and extruder to where `status` says the machine is. */
  public static function follow(project:ProjectDocumentSession, status:MoonrakerStatus):Void {
    var job = project.printJob;
    if (job == null) throw "The project has no printer to mirror";
    var toUnits = 0.001 / project.assemblyUnitMetres();
    var values = new Map<String, Float>();
    for (axis in 0...3) values.set(job.axes[axis], status.position[axis] * toUnits);
    // The gear's radius is in metres and the extruder joint in radians, which are not lengths: filament travel over radius.
    values.set(job.extruder, status.position[3] * 0.001 / job.gearRadius);
    project.previewAssemblyJoints(values);
  }
}
