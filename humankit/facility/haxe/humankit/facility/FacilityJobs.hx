package humankit.facility;

import materia.automation.facility.Facility;

/** Creates lane-following worker jobs from named facility targets. */
class FacilityJobs {
  public static function fetch(facility:Facility, rackId:String, slotId:String):FacilityFetchJob
    return new FacilityFetchJob(new FacilityTargets(facility), rackId, slotId);
}
