package app.editor;

import humankit.facility.FacilityWalk;
import materia.automation.facility.Facility;
import materia.automation.facility.FacilityRouter;
import materia.automation.facility.Lane;
import materia.automation.facility.Station;
import materia.automation.facility.Zone;
import robotkit.mobile.Footprint;
import robotkit.mobile.Pose2;

/**
 * A small built-in AutomationKit facility for the editor's character
 * preview, so --character-facility-route can show a person walking a real
 * FacilityRouter route without a project supplying its own facility. Two
 * lanes carry a dogleg from "dock" through "shelf" to "bench".
 */
class FacilityRouteDemo {
	static var facility:Null<Facility> = null;

	/** Floor points along the FacilityRouter route from fromStationId to toStationId. */
	public static function route(fromStationId:String, toStationId:String):Array<Array<Float>>
		return FacilityWalk.routeFromFacilityRoute(new FacilityRouter(demoFacility()).route(fromStationId,
			toStationId));

	/** The demo facility's station IDs, for usage and error messages. */
	public static function stationIds():Array<String>
		return [for (station in demoFacility().stations()) station.id];

	static function demoFacility():Facility {
		var existing = facility;
		if (existing != null)
			return existing;
		var built = new Facility("preview", "Preview facility");
		built.addZone(new Zone("floor", "Floor", "map", Footprint.rectangle(12.0, 12.0)));
		var dock = new Station("dock", "Dock", "floor", "map", new Pose2(0.0, 0.0, 0.0));
		var shelf = new Station("shelf", "Shelf", "floor", "map", new Pose2(3.0, 0.0, 0.0));
		var bench = new Station("bench", "Bench", "floor", "map", new Pose2(3.0, 2.5, Math.PI / 2));
		built.addStation(dock);
		built.addStation(shelf);
		built.addStation(bench);
		built.addLane(new Lane("dock-shelf", dock.id, shelf.id,
			new robotkit.navigation.Path([dock.pose, shelf.pose], "map"), 1.0, 1.4));
		built.addLane(new Lane("shelf-bench", shelf.id, bench.id,
			new robotkit.navigation.Path([shelf.pose, bench.pose], "map"), 1.0, 1.4));
		facility = built;
		return built;
	}
}
