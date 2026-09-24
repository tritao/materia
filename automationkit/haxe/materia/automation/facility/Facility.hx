package materia.automation.facility;

/** Facility topology kept above RobotWorld and RobotKit's live robot model. */
class Facility {
  public final id:String;
  public final name:String;
  final zonesById = new Map<String,Zone>();
  final stationsById = new Map<String,Station>();
  final lanesById = new Map<String,Lane>();
  final racksById = new Map<String,Rack>();
  final chargersById = new Map<String,Charger>();

  public function new(id:String, name:String) {
    if (id == null || id.length == 0 || name == null || name.length == 0)
      throw "Facility requires an ID and name";
    this.id = id;
    this.name = name;
  }

  public function addZone(zone:Zone):Void {
    if (zone == null || zonesById.exists(zone.id)) throw "Facility zone ID is missing or duplicated";
    zonesById.set(zone.id, zone);
  }

  public function addStation(station:Station):Void {
    requireStation(station);
    stationsById.set(station.id, station);
  }

  public function addRack(rack:Rack):Void {
    requireStation(rack);
    racksById.set(rack.id, rack);
  }

  public function addCharger(charger:Charger):Void {
    requireStation(charger);
    chargersById.set(charger.id, charger);
  }

  public function addLane(lane:Lane):Void {
    if (lane == null || lanesById.exists(lane.id)) throw "Facility lane ID is missing or duplicated";
    var from = stationsById.get(lane.fromStationId);
    var to = stationsById.get(lane.toStationId);
    if (from == null || to == null) throw "Lane endpoints must exist in the facility";
    if (from.frameId != to.frameId || lane.centerline.frameId != from.frameId)
      throw "Lane and endpoint station frames must match";
    lanesById.set(lane.id, lane);
  }

  public function zone(id:String):Null<Zone> return zonesById.get(id);
  public function station(id:String):Null<Station> return stationsById.get(id);
  public function lane(id:String):Null<Lane> return lanesById.get(id);
  public function rack(id:String):Null<Rack> return racksById.get(id);
  public function charger(id:String):Null<Charger> return chargersById.get(id);

  public function zones():Array<Zone> return sortedValues(zonesById);
  public function stations():Array<Station> return sortedValues(stationsById);
  public function lanes():Array<Lane> return sortedValues(lanesById);

  function requireStation(station:Station):Void {
    if (station == null || stationsById.exists(station.id))
      throw "Facility station ID is missing or duplicated";
    if (!zonesById.exists(station.zoneId)) throw "Station zone must exist in the facility";
    var zone = zonesById.get(station.zoneId);
    if (zone.frameId != station.frameId) throw "Station and zone frames must match";
    stationsById.set(station.id, station);
  }

  static function sortedValues<T>(values:Map<String,T>):Array<T> {
    var keys = [for (key in values.keys()) key];
    keys.sort(Reflect.compare);
    return [for (key in keys) values.get(key)];
  }
}
