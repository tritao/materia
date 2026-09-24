package robotkit.material;

/** Immutable observed load condition; unknown is distinct from a confirmed empty fork. */
class LoadState {
  public final payload:Null<Payload>;
  public final observed:Bool;
  public final secured:Bool;

  public function new(payload:Null<Payload>, observed:Bool, secured:Bool) {
    if (!observed && (payload != null || secured))
      throw "An unobserved load state cannot claim payload or restraint";
    if (secured && payload == null)
      throw "A secured load state requires a payload";
    this.payload = payload;
    this.observed = observed;
    this.secured = secured;
  }

  public static function unknown():LoadState return new LoadState(null, false, false);
  public static function empty():LoadState return new LoadState(null, true, false);
  public static function detected(payload:Payload):LoadState {
    if (payload == null) throw "Detected load state requires a payload";
    return new LoadState(payload, true, false);
  }

  public static function carried(payload:Payload):LoadState {
    if (payload == null) throw "Carried load state requires a payload";
    return new LoadState(payload, true, true);
  }
}
