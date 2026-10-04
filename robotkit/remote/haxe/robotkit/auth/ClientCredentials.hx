package robotkit.auth;
/** Explicit bearer credentials. They are never synthesized by a client or server. */
class ClientCredentials {
  public final identity:String;
  public final token:String;
  public function new(identity:String, token:String) {
    if (identity == null || StringTools.trim(identity).length == 0 || token == null || token.length < 16)
      throw "Robot credentials require an identity and a token of at least 16 characters";
    this.identity = identity; this.token = token;
  }
}
