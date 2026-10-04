package tests;
/** Public test-only credentials matching fixtures/authorization.json. */
class IntegrationCredentials {
  public static function controller():robotkit.auth.ClientCredentials
    return new robotkit.auth.ClientCredentials("test-controller","robotkit-test-controller-token-0001");
  public static function observer():robotkit.auth.ClientCredentials
    return new robotkit.auth.ClientCredentials("test-observer","robotkit-test-observer-token-0001");
}
