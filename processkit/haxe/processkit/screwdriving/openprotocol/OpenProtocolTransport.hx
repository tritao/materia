package processkit.screwdriving.openprotocol;

import haxe.io.Bytes;

/** Nonblocking owned byte transport, also implemented by scripted controllers. */
interface OpenProtocolTransport {
  function send(bytes:Bytes):Void;
  function receive():Bytes;
  function close():Void;
}
