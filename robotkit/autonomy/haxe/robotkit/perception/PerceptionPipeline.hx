package robotkit.perception;

import robotkit.streams.ImageDetectionObservation;

import robotkit.core.SensorFrame;

interface PerceptionPipeline {
  function id():String;
  function sensorId():String;
  function submit(frame:SensorFrame):Void;
  function poll():Array<ImageDetectionObservation>;
  function dispose():Void;
}
