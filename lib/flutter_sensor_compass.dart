library flutter_sensor_compass;

import 'dart:async';
import 'dart:math';
import 'package:vector_math/vector_math.dart';
import 'package:flutter_sensors/flutter_sensors.dart';

part 'src/compass.dart';

class Compass {
  static final _Compass _compass = _Compass();

  ///Returns a stream to receive the compass updates.
  ///
  ///Remember to close the stream after using it.
  static Stream<double> compasUpdates() => _compass.getCompassStream;

  /// Checks if the sensors needed for the compass to work are available.
  ///
  /// Returns true if the sensors are available or false otherwise.
  static Future<bool> isCompassAvailable() => _Compass.checkAviability;
}
