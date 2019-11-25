library flutter_sensor_compass;

import 'dart:async';
import 'dart:io';
import 'dart:math';

import 'package:flutter_sensors/flutter_sensors.dart';
import 'package:vector_math/vector_math.dart';

part 'src/compass.dart';

class Compass {
  static final _Compass _compass = _Compass();

  ///Returns a stream to receive the compass updates.
  ///
  ///Remember to close the stream after using it.
  static Stream<double> compassUpdates(
          {Duration interval, double azimuthFix}) =>
      _compass.compassUpdates(interval, azimuthFix);

  /// Checks if the sensors needed for the compass to work are available.
  ///
  /// Returns true if the sensors are available or false otherwise.
  static Future<bool> isCompassAvailable() => _Compass.isCompassAvailable;

  static void setAzimuthFix(double fix) => _compass.azimuthFix = fix;
}
