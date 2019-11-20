part of flutter_sensor_compass;

class _Compass {
  static const double _alpha = 0.97;
  List<double> _gravity = List.filled(3, 0.0);
  List<double> _geomagnetic = List.filled(3, 0.0);
  List<double> _rotationMatrix = List.filled(9, 0.0);
  double _azimuth = 0.0;
  double azimuthFix = 0.0;
  final List<_CompassStreamSubscription> _updatesSubscriptions = [];
  StreamSubscription<SensorEvent> _sensorStream;
  StreamController<double> _internalUpdateController =
      StreamController.broadcast();

  Stream<double> compassUpdates(Duration delay, double azimuthFix) {
    this.azimuthFix = azimuthFix ?? this.azimuthFix;
    StreamController<double> compassStreamController;
    _CompassStreamSubscription compassStreamSubscription;
    // ignore: cancel_subscriptions
    StreamSubscription<double> compassSubscription =
        _internalUpdateController.stream.listen((value) {
      if (delay != null) {
        DateTime instant = DateTime.now();
        int difference = instant
            .difference(compassStreamSubscription.lastUpdated)
            .inMicroseconds;
        if (difference < delay.inMicroseconds) {
          return;
        } else {
          compassStreamSubscription.lastUpdated = instant;
        }
      }
      compassStreamController.add(value);
    });
    compassSubscription.onDone(() {
      _updatesSubscriptions.remove(compassStreamSubscription);
    });
    compassStreamSubscription = _CompassStreamSubscription(compassSubscription);
    _updatesSubscriptions.add(compassStreamSubscription);
    compassStreamController = StreamController<double>.broadcast(
      onListen: () {
        if (_sensorStream != null) return;
        _startSensors();
      },
      onCancel: () {
        compassStreamSubscription.subscription.cancel();
        _updatesSubscriptions.remove(compassStreamSubscription);
        if (_updatesSubscriptions.isEmpty) _stopSensors();
      },
    );
    return compassStreamController.stream;
  }

  static Future<bool> get isCompassAvailable async {
    bool advanced = await isAdvancedSensorAvailable;
    bool def = await isDefaultSensorsAvailable;
    return advanced || def;
  }

  static Future<bool> get isAdvancedSensorAvailable async {
    return SensorManager.isSensorAvailable(11);
  }

  static Future<bool> get isDefaultSensorsAvailable async {
    bool accelerometer =
        await SensorManager.isSensorAvailable(Sensors.ACCELEROMETER);
    bool magneticField =
        await SensorManager.isSensorAvailable(Sensors.MAGNETIC_FIELD);
    return accelerometer && magneticField;
  }

  void _startSensors() async {
    bool advanced = await isAdvancedSensorAvailable;
    bool def = await isDefaultSensorsAvailable;
    if (advanced) {
      _startAdvancedSensor();
    } else if (def) {
      _startDefaultSensors();
    }
  }

  void _startAdvancedSensor() {
    _sensorStream = SensorManager.sensorUpdates(SensorRequest(
      11,
      refreshDelay: Sensors.SENSOR_DELAY_UI,
    )).listen((event) {
      if (Platform.isAndroid) {
        _computeRotationMatrixFromVector(event.data);
        List<double> orientation = _computeOrientation();
        _azimuth = degrees(orientation[0]);
        _azimuth = (_azimuth + azimuthFix + 360) % 360;
      } else if (Platform.isIOS) {
        _azimuth = event.data[0];
      }
      _internalUpdateController.add(_azimuth);
    });
  }

  void _startDefaultSensors() {
    _sensorStream = SensorManager.sensorsUpdates([
      SensorRequest(Sensors.ACCELEROMETER,
          refreshDelay: Sensors.SENSOR_DELAY_UI),
      SensorRequest(Sensors.MAGNETIC_FIELD,
          refreshDelay: Sensors.SENSOR_DELAY_UI),
    ]).listen((event) {
      switch (event.sensor) {
        case Sensors.ACCELEROMETER:
          _gravity[0] = _alpha * _gravity[0] + (1 - _alpha) * event.data[0];
          _gravity[1] = _alpha * _gravity[1] + (1 - _alpha) * event.data[1];
          _gravity[2] = _alpha * _gravity[2] + (1 - _alpha) * event.data[2];
          break;
        case Sensors.MAGNETIC_FIELD:
          _geomagnetic[0] =
              _alpha * _geomagnetic[0] + (1 - _alpha) * event.data[0];
          _geomagnetic[1] =
              _alpha * _geomagnetic[1] + (1 - _alpha) * event.data[1];
          _geomagnetic[2] =
              _alpha * _geomagnetic[2] + (1 - _alpha) * event.data[2];
          break;
      }
      if (_computeRotationMatrix()) {
        List<double> orientation = _computeOrientation();
        _azimuth = degrees(orientation[0]);
        _azimuth = (_azimuth + azimuthFix + 360) % 360;
        _internalUpdateController.add(_azimuth);
      }
    });
  }

  void _stopSensors() {
    if (_sensorStream == null) return;
    _sensorStream.cancel();
    _sensorStream = null;
  }

  /// Updates the current rotation matrix using the values gathered by the
  /// accelerometer and magnetic field sensor.
  ///
  /// Returns true if the computation was successful and false otherwise.
  bool _computeRotationMatrix() {
    double ax = _gravity[0], ay = _gravity[1], az = _gravity[2];
    double ex = _geomagnetic[0], ey = _geomagnetic[1], ez = _geomagnetic[2];
    double hx = ey * az - ez * ay;
    double hy = ez * ax - ex * az;
    double hz = ex * ay - ey * ax;
    double normH = sqrt(hx * hx + hy * hy + hz * hz);
    if (normH < 0.1) return false;
    double invH = 1.0 / normH;
    hx *= invH;
    hy *= invH;
    hz *= invH;
    double invA = 1.0 / sqrt(ax * ax + ay * ay + az * az);
    ax *= invA;
    ay *= invA;
    az *= invA;
    double mx = ay * hz - az * hy;
    double my = az * hx - ax * hz;
    double mz = ax * hy - ay * hx;
    _rotationMatrix = [hx, hy, hz, mx, my, mz, ax, ay, az];
    return true;
  }

  /// Updates the current rotation matrix using the values gathered by the
  /// rotation vector sensor.
  ///
  /// Returns true if the computation was successful and false otherwise.
  void _computeRotationMatrixFromVector(List<double> rotationVector) {
    double q0;
    double q1 = rotationVector[0];
    double q2 = rotationVector[1];
    double q3 = rotationVector[2];
    if (rotationVector.length == 4) {
      q0 = rotationVector[3];
    } else {
      q0 = 1 - q1 * q1 - q2 * q2 - q3 * q3;
      q0 = (q0 > 0) ? sqrt(q0) : 0;
    }
    double sqQ1 = 2 * q1 * q1;
    double sqQ2 = 2 * q2 * q2;
    double sqQ3 = 2 * q3 * q3;
    double q1Q2 = 2 * q1 * q2;
    double q3Q0 = 2 * q3 * q0;
    double q1Q3 = 2 * q1 * q3;
    double q2Q0 = 2 * q2 * q0;
    double q2Q3 = 2 * q2 * q3;
    double q1Q0 = 2 * q1 * q0;
    _rotationMatrix[0] = 1 - sqQ2 - sqQ3;
    _rotationMatrix[1] = q1Q2 - q3Q0;
    _rotationMatrix[2] = q1Q3 + q2Q0;
    _rotationMatrix[3] = q1Q2 + q3Q0;
    _rotationMatrix[4] = 1 - sqQ1 - sqQ3;
    _rotationMatrix[5] = q2Q3 - q1Q0;
    _rotationMatrix[6] = q1Q3 - q2Q0;
    _rotationMatrix[7] = q2Q3 + q1Q0;
    _rotationMatrix[8] = 1 - sqQ1 - sqQ2;
  }

  /// Compute the orientation utilizing the data realized by the
  /// [_computeRotationMatrix] method.
  ///
  /// * [rotationMatrix] the rotation matrix to calculate the orientation.
  ///
  /// Returns a list with the result of the orientation.
  List<double> _computeOrientation() {
    List<double> orientation = List(3);
    orientation[0] = atan2(_rotationMatrix[1], _rotationMatrix[4]);
    orientation[1] = asin(-_rotationMatrix[7]);
    orientation[2] = atan2(-_rotationMatrix[6], _rotationMatrix[8]);
    return orientation;
  }
}

class _CompassStreamSubscription {
  StreamSubscription subscription;
  DateTime lastUpdated;

  _CompassStreamSubscription(this.subscription) {
    this.lastUpdated = DateTime.now();
  }
}
