part of flutter_sensor_compass;

class _Compass {
  List<double> _gravity = List.filled(3, 0.0);
  List<double> _geomagnetic = List.filled(3, 0.0);
  List<double> _rotationMatrix = List.filled(9, 0.0);
  double _azimuth = 0.0;
  double azimuthFix = 0.0;
  final List<_CompassStreamSubscription> _updatesSubscriptions = [];
  StreamSubscription<SensorEvent> _accelerometerStream;
  StreamSubscription<SensorEvent> _magneticFieldStream;
  StreamSubscription<SensorEvent> _rotationSensorStream;
  StreamController<double> _internalUpdateController =
      StreamController.broadcast();

  /// Starts the compass updates.
  Stream<double> compassUpdates(Duration interval, double azimuthFix) {
    this.azimuthFix = azimuthFix ?? this.azimuthFix;
    StreamController<double> compassStreamController;
    _CompassStreamSubscription compassStreamSubscription;
    // ignore: cancel_subscriptions
    StreamSubscription<double> compassSubscription =
        _internalUpdateController.stream.listen((value) {
      if (interval != null) {
        DateTime instant = DateTime.now();
        int difference = instant
            .difference(compassStreamSubscription.lastUpdated)
            .inMicroseconds;
        if (difference < interval.inMicroseconds) {
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
        if (_sensorsStarted()) return;
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

  /// Checks if is possible to use the compass in the device.
  static Future<bool> get isCompassAvailable async {
    bool advanced = await isAdvancedSensorAvailable;
    bool def = await isDefaultSensorsAvailable;
    return advanced || def;
  }

  /// Checks if the rotation sensor is available in the system.
  static Future<bool> get isAdvancedSensorAvailable async {
    return SensorManager().isSensorAvailable(Sensors.ROTATION);
  }

  /// Checks if the defaults sensors used to compute the azimuth are available
  /// in the system.
  static Future<bool> get isDefaultSensorsAvailable async {
    bool accelerometer =
        await SensorManager().isSensorAvailable(Sensors.LINEAR_ACCELERATION);
    bool magneticField =
        await SensorManager().isSensorAvailable(Sensors.MAGNETIC_FIELD);
    return accelerometer && magneticField;
  }

  /// Determines which sensor is available and starts the updates if possible.
  void _startSensors() async {
    bool advanced = await isAdvancedSensorAvailable;
    bool def = await isDefaultSensorsAvailable;
    if (advanced) {
      _startAdvancedSensor();
    } else if (def) {
      _startDefaultSensors();
    }
  }

  /// Starts the rotation sensor for each platform.
  void _startAdvancedSensor() async {
    final stream = await SensorManager().sensorUpdates(
      sensorId: Sensors.ROTATION,
      interval: Sensors.SENSOR_DELAY_UI,
    );
    _rotationSensorStream = stream.listen((event) {
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

  /// Starts the default sensors used to compute the azimuth.
  void _startDefaultSensors() async {
    final accelerometerStream = await SensorManager().sensorUpdates(
      sensorId: Sensors.LINEAR_ACCELERATION,
      interval: Sensors.SENSOR_DELAY_UI,
    );
    final magneticFieldStream = await SensorManager().sensorUpdates(
      sensorId: Sensors.MAGNETIC_FIELD,
      interval: Sensors.SENSOR_DELAY_UI,
    );
    _accelerometerStream = accelerometerStream.listen(_defaultSensorsCallback);
    _magneticFieldStream = magneticFieldStream.listen(_defaultSensorsCallback);
  }

  /// Callback for the default sensor (accelerometer & magnetic field)
  /// used to compute the azimuth.
  void _defaultSensorsCallback(SensorEvent event) {
    switch (event.sensorId) {
      case Sensors.LINEAR_ACCELERATION:
        _gravity[0] = Platform.isAndroid
            ? event.data[0]
            : _gravitationalForceToMS2(event.data[0]);
        _gravity[1] = Platform.isAndroid
            ? event.data[1]
            : _gravitationalForceToMS2(event.data[1]);
        _gravity[2] = Platform.isAndroid
            ? event.data[2]
            : _gravitationalForceToMS2(event.data[2]);
        break;
      case Sensors.MAGNETIC_FIELD:
        _geomagnetic[0] = event.data[0];
        _geomagnetic[1] = event.data[1];
        _geomagnetic[2] = event.data[2];
        break;
    }
    if (_computeRotationMatrix()) {
      List<double> orientation = _computeOrientation();
      _azimuth = degrees(orientation[0]);
      _azimuth = (_azimuth + azimuthFix + 360) % 360;
      _internalUpdateController.add(_azimuth);
    }
  }

  /// Checks if the sensors has been started.
  bool _sensorsStarted() {
    return _rotationSensorStream != null ||
        (_accelerometerStream != null && _magneticFieldStream != null);
  }

  /// Stops the sensors updates subscribed.
  void _stopSensors() {
    if (_accelerometerStream != null) {
      _accelerometerStream.cancel();
      _accelerometerStream = null;
    }
    if (_magneticFieldStream != null) {
      _magneticFieldStream.cancel();
      _magneticFieldStream = null;
    }
    if (_rotationSensorStream != null) {
      _rotationSensorStream.cancel();
      _rotationSensorStream = null;
    }
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

  /// Transform acceleration in Gravitational Force (G) to
  /// metre/(square seconds).
  double _gravitationalForceToMS2(double gForce) {
    return gForce / 0.101971621297793;
  }
}

/// Class that represents a subscription to the stream of compass updates.
class _CompassStreamSubscription {
  /// Subscription to the stream of the compass.
  StreamSubscription subscription;

  /// Date of the last update.
  DateTime lastUpdated;

  _CompassStreamSubscription(this.subscription) {
    this.lastUpdated = DateTime.now();
  }
}
