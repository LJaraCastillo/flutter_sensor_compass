part of flutter_sensor_compass;

class _Compass {
  static const double _alpha = 0.97;
  List<double> _gravity = List.filled(3, 0.0);
  List<double> _geomagnetic = List.filled(3, 0.0);
  List<double> _rotationMatrix = List();
  double _azimuth = 0.0;
  double azimuthFix = 0.0;
  final List<_CompassStreamSubscription> _updatesSubscriptions = [];
  StreamSubscription<SensorEvent> _sensorStream;
  StreamController<double> _internalUpdateController =
      StreamController.broadcast();

  Stream<double> compassUpdates(Duration delay) {
    StreamController<double> compassStreamController;
    _CompassStreamSubscription compassStreamSubscription;
    StreamSubscription<double> compassSubscription =
        _internalUpdateController.stream.listen((value) {
      DateTime instant = DateTime.now();
      int difference = instant
          .difference(compassStreamSubscription.lastUpdated)
          .inMicroseconds;
      if (difference >= delay.inMicroseconds) {
        compassStreamSubscription.lastUpdated = instant;
        compassStreamController.add(value);
      }
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

  static Future<bool> get checkAviability async {
    bool accelerometer =
        await SensorManager.isSensorAvailable(Sensors.ACCELEROMETER);
    bool magneticField =
        await SensorManager.isSensorAvailable(Sensors.MAGNETIC_FIELD);
    return accelerometer && magneticField;
  }

  void _startSensors() {
    _sensorStream = SensorManager.sensorsUpdates([
      SensorRequest(Sensors.ACCELEROMETER,
          refreshDelay: Sensors.SENSOR_DELAY_GAME),
      SensorRequest(Sensors.MAGNETIC_FIELD,
          refreshDelay: Sensors.SENSOR_DELAY_GAME),
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

  /// Updates the current rotation matrix using the values gattered by the
  /// accelerometer and magenetic field sensor.
  ///
  /// Returns true if the computation was successful and false otherwise.
  bool _computeRotationMatrix() {
    double ax = _gravity[0], ay = _gravity[1], az = _gravity[2];
    double normsqa = (ax * ax + ay * ay + az * az);
    double g = 9.81;
    double freeFallGravitySquared = 0.01 * g * g;
    if (normsqa < freeFallGravitySquared) return false;
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
    // Fill r matrix
    _rotationMatrix = [hx, hy, hz, mx, my, mz, ax, ay, az];
    return true;
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
