import 'package:flutter/material.dart';
import 'dart:async';

import 'package:flutter_sensor_compass/flutter_sensor_compass.dart';

void main() => runApp(MyApp());

class MyApp extends StatefulWidget {
  @override
  _MyAppState createState() => _MyAppState();
}

class _MyAppState extends State<MyApp> {
  bool _compassEnabled = false;
  double _degrees = 0.0;
  StreamSubscription _compassSubscription;

  @override
  void initState() {
    _checkCompassAvailability();
    super.initState();
  }

  @override
  void dispose() {
    _stopCompass();
    super.dispose();
  }

  void _checkCompassAvailability() async {
    Compass.isCompassAvailable().then((value) {
      setState(() {
        _compassEnabled = value;
      });
    });
  }

  void _startCompass() {
    if (_compassSubscription != null) return;
    _compassSubscription =
        Compass.compassUpdates(Duration(milliseconds: 1)).listen((value) {
      setState(() {
        _degrees = value;
      });
    });
  }

  void _stopCompass() {
    if (_compassSubscription == null) return;
    _compassSubscription.cancel();
    _compassSubscription = null;
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(
          title: const Text('Compass example'),
        ),
        body: Container(
          padding: EdgeInsets.all(16.0),
          alignment: AlignmentDirectional.topCenter,
          child: Column(
            children: <Widget>[
              Text("Compass Enabled: $_compassEnabled"),
              Padding(padding: EdgeInsets.only(top: 16.0)),
              Text("Degrees: $_degrees"),
              Padding(padding: EdgeInsets.only(top: 16.0)),
              MaterialButton(
                child: Text("Start"),
                color: Colors.green,
                onPressed: _compassEnabled ? () => _startCompass() : null,
              ),
              MaterialButton(
                child: Text("Stop"),
                color: Colors.red,
                onPressed: _compassEnabled ? () => _stopCompass() : null,
              ),
            ],
          ),
        ),
      ),
    );
  }
}
