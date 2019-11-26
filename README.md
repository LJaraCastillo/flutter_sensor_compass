# flutter_sensor_compass

A plugin to get the orientation of the device (in degrees) relative to the magnetic north.

## Installation

First add the plugin in your project. 
Copy the following line below dependencies in your **pubspec.yaml** file.

```yaml
dependencies:
     ...
    flutter_sensor_compass: ^0.1.0
```

Then you need to import the dependency.

```dart
import 'package:flutter_sensor_compass/flutter_sensor_compass.dart';
```

### iOS only

You need to add the following key-value pair into your **Info.plist** file inside the **ios/Runner** folder in your project.

```plist
<key>NSMotionUsageDescription</key>
<string>A reason to get the permission</string>
```

## How to use

You can check if the device is capable of using the compass.

```dart
bool isAvailable = await Compass().isCompassAvailable();
```

Then you can register a new listener for the compass updates.

```dart
_compassSubscription =
        Compass().compassUpdates(interval: Duration(milliseconds: 200)).listen((value) {
      setState(() {
        _degrees = value;
      });
    });
```

Remember to cancel your **StreamSubscriptions** after you are done with 
the compass updates.

```dart
_compassSubscription.cancel();
```