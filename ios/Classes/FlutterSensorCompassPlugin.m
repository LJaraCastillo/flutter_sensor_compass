#import "FlutterSensorCompassPlugin.h"
#import <flutter_sensor_compass/flutter_sensor_compass-Swift.h>

@implementation FlutterSensorCompassPlugin
+ (void)registerWithRegistrar:(NSObject<FlutterPluginRegistrar>*)registrar {
  [SwiftFlutterSensorCompassPlugin registerWithRegistrar:registrar];
}
@end
