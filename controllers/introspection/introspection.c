#include <webots/device.h>
#include <webots/robot.h>

#include <stdio.h>

int main() {
  wb_robot_init();

  int n = wb_robot_get_number_of_devices();          // how many devices total
  for (int i = 0; i < n; ++i) {
    WbDeviceTag tag   = wb_robot_get_device_by_index(i);  // index-based access
    const char *name  = wb_device_get_name(tag);          // “finger_1_joint_1_sensor”, …
    int type          = wb_device_get_node_type(tag);     // enum WbNodeType (MOTOR, POSITION_SENSOR …)
    printf("#%02d  %-30s  type=%d\n", i, name, type);
  }
  wb_robot_cleanup();
}
