#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#include <math.h>
#include <stdio.h>

#define TIME_STEP_MS 32
#define DT (TIME_STEP_MS / 1000.0)
#define NUM_FINGERS  3
#define NUM_ARM_JOINTS 4

// sensor units
#define GRAB_THRESH  350.0

// rad
#define FINGER_OPEN  0.0
#define FINGER_CLOSED 0.85

// tolerance for "joint reached target" (rad)
#define POS_EPS      0.02

// rad per second
#define MAX_JOINT_VEL 1.0

static const double HOME_POS[NUM_ARM_JOINTS]  = { 0.0, 0.0, 0.0, 0.0 };
static const double DROP_POS[NUM_ARM_JOINTS]  = { -1.88, -2.14, -2.38, -1.51 };

typedef enum {
  WAITING,
  GRASPING,
  MOVING_TO_DROP,
  RELEASING,
  MOVING_HOME
} State;

// if abs(distance between element and target) < tolerance
static bool targets_reached(const WbDeviceTag *sensors,
                            const double *target,
                            int n,
                            double tol) {
  for (int i = 0; i < n; ++i) {
    double pos = wb_position_sensor_get_value(sensors[i]);
    if (fabs(pos - target[i]) > tol)
      return false;
  }
  return true;
}

// move joints with capped velocity
static void move_with_ramp(const WbDeviceTag *motors,
                           const WbDeviceTag *sensors,
                           const double *target,
                           int n) {
  for (int i = 0; i < n; ++i) {
    double curr = wb_position_sensor_get_value(sensors[i]);
    double err  = target[i] - curr;
    double step = MAX_JOINT_VEL * DT;
    if (fabs(err) < step) step = fabs(err);
    double new_goal = curr + copysign(step, err);
    wb_motor_set_position(motors[i], new_goal);
  }
}

int main(int argc, char **argv) {
  wb_robot_init();
  
  const char *finger_names[NUM_FINGERS] = {
    "finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"
  };
  const char *arm_names[NUM_ARM_JOINTS] = {
    "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint"
  };
  
  WbDeviceTag finger_motors[NUM_FINGERS], finger_sensors[NUM_FINGERS];
  WbDeviceTag arm_motors[NUM_ARM_JOINTS], arm_sensors[NUM_ARM_JOINTS];

  for (int i = 0; i < NUM_FINGERS; ++i) {
    finger_motors[i]  = wb_robot_get_device(finger_names[i]);
    char sensor_name[64];
    // building new string named sensor_name to not write them all over again
    sprintf(sensor_name, "%s_sensor", finger_names[i]);
    finger_sensors[i] = wb_robot_get_device(sensor_name);
    printf("finger sensor: %hu\n", finger_sensors[i]); // debug
    wb_position_sensor_enable(finger_sensors[i], TIME_STEP_MS);
  }

  for (int i = 0; i < NUM_ARM_JOINTS; ++i) {
    arm_motors[i] = wb_robot_get_device(arm_names[i]);
    wb_motor_set_velocity(arm_motors[i], MAX_JOINT_VEL); /* soft cap */
    char sensor_name[64];
    sprintf(sensor_name, "%s_sensor", arm_names[i]);
    arm_sensors[i] = wb_robot_get_device(sensor_name);
    printf("arm sensor: %hu\n", arm_sensors[i]); // debug
    wb_position_sensor_enable(arm_sensors[i], TIME_STEP_MS);
  }
  
  WbDeviceTag dist = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(dist, TIME_STEP_MS);
  
  
}

