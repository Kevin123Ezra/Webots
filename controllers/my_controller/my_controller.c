#include <stdio.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

int main(int argc, char **argv) {
  wb_robot_init();

  int i;
  WbDeviceTag ps[8];
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };

  for (i = 0; i < 8; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  WbDeviceTag ls[8];
  char ls_names[8][4] = {
    "ls0", "ls1", "ls2", "ls3",
    "ls4", "ls5", "ls6", "ls7"
  };

  for (i = 0; i < 8; i++) {
    ls[i] = wb_robot_get_device(ls_names[i]);
    wb_light_sensor_enable(ls[i], TIME_STEP);
  }

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  bool found_wall = false;
  double brightest = 99999.0;

  while (wb_robot_step(TIME_STEP) != -1) {
    double ps_values[8];
    for (i = 0; i < 8; i++)
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);

    for (i = 0; i < 8; i++)
      printf("ps%d: %.1f  ", i, ps_values[i]);
    printf("\n");

    double light = 0.0;
    for (i = 0; i < 8; i++)
      light += wb_light_sensor_get_value(ls[i]);
    light = light / 8.0;

    printf("light: %.1f\n", light);

    if (light < 2000.0)
      printf("Light detected!\n");

    if (light < brightest) {
      brightest = light;
      printf("New brightest: %.1f at time %.1f s\n", brightest, wb_robot_get_time());
    }

    bool front_wall = ps_values[0] > 80.0 || ps_values[7] > 80.0;
    bool right_corner = ps_values[1] > 80.0;
    bool right_wall = ps_values[2] > 80.0;

    if (front_wall)
      found_wall = true;

    double left_speed = 0.5 * MAX_SPEED;
    double right_speed = 0.5 * MAX_SPEED;

    if (found_wall) {
      if (front_wall) {
        left_speed = -0.5 * MAX_SPEED;
        right_speed = 0.5 * MAX_SPEED;
      }
      else if (right_corner) {
        left_speed = 0.25 * MAX_SPEED;
        right_speed = 0.5 * MAX_SPEED;
      }
      else if (right_wall) {
        left_speed = 0.5 * MAX_SPEED;
        right_speed = 0.5 * MAX_SPEED;
      }
      else {
        left_speed = 0.5 * MAX_SPEED;
        right_speed = 0.25 * MAX_SPEED;
      }
    }

    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}
