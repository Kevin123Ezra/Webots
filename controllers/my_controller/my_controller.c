#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#define PI 3.14159265358979
#define TIME_STEP 64
#define MAX_SPEED 6.28
#define BASE_SPEED (0.5 * MAX_SPEED)
#define TURN_SPEED (0.3 * MAX_SPEED)
#define WHEEL_RADIUS 0.0205
#define AXLE_LENGTH 0.058

#define FRONT_THRESHOLD 90.0
#define LEFT_SEEN 80.0
#define LEFT_LOST 75.0
#define LEFT_TARGET 120.0
#define RIGHT_TOO_CLOSE 150.0
#define PASS_DISTANCE 0.085
#define PROBE_DISTANCE 0.20

#define LIGHT_THRESHOLD 2000.0
#define LIGHT_RESET 2200.0

#define DEAD_END_GAP 0.12
#define DEAD_END_SPACING 0.25
#define MAX_DEAD_ENDS 64

#define LAP_ARM_DISTANCE 0.10
#define LAP_MIN_TRAVEL 3.0
#define HOME_RANGE 0.10
#define HOME_TOLERANCE 0.01

enum { FIND_WALL, FOLLOW, TURN_RIGHT, PASS_GAP, TURN_LEFT, PROBE, GO_HOME, DONE };

typedef struct {
  double x, y;
} Point;

typedef struct {
  WbDeviceTag ps[8];
  WbDeviceTag ls[8];
  WbDeviceTag gps;
  WbDeviceTag left_enc, right_enc;
  WbDeviceTag left_motor, right_motor;
} Devices;

typedef struct {
  Point origin;
  Point pos;
  double ps[8];
  double light;

  double heading;
  bool heading_known;
  double prev_left, prev_right;
  double step_dist;
  Point check_pos;
  double check_turn;

  int state;
  double target_dir;
  double state_dist;
  int lost_count;

  bool lap_armed, lap_done;
  Point lap_start;
  double lap_dir;
  double lap_travel;
  double follow_run;

  bool light_seen, in_light;
  double light_w, light_wx, light_wy;

  bool last_turn_right;
  double since_right_turn;
  Point right_turn_pos;
  Point dead_ends[MAX_DEAD_ENDS];
  int dead_end_count;
} Robot;

double wrap_angle(double a) {
  while (a > PI)
    a -= 2.0 * PI;
  while (a < -PI)
    a += 2.0 * PI;
  return a;
}

double clamp(double v, double lo, double hi) {
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

double distance(Point a, Point b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

void init_devices(Devices *d) {
  char name[8];
  for (int i = 0; i < 8; i++) {
    sprintf(name, "ps%d", i);
    d->ps[i] = wb_robot_get_device(name);
    wb_distance_sensor_enable(d->ps[i], TIME_STEP);
    sprintf(name, "ls%d", i);
    d->ls[i] = wb_robot_get_device(name);
    wb_light_sensor_enable(d->ls[i], TIME_STEP);
  }

  d->gps = wb_robot_get_device("gps");
  wb_gps_enable(d->gps, TIME_STEP);

  d->left_enc = wb_robot_get_device("left wheel sensor");
  d->right_enc = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(d->left_enc, TIME_STEP);
  wb_position_sensor_enable(d->right_enc, TIME_STEP);

  d->left_motor = wb_robot_get_device("left wheel motor");
  d->right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(d->left_motor, INFINITY);
  wb_motor_set_position(d->right_motor, INFINITY);
  wb_motor_set_velocity(d->left_motor, 0.0);
  wb_motor_set_velocity(d->right_motor, 0.0);
}

void start_robot(Devices *d, Robot *r) {
  const double *gps = wb_gps_get_values(d->gps);
  r->origin.x = gps[0];
  r->origin.y = gps[1];
  r->prev_left = wb_position_sensor_get_value(d->left_enc);
  r->prev_right = wb_position_sensor_get_value(d->right_enc);
  r->state = FIND_WALL;
  r->since_right_turn = 1e9;
  printf("SPAWN    at (0.000, 0.000)\n");
}

void read_sensors(Devices *d, Robot *r) {
  for (int i = 0; i < 8; i++)
    r->ps[i] = wb_distance_sensor_get_value(d->ps[i]);

  r->light = 0.0;
  for (int i = 0; i < 8; i++)
    r->light += wb_light_sensor_get_value(d->ls[i]);
  r->light /= 8.0;

  const double *gps = wb_gps_get_values(d->gps);
  r->pos.x = gps[0] - r->origin.x;
  r->pos.y = gps[1] - r->origin.y;

  double left = wb_position_sensor_get_value(d->left_enc);
  double right = wb_position_sensor_get_value(d->right_enc);
  double d_left = (left - r->prev_left) * WHEEL_RADIUS;
  double d_right = (right - r->prev_right) * WHEEL_RADIUS;
  r->prev_left = left;
  r->prev_right = right;

  double d_theta = (d_right - d_left) / AXLE_LENGTH;
  r->step_dist = fabs(d_right + d_left) / 2.0;
  r->heading = wrap_angle(r->heading + d_theta);
  r->state_dist += r->step_dist;
  r->since_right_turn += r->step_dist;
  r->lap_travel += r->step_dist;
  if (r->state == FOLLOW)
    r->follow_run += r->step_dist;

  Point zero = {0.0, 0.0};
  if (!r->heading_known && distance(r->pos, zero) > 0.01) {
    r->heading = atan2(r->pos.y, r->pos.x);
    r->heading_known = true;
  }

  r->check_turn += fabs(d_theta);
  if (r->check_turn > 0.08 || d_right + d_left <= 0.0) {
    r->check_pos = r->pos;
    r->check_turn = 0.0;
  } else if (distance(r->pos, r->check_pos) > 0.03) {
    double gps_dir = atan2(r->pos.y - r->check_pos.y, r->pos.x - r->check_pos.x);
    r->heading = wrap_angle(r->heading + 0.5 * wrap_angle(gps_dir - r->heading));
    r->check_pos = r->pos;
    r->check_turn = 0.0;
  }
}

void track_light(Robot *r) {
  if (r->light < LIGHT_THRESHOLD && !r->lap_done) {
    double w = LIGHT_THRESHOLD - r->light;
    r->light_w += w;
    r->light_wx += w * r->pos.x;
    r->light_wy += w * r->pos.y;
  }

  if (!r->in_light && r->light < LIGHT_THRESHOLD) {
    r->in_light = true;
    if (!r->light_seen) {
      r->light_seen = true;
      printf("LIGHT    detected, robot at (%.3f, %.3f)\n", r->pos.x, r->pos.y);
    }
  } else if (r->in_light && r->light > LIGHT_RESET) {
    r->in_light = false;
  }
}

void track_lap(Robot *r) {
  Point zero = {0.0, 0.0};

  if (!r->lap_armed && !r->lap_done && r->state == FOLLOW && r->follow_run >= LAP_ARM_DISTANCE) {
    r->lap_armed = true;
    r->lap_start = r->pos;
    r->lap_dir = r->target_dir;
    r->lap_travel = 0.0;
  }

  if (r->lap_armed && r->state == FOLLOW && r->lap_travel > LAP_MIN_TRAVEL &&
      fabs(wrap_angle(r->target_dir - r->lap_dir)) < 0.1) {
    double dx = r->pos.x - r->lap_start.x;
    double dy = r->pos.y - r->lap_start.y;
    double along = dx * cos(r->lap_dir) + dy * sin(r->lap_dir);
    double side = -dx * sin(r->lap_dir) + dy * cos(r->lap_dir);
    if (along >= 0.0 && along < 0.05 && fabs(side) < 0.05) {
      if (r->lap_done) {
        r->state = GO_HOME;
      } else {
        printf("LAP COMPLETE at (%.3f, %.3f) - %d dead end(s) found\n", r->pos.x, r->pos.y, r->dead_end_count);
        if (r->light_w > 0.0)
          printf("LIGHT    source estimated at (%.3f, %.3f)\n", r->light_wx / r->light_w, r->light_wy / r->light_w);
        else
          printf("LIGHT    none found\n");
        r->lap_done = true;
        r->lap_travel = 0.0;
      }
    }
  }

  if (r->lap_done && r->state != GO_HOME && distance(r->pos, zero) < HOME_RANGE)
    r->state = GO_HOME;
}

void report_dead_end(Robot *r) {
  for (int i = 0; i < r->dead_end_count; i++)
    if (distance(r->dead_ends[i], r->right_turn_pos) < DEAD_END_SPACING)
      return;

  if (r->dead_end_count < MAX_DEAD_ENDS) {
    r->dead_ends[r->dead_end_count] = r->right_turn_pos;
    r->dead_end_count++;
  }
  printf("DEAD END at (%.3f, %.3f)\n", r->right_turn_pos.x, r->right_turn_pos.y);
}

void set_state(Robot *r, int state) {
  r->state = state;
  r->state_dist = 0.0;
  r->follow_run = 0.0;
  r->lost_count = 0;
}

void start_turn(Robot *r, int turn) {
  if (turn == TURN_RIGHT) {
    if (r->last_turn_right && r->since_right_turn < DEAD_END_GAP) {
      report_dead_end(r);
      r->last_turn_right = false;
    } else {
      r->last_turn_right = true;
      r->right_turn_pos = r->pos;
    }
    r->since_right_turn = 0.0;
    r->target_dir = wrap_angle(r->target_dir - PI / 2.0);
  } else {
    r->last_turn_right = false;
    r->target_dir = wrap_angle(r->target_dir + PI / 2.0);
  }
  set_state(r, turn);
}

void drive(Robot *r, double dir, double speed, double *left, double *right) {
  double turn = clamp(wrap_angle(dir - r->heading), -0.3, 0.3) * MAX_SPEED;
  *left = speed - turn;
  *right = speed + turn;
}

void spin(double error, double *left, double *right) {
  double speed = clamp(1.5 * fabs(error), 0.08, 1.0) * TURN_SPEED;
  if (error > 0) {
    *left = -speed;
    *right = speed;
  } else {
    *left = speed;
    *right = -speed;
  }
}

void decide(Robot *r, double *left, double *right) {
  bool front_wall = r->ps[0] > FRONT_THRESHOLD || r->ps[7] > FRONT_THRESHOLD;
  bool left_wall = r->ps[5] > LEFT_SEEN;
  Point zero = {0.0, 0.0};
  *left = 0.0;
  *right = 0.0;

  if (r->state == FIND_WALL) {
    if (front_wall) {
      r->target_dir = wrap_angle(round(r->heading / (PI / 2.0)) * (PI / 2.0));
      start_turn(r, TURN_LEFT);
    } else {
      *left = BASE_SPEED;
      *right = BASE_SPEED;
    }
  } else if (r->state == FOLLOW) {
    if (r->ps[5] < LEFT_LOST && r->ps[6] < LEFT_LOST)
      r->lost_count++;
    else
      r->lost_count = 0;

    if (front_wall) {
      start_turn(r, TURN_RIGHT);
    } else if (r->lost_count >= 2) {
      set_state(r, PASS_GAP);
    } else {
      double offset = 0.0;
      if (left_wall)
        offset = -0.004 * (r->ps[5] - LEFT_TARGET);
      if (r->ps[2] > RIGHT_TOO_CLOSE)
        offset += 0.004 * (r->ps[2] - RIGHT_TOO_CLOSE);
      drive(r, r->target_dir + clamp(offset, -0.3, 0.3), BASE_SPEED, left, right);
    }
  } else if (r->state == PASS_GAP) {
    if (left_wall)
      set_state(r, FOLLOW);
    else if (front_wall || r->state_dist >= PASS_DISTANCE)
      start_turn(r, TURN_LEFT);
    else
      drive(r, r->target_dir, BASE_SPEED, left, right);
  } else if (r->state == PROBE) {
    if (front_wall)
      start_turn(r, TURN_RIGHT);
    else if (left_wall)
      set_state(r, FOLLOW);
    else if (r->state_dist >= PROBE_DISTANCE)
      start_turn(r, TURN_LEFT);
    else
      drive(r, r->target_dir, BASE_SPEED, left, right);
  } else if (r->state == TURN_LEFT || r->state == TURN_RIGHT) {
    double error = wrap_angle(r->target_dir - r->heading);
    if (fabs(error) > 0.02)
      spin(error, left, right);
    else if (r->state == TURN_LEFT)
      set_state(r, PROBE);
    else if (front_wall)
      start_turn(r, TURN_RIGHT);
    else
      set_state(r, FOLLOW);
  } else if (r->state == GO_HOME) {
    double dist = distance(r->pos, zero);
    double error = wrap_angle(atan2(-r->pos.y, -r->pos.x) - r->heading);
    if (dist < HOME_TOLERANCE) {
      printf("BACK AT SPAWN (%.3f, %.3f) - stopping\n", r->pos.x, r->pos.y);
      r->state = DONE;
    } else if (fabs(error) > 0.1) {
      spin(error, left, right);
    } else if (dist < 0.05) {
      drive(r, r->heading + error, 0.2 * MAX_SPEED, left, right);
    } else {
      drive(r, r->heading + error, BASE_SPEED, left, right);
    }
  }
}

int main() {
  Devices dev;
  Robot robot = {0};
  double left, right;

  wb_robot_init();
  init_devices(&dev);
  wb_robot_step(TIME_STEP);
  start_robot(&dev, &robot);

  while (robot.state != DONE && wb_robot_step(TIME_STEP) != -1) {
    read_sensors(&dev, &robot);
    track_light(&robot);
    track_lap(&robot);
    decide(&robot, &left, &right);
    wb_motor_set_velocity(dev.left_motor, left);
    wb_motor_set_velocity(dev.right_motor, right);
  }

  wb_motor_set_velocity(dev.left_motor, 0.0);
  wb_motor_set_velocity(dev.right_motor, 0.0);
  wb_robot_step(TIME_STEP);
  wb_robot_cleanup();
  return 0;
}
