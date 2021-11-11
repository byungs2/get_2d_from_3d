#include <stdlib.h>
#include <stdio.h>
#include <k4arecord/playback.h>
#include <k4a/k4a.h>
#include <k4abt.h>
#include "matrix.h"

float intrinsic[3][3] = {{613.168701, 0, 639.633728},{0, 612.880493, 364.357513},{0, 0, 1}};
Mat *intrinsic_mat;
float point_arr[4][1];
Mat *point_mat;
Mat *image_mat;

float rotate_arr[4][4] = {{0.999945, 0.010509, 0.000076, 0},{-0.010469,  0.995458, 0.094629, 0},{0.000918, -0.094624, 0.995513, 0}, {0, 0, 0, 1}};
Mat *rotate_mat;
float translation_arr[4][4] = {{1,0,0,-32.102242},{0, 1, 0, -2.036716},{0, 0, 1, 4.053820},{0, 0, 0, 1}};
Mat *tr_mat;
float projection_arr[3][4] = {{1, 0, 0, 0},{0, 1, 0, 0}, {0, 0, 1, 0}};
Mat *pr_mat;

float rt_arr[4][4] = {{0.999945, 0.010509, 0.000076, -32.102242},{-0.010469,  0.995458, 0.094629, -2.036716},{0.000918, -0.094624, 0.995513, 4.053820}, {0, 0, 0, 1}};
Mat *rt_mat;

Mat *result_mat;

void get_2d_skeleton_joint_points_from_mkv_file (char *argv[]) {
  k4a_calibration_t sensor_calibration;
  k4abt_tracker_t tracker = NULL;
  k4a_playback_t playback_handle = NULL;
  const char *file_path = argv[1];
  if(k4a_playback_open(file_path, &playback_handle) != K4A_RESULT_SUCCEEDED) {
    printf("Failed to open file\n");
    return ;
  }
  if(k4a_playback_get_calibration(playback_handle, &sensor_calibration) != K4A_RESULT_SUCCEEDED) {
    printf("Failed to get calibration params\n");
    return ;
  }
  k4a_capture_t capture = NULL;
  k4a_stream_result_t result = K4A_STREAM_RESULT_SUCCEEDED;
  k4abt_tracker_configuration_t tracker_config = { K4ABT_SENSOR_ORIENTATION_DEFAULT };
  tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
  k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker);
  k4abt_tracker_set_temporal_smoothing(tracker, 1);

  int depth_width = sensor_calibration.depth_camera_calibration.resolution_width;
  int depth_height = sensor_calibration.depth_camera_calibration.resolution_height;

  while (result == K4A_STREAM_RESULT_SUCCEEDED) {
    result = k4a_playback_get_next_capture(playback_handle, &capture);
    if(result != K4A_STREAM_RESULT_EOF) {
      k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
      if(depth_image == NULL) {
        printf("No depth image\n");
        k4a_capture_release(capture);
        continue;
      }
      k4a_image_release(depth_image);
    }
    if(result == K4A_STREAM_RESULT_SUCCEEDED) {
      k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, capture, K4A_WAIT_INFINITE);
      k4a_capture_release(capture);
      k4abt_frame_t body_frame = NULL;
      k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
      if(pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED) {
        k4abt_skeleton_t skeleton;
        size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
        printf("%zu bodies detected from device source\n", num_bodies);
        for(int i = 0; i < num_bodies; i++) {
          k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
          k4a_float2_t image_xy;
          int valid;
          for(int j = 0; j < 32; j++) {
            // by sdk
            k4abt_joint_t joint = skeleton.joints[j];
            sensor_calibration.color_camera_calibration.intrinsics.parameters.param.k1 = 100;
            k4a_calibration_3d_to_2d (&sensor_calibration, &joint.position, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_COLOR, &image_xy, &valid);
            printf("%dth 2d joint point by sdk X :: %f, Y :: %f confidence :: %d\n", j, image_xy.xy.x, image_xy.xy.y, joint.confidence_level);

            // by self
            /*
            k4abt_joint_t joint = skeleton.joints[j];
            point_arr[0][0] = joint.position.xyz.x;
            point_arr[1][0] = joint.position.xyz.y;
            point_arr[2][0] = joint.position.xyz.z;
            point_arr[3][0] = 1.0;
            mat_init(point_mat, (float *)point_arr);

            mat_mul(result_mat, point_mat, image_mat);
            mat_devide(image_mat, image_mat->mat_array[2][0]);

            printf("%dth joint 2d point X:: %f, Y:: %f S:: %f\n", j, image_mat->mat_array[0][0], image_mat->mat_array[1][0], image_mat->mat_array[2][0]);
            */

            //printf("%dth joint 3d point X:: %f, Y:: %f, Z:: %f, Confidence :: %d\n", j, joint.position.xyz.x, joint.position.xyz.y, joint.position.xyz.z, joint.confidence_level);
          }
        }
        k4abt_frame_release(body_frame);
      } else {
        printf("Failure\n");
        break;
      }
    }
    if(result == K4A_STREAM_RESULT_EOF) {
      break;
    }
  }
  k4abt_tracker_shutdown(tracker);
  k4abt_tracker_destroy(tracker);
  printf("Finish\n");
  k4a_playback_close(playback_handle);

}

//TODO
void set_calibration (k4a_calibration_t *callib) {

}

void get_2d_skeleton_joint_points_from_device (char *argv[]) {
  k4a_device_t device = NULL;
  k4a_device_open(0, &device);

  k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
  device_config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
  k4a_device_start_cameras(device, &device_config);

  k4a_calibration_t sensor_calibration;
  k4a_device_get_calibration(device, device_config.depth_mode, device_config.color_resolution, &sensor_calibration);
  int depth_width = sensor_calibration.depth_camera_calibration.resolution_width;
  int depth_height = sensor_calibration.depth_camera_calibration.resolution_height;

  k4abt_tracker_t tracker = NULL;
  k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
  tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
  k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker);
  while(1) {
    k4a_capture_t sensor_capture = NULL;
    k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, 0);
    if(get_capture_result == K4A_WAIT_RESULT_SUCCEEDED) {
      k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, 0);
      k4a_capture_release(sensor_capture);
      if(queue_capture_result == K4A_WAIT_RESULT_FAILED) {
        printf("Error \n");
        break;
      }
    } else if(get_capture_result != K4A_WAIT_RESULT_TIMEOUT) {
      printf("Error !!! \n");
    }
    k4abt_frame_t body_frame = NULL;
    k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, 0);
    if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED) {
      k4abt_skeleton_t skeleton;
      size_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
      printf("%zu bodies detected from device source\n", num_bodies);
      for(int i = 0; i < num_bodies; i++) {
        k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
        for(int j = 0; j < 32; j++) {
          k4abt_joint_t joint = skeleton.joints[j];
          printf("%dth joint point X:: %f, Y:: %f, Z:: %f, Confidence :: %d\n", j, joint.position.xyz.x, joint.position.xyz.y, joint.position.xyz.z, joint.confidence_level);
        }
      }
      k4abt_frame_release(body_frame);
    }
  }
  k4abt_tracker_shutdown(tracker);
  k4abt_tracker_destroy(tracker);
  k4a_device_stop_cameras(device);
  k4a_device_close(device);
}

int main (int argc, char *argv[]) {
  intrinsic_mat = mat_new(3, 3);
  mat_init(intrinsic_mat, (float *)intrinsic);
  point_mat = mat_new(4, 1);
  image_mat = mat_new(3, 1);
  rotate_mat = mat_new(4, 4);
  mat_init(rotate_mat, (float *)rotate_arr);
  tr_mat = mat_new(4, 4);
  mat_init(tr_mat, (float *)translation_arr);
  pr_mat = mat_new(3, 4);
  mat_init(pr_mat, (float *)projection_arr);
  rt_mat = mat_new(4, 4);
  mat_init(rt_mat, (float *)rt_arr);

  result_mat = mat_new(3,4);

  mat_mul(rotate_mat, tr_mat, rotate_mat);
  mat_mul(pr_mat, rotate_mat, pr_mat);
  mat_mul(intrinsic_mat, pr_mat, result_mat);
  for(int i = 0; i < result_mat->row; i++) {
    printf("\n");
    for(int j = 0; j < result_mat->column; j++) {
      printf(" %f ", result_mat->mat_array[i][j]);
    }
  }

  if(argc > 1) {
    printf("File path :: %s \n", argv[1]);
    get_2d_skeleton_joint_points_from_mkv_file(argv);
  } else {
    get_2d_skeleton_joint_points_from_device(argv);
  }
}
