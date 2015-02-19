/*
 * The program used in the YouTube video http://www.youtube.com/watch?v=Gg6zqjDq1ho
 *
 * This program allow to rotate a webcam to track an object. This is
 * done by using Arduino to control a servo motor. The communication
 * is done using a serial connection (through USB or native).
 *
 * To compile it under Linux (assuming OpenCV was previously installed):
 *
 *   g++ -O2 -W -Wall -lhighgui tracking.cc -o tracking
 *
 * See the documentation to get the Arduino program.
 *
 * Author: Frédéric Jolliton <frederic@jolliton.com>
 * Date: january 22, 2011
 * Documentation: http://doc.tuxee.net/tracking
 */

#include <iostream>

#include <sys/time.h>
#include <stdio.h>

#include <opencv/highgui.h>
#include <opencv/cv.h>

//-------- PID parameters --------

// See http://en.wikipedia.org/wiki/PID_controller

// These values must be chosen CAREFULLY. The strategy to find good
// values is to set `ci' and `cd' to 0.0, then try to find a value of
// `cp' that works the best (without too much oscillation) then, from
// that, lower `cp' and increase `cd' until the system is able to
// stalibilize more quickly. Increase `ci' if the system take time to
// move to the target position. There are more complexes method to
// find the "right" values.

static double cp = 0.2;
static double ci = 0.0;
static double cd = 0.02;

/*
 * Get the current time in seconds
 */
static double
gettime()
{
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_sec + tv.tv_usec / 1e6;
}

int main()
{
  CvCapture*          capture = cvCreateCameraCapture(0);
  if (capture == 0) {
    std::cerr << "Failed to open the camera.\n";
    return 1;
  }

  /*
   * Window to display the input image.
   */
  cvNamedWindow("RGB", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("RGB", 0, 0);

  /*
   * Window to display the mask (the selected part of the input image)
   */
  cvNamedWindow("Mask", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("Mask", 0, 505);

  /*
   * The settings window
   */

  cvNamedWindow("Settings", 0);
  cvMoveWindow("Settings", 652, 505);

  // The hue range to select.
  int                 hue_level_start = 0;
  int                 hue_level_stop = 12;
  cvCreateTrackbar("Hue level (start)", "Settings", &hue_level_start, 255, 0);
  cvCreateTrackbar("Hue level (stop)", "Settings", &hue_level_stop, 255, 0);

  // The minimum saturation level.
  int                 sat_level = 100;
  cvCreateTrackbar("Saturation level", "Settings", &sat_level, 255, 0);

  // Target position.
  int                 target = 320;
  cvCreateTrackbar("Target", "Settings", &target, 640, 0);

  /*
   * Window to display the tracking state
   */
  cvNamedWindow("Track", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("Track", 646, 0);

  FILE*               serial = fopen("/dev/ttyACM0", "w");
  if (serial == 0) {
    printf("Failed to open serial port\n");
  }
  sleep(1);

  IplImage*           chan1 = 0;
  IplImage*           chan2 = 0;
  IplImage*           chan3 = 0;
  IplImage*           hsv = 0;
  IplImage*           monitor = 0;
  IplImage*           result = 0;

  double              angle = 1500.0; // the position of the camera
  int                 tcolor = 0; // target color - Used to switch to predefined hue levels.
  double              last_time = 0.0; // last time we updated PID
  int                 last_known_x = 320; // last known position of the target.
  double              last_error = 0.0;
  double              i = 0.0; // integral term (here because it is accumulating)
  int                 last_sent_value = -1;

  for (int n = 0;; ++n) {

    //-------- Get the input image --------

    IplImage*           frame = cvQueryFrame(capture);
    if (frame == 0) break;
    cvShowImage("RGB", frame);

    //-------- Allocate images --------

    if (hsv == 0) {
      // Allocate images if it is the first iteration.
      hsv     = cvCreateImage(cvGetSize(frame), 8, 3); // 8 bits, 3 channels
      chan1   = cvCreateImage(cvGetSize(frame), 8, 1); // 8 bits, 1 channels
      chan2   = cvCreateImage(cvGetSize(frame), 8, 1);
      chan3   = cvCreateImage(cvGetSize(frame), 8, 1);
      monitor = cvCreateImage(cvGetSize(frame), 8, 3);
      result  = cvCreateImage(cvGetSize(frame), 8, 3);
    }

    //-------- Process the input image --------

    cvCvtColor(frame, hsv, CV_BGR2HSV); // convert to HSV (Hue-Saturation-Value)
    cvSplit(hsv, chan1, chan2, 0, 0); // extract Hue & Saturation

    // Create a mask matching only the selected range of hue values.
    if (hue_level_start <= hue_level_stop) {
      cvInRangeS(chan1, cvScalar(hue_level_start), cvScalar(hue_level_stop), chan1);
    } else {
      cvInRangeS(chan1, cvScalar(hue_level_stop), cvScalar(hue_level_start), chan1);
      cvSubRS(chan1, cvScalar(255), chan1);
    }

    // Create a mask matching only the selected saturation levels.
    cvCmpS(chan2, sat_level, chan2, CV_CMP_GT); // Test Saturation

    cvAnd(chan1, chan2, chan3); // Merge masks
    cvErode(chan3, chan3, 0, 2); // Suppress noise

    // Find the position (moment) of the selected regions.
    CvMoments           moments;
    cvMoments(chan3, &moments, 1);
    int                 r = sqrt(moments.m00);
    int                 x = moments.m10/moments.m00;
    int                 y = moments.m01/moments.m00;

    //-------- Mask window --------

    // blue = hue selection, green = saturation selection, red = selected regions
    cvConvertScale(frame, monitor, .3, 0); // faded out input
    cvSet(monitor, cvScalar(255, 0, 0), chan1); // blue overlay
    cvSet(monitor, cvScalar(0, 255, 0), chan2); // green overlay
    cvSet(monitor, cvScalar(0, 0, 255), chan3); // red overlay
    if (x > 0 && y > 0) {
      cvCircle(monitor, cvPoint(x, y), r, cvScalar(0, 0, 0), 4, CV_AA, 0);
      cvCircle(monitor, cvPoint(x, y), r, cvScalar(255, 255, 255), 2, CV_AA, 0);
    }
    cvShowImage("Mask", monitor);

    //-------- Tracking state --------

    cvCopy(frame, result, 0); // input image
    cvLine(result, cvPoint(target - 60, 0), cvPoint(target - 60, 480), cvScalar(0, 0, 0), 2);
    cvLine(result, cvPoint(target, 0), cvPoint(target, 480), cvScalar(0, 0, 255), 3);
    cvLine(result, cvPoint(target + 60, 0), cvPoint(target + 60, 480), cvScalar(0, 0, 0), 2);
    if (x > 0 && y > 0) {
      cvCircle(result, cvPoint(x, y), 10, cvScalar(0, 0, 0), 6, CV_AA, 0);
      cvCircle(result, cvPoint(x, y), 10, cvScalar(0, 255, 255), 4, CV_AA, 0);
    }
    cvShowImage("Track", result);

    //-------- Handle keyboard events --------

    int key = cvWaitKey(33);
    if (key == 27) break;
    switch (key) {
    case 'r':
      // Reset the current position. This is used to check how fast
      // the system can return to the correct position.
      angle = 1500;
      break;
    case 't':
      // Quickly switch to a different tracking color (red or blue)
      tcolor = 1 - tcolor;
      if (tcolor == 0) {
        cvSetTrackbarPos("Hue level (start)", "Settings", 0);
        cvSetTrackbarPos("Hue level (stop)", "Settings", 12);
      } else {
        cvSetTrackbarPos("Hue level (start)", "Settings", 109);
        cvSetTrackbarPos("Hue level (stop)", "Settings", 116);
      }
      break;
    default:
      // ignore other keys.
      break;
    }

    //-------- PID processing --------

    // If the object is out of the window, use last known position to
    // find it.
    if (x < 0 || x > 640) {
      x = 2.5 * (last_known_x - 320) + 320;
    } else {
      last_known_x = x;
    }

    double              time = gettime();
    double              dt = time - last_time;
    if (last_time == 0.0)
      dt = 1.0;
    last_time = time;

    // the error we want to correct
    double              error = x - target;

    // the proportional term
    double              p = error * cp;

    // update the integral term
    i += error * dt * ci;

    // Clamp integral term
    if (i > 30.0)
      i = 30.0;
    else if (i < -30.0)
      i = -30.0;

    // the derivative term
    double              d = (error - last_error) / dt * cd;
    last_error = error;

    // the PID value
    double              pid = p + i + d;

    // Clamp PID
    if (pid < -100)
      pid = -100;
    else if (pid > 100)
      pid = 100;

    // Update the position from the PID value.
    angle += pid;

    // Clamp angle
    if (angle < 0)
      angle = 0;
    else if (angle > 2000)
      angle = 2000;

    printf("pos = %d, P = %f, I = %f, D = %f, angle = %f, dt = %f\n", x, p, i, d, angle, dt);

    //-------- Send the position to Arduino --------

    if (serial != 0) {
      int                 current_value = angle;
      // Send the new position if it changed since the last time.
      if (current_value != last_sent_value) {
        fprintf(serial, "%d\n", current_value);
        printf("SENT %d\n", current_value);
        last_sent_value = current_value;
      }
    }
  }
  cvReleaseCapture(&capture);
}
