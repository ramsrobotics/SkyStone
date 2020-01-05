package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;

/**
 * Implements the common algorithms used both by BoKAutoBlue* and BoKAutoRed*.
 * Its primary responsibilities include:
 * initSoftware() method which
 * 1. initialize OpenCV
 * 2. initialize Vuforia
 * moveForward() method
 * <p>
 * <p>
 * Flow of Auto: Goal: Reach soft cap auto points
 * detect block 1 sec
 * drive forward 2 sec
 * align and grab 4 sec
 * back up 2 sec
 * turn towards the foundation 1 sec
 * drive against the wall 4 sec
 * drive by dist
 * align to foundation 4 sec
 * place 2 sec
 * pull with intake 3 sec
 * turn 1 sec
 * park 1 sec
 */
public abstract class CCAutoCommon implements CCAuto {
    protected static final boolean DEBUG_OPEN_CV = false;
    // CONSTANTS
    private static final double DT_RAMP_SPEED_INIT = 0.2;
    private static final double P_TURN_COEFF = 0.5;
    // Sampling locations in the image; phone's image is 1280x720s
    private static final int HOUGH_CIRCLE_MIN_RAD = 20;
    private static final int HOUGH_CIRCLE_MAX_RAD = 105;
    private static final int SPHERE_LOC_Y_MIN = 275;
    private static final int SPHERE_LOC_Y_MAX = 600;
    private static final int CUBE_LOC_LEFT_X_MIN = 350;
    private static final int CUBE_LOC_RIGHT_X_MAX = 850;
    private static final int ROI_WIDTH = 50;
    private static final int ROI_HEIGHT = 50;
    private static final String VUFORIA_CUBE_IMG = "vuImage.png";
    private static final String VUFORIA_ROI_IMG = "vuImageROI.png";
    private static final double SAMPLE_RATE_SEC = 0.05;
    private static final int RS_DIFF_THRESHOLD_CM = 5;
    private static final double DETECT_BUMP_THRESHOLD = 1.5;
    // Constants for runAuto
    private static final double MOVE_POWER_LOW = 0.3;
    private static final double MOVE_POWER_HIGH = 0.5;
    private static final boolean PARK = true;
    private static final boolean MOVE_FOUNDATION = true;
    private static final boolean TWO_STONES = false;
    private static final boolean WAIT = false;
    private static final int WAIT_SECONDS = 0;
    protected CCAuto.BoKAllianceColor allianceColor;
    protected ElapsedTime runTime = new ElapsedTime();
    protected CCAutoOpMode opMode;  // save a copy of the current opMode and robot
    protected CCHardwareBot robot;
    protected Orientation angles;
    private int YELLOW_PERCENT = 90;
    private AppUtil appUtil = AppUtil.getInstance();
    private VuforiaLocalizer vuforiaFTC;
    protected boolean noStone = true;
    // OpenCV Manager callback when we connect to the OpenCV manager
    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(appUtil.getActivity()) {
        @Override
        public void onManagerConnected(int status) {
            super.onManagerConnected(status);
        }
    };

    // All BoKAuto*OpModes must be derived from CCAutoCommon. They must override runSoftware
    // method in order to run specific methods from CCAutoCommon based on the missions.

    /**
     * writeFile
     * Helper method to save a copy of the image seen by the robot. This image can be opened in
     * Paint for further analysis as the image is saved in PNG format.
     *
     * @param fname
     * @param img
     * @param always
     */
    protected static void writeFile(String fname, Mat img, boolean always) {
        if (always || DEBUG_OPEN_CV) {
            String filePath = "/sdcard/FIRST/" + fname;
            //Log.v("BOK", "Saving image" + filePath);
            Imgcodecs.imwrite(filePath, img);
        }
    }

    @Override
    public void runSoftware() {
        runAuto(true, true);
    }

    /**
     * initSoftware
     * Initialize OpenCV, Vuforia. Setup the frame queue in Vuforia to 1 so that we can
     * grab a frame from Vuforia and run OpenCV algorithm on the frame.
     * Wait for the drive team to press X on Gamepad1 before initializing IMU.
     * Parameters:
     * 1. opMode: caller's opMode
     * 2. robot: hardware used by the opMode
     * Returns BOK_AUTO_SUCCESS
     */

    //@Override
    public CCAuto.BoKAutoStatus initSoftware(CCAutoOpMode opMode,
                                             CCHardwareBot robot) {
        loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        Log.v("BOK", "Initializing OpenCV");
        // Initialize OpenCV
        if (!OpenCVLoader.initDebug()) {
            Log.v("BOK", "initDebug False");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION,
                    appUtil.getActivity(), loaderCallback);
        } else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }


        Log.v("BOK", "Initializing Vuforia");
        // Initialize Vuforia
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the
         * RC phone); If no camera monitor is desired, use the parameterless constructor instead.
         */
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Vuforia License Key
        parameters.vuforiaLicenseKey = "ARRy24H/////AAABmWUQmKdoZknqg/9YdrBoyz1A6PA84DX24hMeuG/wA60YzwbhQoJHjFvdO0dHMALr1N9D3tIFlQNREybNz8TlycHSnb5bFqTlt3iBHwlgjz0ZRKwXVVIeX531cNltqKzaja0/WjfjU5baqWN2TdrivXUqwwd/+mjTyo2v/70pHQZ+mUuNO6Lnbw1xdRU1t8Jjf1zYZ9zp3eWAXl7ozKcI8VkBqnzhuU3EMOULrvQYC99dYp6G684cQc7jbXcvimQ2kBUdghB6IzmavVCUBDn4FUE99WzH7HxiW4wWRnSxxkcmDP32PeEtSlggRZDTIk1pKlFtMUo4739NQMc3ANaapZHhGWJYhV1KUVtkNjhB2S15";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforiaFTC = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforiaFTC.setFrameQueueCapacity(1); // change the frame queue capacity to 1

        Log.v("BOK", "Done initializing software");
        this.opMode = opMode;
        this.robot = robot;

        while (!opMode.gamepad1.x) {
            opMode.telemetry.addData("Status", "Press \"X\" to start gyro init");
            opMode.telemetry.update();
        }

        opMode.telemetry.addData("Status", "Initializing gyro");
        opMode.telemetry.update();
        setupRobot();
        opMode.telemetry.addData("Status", "Done initializing gyro!");
        opMode.telemetry.update();
        return CCAuto.BoKAutoStatus.BOK_AUTO_SUCCESS;


    }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    /**
     * setupRobot
     * Helper method called by initSoftware to complete any software setup tasks.
     */
    private void setupRobot() {
        // now initialize the IMU
        robot.initializeImu();
    }

    /**
     * setupOpenCVImg
     * Helper  method to setup OpenCV mat object from a rgb image in Vuforia.
     *
     * @param rgb
     * @param fileName
     * @param always
     * @return
     */
    private Mat setupOpenCVImg(Image rgb, String fileName, boolean always) {
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(),
                rgb.getHeight(),
                Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        Mat img = new Mat(rgb.getHeight(), rgb.getWidth(), CvType.CV_8UC3);
        Utils.bitmapToMat(bm, img);

        // OpenCV only deals with BGR
        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2BGR);
        writeFile(fileName, img, always);

        // First convert from BGR to HSV; separate the color components from intensity.
        // Increase robustness to lighting changes.
        // Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);
        return img;
    }

    /**
     * move
     * Algorithm to move forward or back using encoder sensor on the DC motors on the drive train
     * Parameters
     * leftPower, rightPower, inches to move, forward or back, waitForSec before timing out
     *
     * @param leftPower
     * @param rightPower
     * @param inches
     * @param forward
     * @param waitForSec
     */
    protected void move(double leftPower,
                        double rightPower,
                        double inches,
                        boolean forward,
                        double waitForSec) {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            robot.resetDTEncoders();
            robot.startMove(leftPower, rightPower, inches, forward);

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    /*(robot.getDTCurrentPosition() == false) &&*/
                    robot.areDTMotorsBusy()) {
                if (runTime.seconds() >= waitForSec) {
                    Log.v("BOK", "move timed out!" + String.format(" %.1f", waitForSec));
                    break;
                }
            }

            robot.stopMove();
        }
    }


    /**
     * moveRamp
     * Algorithm to move forward or back using encoder sensor on the DC motors on the drive train
     * using a ramp up and ramp down period to prevent the robot from getting jerks.
     * Parameters
     * maxPower, inches to move, forward or back, waitForSec before timing out
     *
     * @param maxPower
     * @param inches
     * @param forward
     * @param waitForSec
     */
    protected void moveRamp(double maxPower,
                            double inches,
                            boolean forward,
                            double waitForSec) {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            boolean steady = false;
            robot.resetDTEncoders();
            int targetEncCount = robot.startMove(DT_RAMP_SPEED_INIT,
                    DT_RAMP_SPEED_INIT,
                    inches,
                    forward);
            // speed up during the initial 1/4 enc counts
            // maintain max power during the next 1/2 enc counts
            // speed down during the last 1/4 enc counts
            int rampupEncCount = targetEncCount / 4;
            int rampdnEncCount = targetEncCount - rampupEncCount;
            double ratePower = (maxPower - DT_RAMP_SPEED_INIT) / rampupEncCount;

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    robot.areDTMotorsBusy()) {
                if (runTime.seconds() >= waitForSec) {
                    Log.v("BOK", "moveRamp timed out!" +
                            String.format(" %.1f", waitForSec));
                    break;
                }
                int lfEncCount = Math.abs(robot.getLFEncCount());
                if (lfEncCount < rampupEncCount) {
                    double power = DT_RAMP_SPEED_INIT + ratePower * lfEncCount;
                    robot.setPowerToDTMotors(power, forward);

                } else if (lfEncCount < rampdnEncCount) {
                    if (!steady) {
                        robot.setPowerToDTMotors(maxPower, forward);
                        steady = true;
                    }
                } else {
                    double power = DT_RAMP_SPEED_INIT -
                            ratePower * (lfEncCount - targetEncCount);
                    robot.setPowerToDTMotors(power, forward);
                }
            }
            // finally stop moving
            robot.stopMove();
        }
    }

    /**
     * strafe
     * Allows robot to move left and right by amount of motor rotations
     *
     * @param maxPower
     * @param rotations
     * @param right
     * @param waitForSec
     */
    protected void strafe(double maxPower,
                          double rotations,
                          boolean right,
                          double waitForSec) {
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            robot.resetDTEncoders();
            int targetEncCount = -1;
            try {
                targetEncCount = robot.startStrafeWEnc(maxPower, rotations, right);
            } catch (UnsupportedOperationException e) {
                return;
            }
            Log.v("BOK", "strafeRamp: " + targetEncCount);

            // speed up during the initial 1/4 enc counts
            // maintain max power during the next 1/2 enc counts
            // speed down during the last 1/4 enc counts
            int rampupEncCount = targetEncCount / 4;
            int rampdnEncCount = targetEncCount - rampupEncCount;
            double ratePower = (maxPower - DT_RAMP_SPEED_INIT) / rampupEncCount;

            runTime.reset();
            while (opMode.opModeIsActive() &&
                    (robot.getAvgEncCount() < targetEncCount)) {
                if (runTime.seconds() >= waitForSec) {
                    Log.v("BOK", "strafeRamp timed out!" + String.format(" %.1f", waitForSec));
                    break;
                }


            }
            robot.stopMove();
        }
    }

    /**
     * isCubePresent
     * Helper method that checks for yellow pixels to confirm the presence of the cube mineral. The
     * input image is in HSV format to prevent the effect of ambient light conditions.
     * This method should always return false because this is called by findCube only for spheres.
     *
     * @param imgHSV
     * @param roi
     * @return
     */
    private boolean isSkystone(Mat imgHSV, Rect roi) {
        Mat hist = new Mat();
        MatOfInt histSize = new MatOfInt(180);
        MatOfFloat ranges = new MatOfFloat(0f, 180f);
        Mat mask = new Mat(imgHSV.rows(), imgHSV.cols(),
                CvType.CV_8UC1, new Scalar(0));
        float[] resFloat = new float[180];
        boolean foundYellow = false;

        Imgproc.rectangle(imgHSV, new Point(roi.x, roi.y),
                new Point(roi.x + roi.width,
                        roi.y + roi.height),
                new Scalar(0, 255, 0), 10);

        Mat subMask;
        try {
            subMask = mask.submat(roi);
        } catch (CvException cvE) {
            Log.v("BOK", "Caught CvException " + cvE.toString());
            try {
                Rect newRoi = new Rect(roi.x, roi.y, roi.width / 2, roi.height / 2);
                roi = newRoi;
                subMask = mask.submat(roi);
            } catch (CvException e) {
                Log.v("BOK", "Caught another CvException!" + cvE.toString());
                return false;
            }
        }
        subMask.setTo(new Scalar(255));

        Imgproc.calcHist(Arrays.asList(imgHSV), new MatOfInt(0),
                mask, hist, histSize, ranges);
        //writeFile(HSV_IMG, img, DEBUG_OPEN_CV);
        //Core.normalize(hist, hist, 256, 0, Core.NORM_MINMAX);
        hist.get(0, 0, resFloat);

        int p, nYellowPixels = 0;
        int numPixels = roi.width * roi.height;
        // Red is 0 (in HSV),
        // but we need to check between 10 and 35
        for (p = 5; p < 20; p++) {
            nYellowPixels += (int) resFloat[p];
        }

        if (Math.abs(nYellowPixels) >= Math.abs(((numPixels * YELLOW_PERCENT) / 100))) {
            foundYellow = true;
            Log.v("BOK", "Yellow is true");
        }

        Log.v("BOK", "num Yellow pixels: " + nYellowPixels + " out of " + numPixels);
        Log.v("BOK", "nYellow: " + Math.abs(nYellowPixels) + " Percent: " + Math.abs(((numPixels * YELLOW_PERCENT) / 100)));
        hist.release();
        histSize.release();
        ranges.release();
        mask.release();

        return foundYellow;
    }

    /**
     * findCube
     * Finds the location of the cube (sampling mission).
     * It takes a frame from Vuforia frame queue. It converts the frame from rgb to gray. It uses
     * the HoughCircles algorithm to detect the spheres in the band of the picture where the
     * spheres are most likely to be found.
     *
     * @return
     */

    protected CCAutoStoneLocation findCube() {
        int numYellow = 0;
        CCAutoStoneLocation ret = CCAutoStoneLocation.CC_CUBE_RIGHT;
        VuforiaLocalizer.CloseableFrame frame;

        // takes the frame at the head of the queue
        try {
            frame = vuforiaFTC.getFrameQueue().take();
        } catch (InterruptedException e) {
            Log.v("BOK", "Exception while finding cube!!");
            return ret;
        }

        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                Image rgb = frame.getImage(i);
                // rgb is now the Image object that we’ve used in the video
                if (rgb != null) {
                    Mat src = setupOpenCVImg(rgb, VUFORIA_CUBE_IMG, true);
                    Mat srcHSV = new Mat();

                    Mat srcGray = new Mat(); // Convert image to gray scale
                    Imgproc.cvtColor(src, srcGray, Imgproc.COLOR_BGR2GRAY);
                    Imgproc.cvtColor(src, srcHSV, Imgproc.COLOR_BGR2HSV);
                    Size s = new Size(50, 50);

                    if (BoKAllianceColor.BOK_ALLIANCE_RED == allianceColor) {
                        Rect LeftRoi = new Rect(new Point(490, 130), s);
                        Rect CenterRoi = new Rect(new Point(700, 130), s);
                        Rect RightRoi = new Rect(new Point(980, 130), s);
                        boolean left = isSkystone(srcHSV, LeftRoi);
                        boolean center = isSkystone(srcHSV, CenterRoi);
                        boolean right = isSkystone(srcHSV, RightRoi);
                        if (!left && center && right) {
                            ret = CCAutoStoneLocation.CC_CUBE_LEFT;
                        }
                        if (!center && left && right) {
                            ret = CCAutoStoneLocation.CC_CUBE_CENTER;
                        }
                        if (!right && left && center) {
                            ret = CCAutoStoneLocation.CC_CUBE_RIGHT;
                        }
                        /*
                        if (center && left) {
                            ret = CCAutoStoneLocation.CC_CUBE_RIGHT;
                        }
                        if (!center && left) {
                            ret = CCAutoStoneLocation.CC_CUBE_CENTER;
                        }
                        if (!left && center) {
                            ret = CCAutoStoneLocation.CC_CUBE_LEFT;
                        }
                        */
                        writeFile(VUFORIA_ROI_IMG, srcHSV, true);
                        Log.v("BOK", "Left: " + left +
                                " Center: " + center + " Right: " + right);
                    }
                    if (BoKAllianceColor.BOK_ALLIANCE_BLUE == allianceColor) {
                        Rect LeftRoi = new Rect(new Point(250, 600), s);
                        Rect CenterRoi = new Rect(new Point(250, 400), s);
                        boolean left = isSkystone(srcHSV, LeftRoi);
                        boolean center = isSkystone(srcHSV, CenterRoi);
                        if (center && left) {
                            ret = CCAutoStoneLocation.CC_CUBE_RIGHT;
                        }
                        if (!center && left) {
                            ret = CCAutoStoneLocation.CC_CUBE_CENTER;
                        }
                        if (!left && center) {
                            ret = CCAutoStoneLocation.CC_CUBE_LEFT;
                        }
                        writeFile(VUFORIA_ROI_IMG, srcHSV, true);
                        Log.v("BOK", "Left: " + left +
                                " Center: " + center);
                    }


                }

            }
        }
        frame.close();
        Log.v("BOK", "Cube loc:" + ret.toString());
        return ret;
    }

    /**
     * takePicture
     * Helper method to take picture from Vuforia and save it to a file.
     *
     * @param sFileName
     */
    protected void takePicture(String sFileName) {
        VuforiaLocalizer.CloseableFrame frame;
        // takes the frame at the head of the queue
        try {
            frame = vuforiaFTC.getFrameQueue().take();
        } catch (InterruptedException e) {
            Log.v("BOK", "Exception while taking picture!!");
            return;
        }
        long numImages = frame.getNumImages();
        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                Image rgb = frame.getImage(i);
                // rgb is now the Image object that we’ve used in the video
                if (rgb != null) {
                    Mat img = setupOpenCVImg(rgb, sFileName, true);
                    img.release();
                }
                break;
            } // PIXEL_FORMAT.RGB565
        }
        frame.close();
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */

    public double gyroTurn(double speed,
                           double init_angle,
                           double angle,
                           int threshold,
                           boolean tank,
                           boolean leftTank,
                           double waitForSeconds) {
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() &&
                !onHeading(speed, init_angle, angle, threshold, tank, leftTank, P_TURN_COEFF)) {
            if (runTime.seconds() >= waitForSeconds) {
                Log.v("BOK", "gyroTurn timed out!" + String.format(" %.1f", waitForSeconds));
                break;
            }
            // Update telemetry & Allow time for other processes to run.
            opMode.telemetry.update();
            //opMode.sleep(CCHardwareBot.OPMODE_SLEEP_INTERVAL_MS_SHORT);
        }

        robot.setPowerToDTMotors(0);
        Log.v("BOK", "turnF: " + angles.thirdAngle);
        return angles.thirdAngle;
    }

    // Code copied from the sample PushbotAutoDriveByGyro_Linear

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    protected boolean onHeading(double speed,
                                double init_angle,
                                double angle,
                                int threshold,
                                boolean tank,
                                boolean leftTank,
                                double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);
        if (Math.abs(error) <= (Math.abs(angle - init_angle) * 0.25)) {
            // slow down as we are withing 1/4th of the target
            if (speed > DT_TURN_SPEED_LOW) {
                speed = DT_TURN_SPEED_LOW;
            }
            //speed /= 2;
        }

        if (Math.abs(error) <= threshold) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);

            rightSpeed = speed * steer;
            if (rightSpeed > 0) {
                rightSpeed = Range.clip(rightSpeed,
                        DT_TURN_SPEED_LOW,
                        DT_TURN_SPEED_HIGH);
            } else {
                rightSpeed = Range.clip(rightSpeed,
                        -DT_TURN_SPEED_HIGH,
                        -DT_TURN_SPEED_LOW);
            }

            if (!tank) {
                leftSpeed = rightSpeed;
            } else if (leftTank) {
                leftSpeed = rightSpeed;
                rightSpeed = 0;
            } else {
                leftSpeed = 0;
            }
        }

        // Send desired speeds to motors.
        robot.setOnHeading(leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of
     * reference; +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    protected double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES);
        robotError = targetAngle - angles.thirdAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    protected double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /**
     * followHeadingPID
     * Parameters:
     * heading: gyro heading (z axis)
     * power: input power (steady state)
     * dist: distance to move in inches
     * detectBump: true if the robot should stop after detecting bump
     * waitForSec: seconds before timing out
     *
     * @param heading
     * @param power
     * @param dist
     * @param detectBump
     * @param waitForSec
     */
    protected void followHeadingPID(double heading,
                                    double power,
                                    double dist,
                                    boolean detectBump,
                                    double waitForSec) {
        double angle, error, diffError, turn, speedL, speedR,
                sumError = 0, lastError = 0, lastTime = 0;
        double Kp = 0.1, Ki = 0, Kd = 0; // Ki = 0.165; Kd = 0.093;
        double targetEnc = robot.getTargetEncCount(dist); // convert inches to target enc count
        //String logString = "dTime,ang,err,sum,last,diff,turn,speedL,speedR\n";
        Log.v("BOK", "followHeadingPID: " + heading + ", dist: " + dist);
        robot.resetDTEncoders();
        runTime.reset();

        while (opMode.opModeIsActive() && (runTime.seconds() < waitForSec) &&
                (robot.getAvgEncCount() < targetEnc)) {
            double currTime = runTime.seconds();
            double deltaTime = currTime - lastTime;
            if (deltaTime >= SAMPLE_RATE_SEC) {
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES);
                angle = angles.thirdAngle;
                error = angle - heading;
                sumError = sumError * 0.66 + error;
                diffError = error - lastError;
                turn = Kp * error + Ki * sumError + Kd * diffError;
                speedL = power + turn;
                speedR = power - turn;
                speedR *= -1;

                speedL = Range.clip(speedL, DT_MOVE_LOW, Math.abs(2 * power));
                speedR = Range.clip(speedR, -Math.abs(2 * power), -DT_MOVE_LOW);

                speedL = Range.clip(speedL, -1, 1);
                speedR = Range.clip(speedR, -1, 1);
                robot.setPowerToDTMotors(speedL, speedR);

                lastError = error;
                lastTime = currTime;
                if (detectBump) {

                    if (Math.abs(angles.secondAngle) > DETECT_BUMP_THRESHOLD) {
                        Log.v("BOK", "theta y in if " + angles.secondAngle);
                        break;
                    }
                }
            }
        }
        robot.setPowerToDTMotors(0);
        if (runTime.seconds() >= waitForSec) {
            Log.v("BOK", "followHeadingPID timed out!");
        }

    }

    /**
     * moveWithRangeSensor
     * Parameters:
     * power: input power (steady state)
     * targetDistanceCm: distance to the wall before stopping the robot
     * capDistCm: max distance that can possibly be reported by the distance sensor
     * waitForSec: seconds before timing out
     *
     * @param power
     * @param targetDistanceCm
     * @param capDistCm
     * @param waitForSec
     */
    public void moveWithRangeSensor(double power,
                                    int targetDistanceCm,
                                    int capDistCm,
                                    double waitForSec) {
        double cmCurrent, diffFromTarget, pCoeff, wheelPower;
        robot.setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        AnalogInput rangeSensor = robot.distanceLeftBack;
        opMode.sleep(40);
        cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.25, opMode);
        Log.v("BOK", "moveWithRangeSensor: " + cmCurrent + ", target: " + targetDistanceCm);
        //if (!Double.isNaN(cmCurrent))
        diffFromTarget = targetDistanceCm - cmCurrent;
        runTime.reset();

        while (opMode.opModeIsActive() &&
                (Math.abs(diffFromTarget) >= RS_DIFF_THRESHOLD_CM)) {
            //Log.v("BOK", "Distance from wall "+cmCurrent);
            if (runTime.seconds() >= waitForSec) {
                Log.v("BOK", "moveWithRS timed out!" + String.format(" %.1f", waitForSec));
                break;
            }

            cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.25, opMode);
            if (Double.isNaN(cmCurrent) || (cmCurrent >= 255)) // Invalid sensor reading
                continue;

            diffFromTarget = targetDistanceCm - cmCurrent;
            pCoeff = diffFromTarget / 15;

            wheelPower = Range.clip(power * pCoeff, -Math.abs(power), Math.abs(power));
            if (wheelPower > 0 && wheelPower < 0.4)
                wheelPower = 0.4; // min power to move
            if (wheelPower < 0 && wheelPower > -0.4)
                wheelPower = -0.4;

            //Log.v("BOK", "CM current " + cmCurrent + " diff "+diffFromTarget);
            // back range sensor

            if (diffFromTarget < 0) {// we are still far away!
                robot.setPowerToDTMotors(Math.abs(wheelPower), false /* going back*/);
            } else {
                // if diffFromTarget > 0 then wheelPower is +ve
                // robot.setPowerToDTMotors(wheelPower, true /* going forward */);
            }
            //Log.v("BOK", "Back current RS: " + cmCurrent +
            //        " Difference: " + diffFromTarget +
            //        " Power: (move fwd) " + wheelPower);
        }

        robot.setPowerToDTMotors(0);
        if (runTime.seconds() >= waitForSec) {
            Log.v("BOK", "moveWithRangeSensor timed out!");
        }
        Log.v("BOK", "moveWithRangeSensor: " + cmCurrent);
    }

    /**
     * moveWithRangeSensorBack
     * Parameters:
     * power: input power (steady state)
     * targetDistanceCm: distance to the wall before stopping the robot
     * capDistCm: max distance that can possibly be reported by the distance sensor
     * waitForSec: seconds before timing out
     *
     * @param power
     * @param targetDistanceCm
     * @param capDistCm
     * @param waitForSec
     * @param rangeSensor
     * @param forward
     */
    public void moveWithRangeSensorBack(double power,
                                        int targetDistanceCm,
                                        int capDistCm,
                                        double waitForSec, AnalogInput rangeSensor, boolean forward) {
        double cmCurrent, diffFromTarget = targetDistanceCm, pCoeff, wheelPower;
        robot.setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //   AnalogInput rangeSensor = robot.distanceBack;
        opMode.sleep(40);
        cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.25, opMode);//0.25
        Log.v("BOK", "moveWithRangeSensorBack: " + cmCurrent + ", target: " + targetDistanceCm);

        //if (!Double.isNaN(cmCurrent))
        if (!forward) {
            diffFromTarget = targetDistanceCm - cmCurrent;
        } else {
            diffFromTarget = cmCurrent - targetDistanceCm;
        }
        runTime.reset();

        while (opMode.opModeIsActive() &&
                (Math.abs(diffFromTarget) >= RS_DIFF_THRESHOLD_CM)) {
            //Log.v("BOK", "Distance from wall "+cmCurrent);
            if (runTime.seconds() >= waitForSec) {
                Log.v("BOK", "moveWithRSBack timed out!" + String.format(" %.1f", waitForSec));
                break;
            }

            cmCurrent = robot.getDistanceCM(rangeSensor, capDistCm, 0.25, opMode);
            if (Double.isNaN(cmCurrent) || (cmCurrent >= 255)) // Invalid sensor reading
                continue;

            diffFromTarget = targetDistanceCm - cmCurrent;
            pCoeff = diffFromTarget / 15;
            wheelPower = Range.clip(power * pCoeff, -Math.abs(power), Math.abs(power));
            if (wheelPower > 0 && wheelPower < 0.4)
                wheelPower = 0.4; // min power to move
            if (wheelPower < 0 && wheelPower > -0.4)
                wheelPower = -0.4;

            //Log.v("BOK", "CM current " + cmCurrent + " diff "+diffFromTarget);
            // back range sensor
            if (diffFromTarget < 0 || forward) { // we are still far away!
                robot.setPowerToDTMotors(Math.abs(wheelPower), false /* going back*/);
            } else if (!forward) {
                // if diffFromTarget > 0 then wheelPower is +ve
                robot.setPowerToDTMotors(Math.abs(wheelPower), true /* going forward */);
            }
            //Log.v("BOK", "Back current RS: " + cmCurrent +
            //        " Difference: " + diffFromTarget +
            //        " Power: (move fwd) " + wheelPower);
        }

        robot.setPowerToDTMotors(0);
        if (runTime.seconds() >= waitForSec) {
            Log.v("BOK", "moveWithRangeSensorBack timed out!");
        }
        Log.v("BOK", "moveWithRangeSensorBack: " + cmCurrent);
    }

    protected void resetLift() {
        boolean reset = true;
        while (reset) {
            robot.liftLeftMotor.setTargetPosition(0);
            robot.liftRightMotor.setTargetPosition(0);
            robot.liftLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftLeftMotor.setPower(-0.4);
            robot.liftRightMotor.setPower(-0.4);
            if (!robot.liftLeftMotor.isBusy() && !robot.liftRightMotor.isBusy()) {
                reset = false;
            }
        }
    }

    protected void rateServoOut(double waitForSec) {
        boolean intServo = true;
        runTime.reset();
        while (intServo&&opMode.opModeIsActive()&& runTime.seconds() < waitForSec) {
            double rightGrip = robot.gripperRotateRightServo.getPosition();
            double leftGrip = robot.gripperRotateLeftServo.getPosition();
            double oriPos = robot.gripperOrientationServo.getPosition();
            if (rightGrip <= robot.ROTATE_UP_POS && leftGrip >= robot.ROTATE_UP_POS_LEFT && oriPos >= robot.ORI_UP) {
                robot.gripperOrientationServo.setPosition(robot.ORI_UP);
                robot.gripperRotateLeftServo.setPosition(robot.ROTATE_UP_POS_LEFT-0.1);
                robot.gripperRotateRightServo.setPosition(robot.ROTATE_UP_POS+0.1);
                intServo = false;
            }
            if (!(rightGrip <= robot.ROTATE_UP_POS+0.1)) {
                robot.gripperRotateRightServo.setPosition(rightGrip - 0.04);
            }
            if (!(leftGrip >= robot.ROTATE_UP_POS_LEFT-0.1)) {
                robot.gripperRotateLeftServo.setPosition(leftGrip + 0.04);
            }
            if (!(oriPos >= robot.ORI_UP)) {
                robot.gripperOrientationServo.setPosition(oriPos + 0.06);
            }
        }

    }

    protected void rateServoIn(double waitForSec) {
        boolean intGripServo = true;
        robot.gripperRotateRightServo.setPosition(robot.ROTATE_DOWN_POS);
        robot.gripperRotateLeftServo.setPosition(robot.ROTATE_DOWN_LEFT_POS);
        robot.gripperOrientationServo.setPosition(robot.ORI_DOWN);
        runTime.reset();
        while (intGripServo && opMode.opModeIsActive() && runTime.seconds() < waitForSec) {
            double gripPos = robot.gripperServo.getPosition();
            double rightGrip = robot.gripperRotateRightServo.getPosition();
            double leftGrip = robot.gripperRotateLeftServo.getPosition();
            double oriPos = robot.gripperOrientationServo.getPosition();
            if (gripPos >= robot.INTAKE_GRAB_POS) {
                robot.gripperRotateRightServo.setPosition(robot.ROTATE_DOWN_POS);
                robot.gripperRotateLeftServo.setPosition(robot.ROTATE_DOWN_LEFT_POS);
                robot.gripperOrientationServo.setPosition(robot.ORI_DOWN);
                robot.intakeRightMotor.setPower(0);
                robot.intakeLeftMotor.setPower(0);
                intGripServo = false;
            }

            if (!(leftGrip <= robot.ROTATE_DOWN_LEFT_POS)) {
                // robot.gripperRotateLeftServo.setPosition(leftGrip - 0.04);
            }
            if (!(rightGrip >= robot.ROTATE_DOWN_POS)) {
                // robot.gripperRotateRightServo.setPosition(rightGrip + 0.04);
            }
            if (!(gripPos >= robot.INTAKE_GRAB_POS)) {
                robot.gripperServo.setPosition(gripPos + 0.06);
            }
            if (!(oriPos <= robot.ORI_DOWN)) {
                // robot.gripperOrientationServo.setPosition(oriPos - 0.03);
            }

        }
    }

    protected boolean getStone(double pwr, double initAngle, boolean left) {
        robot.gripperRotateRightServo.setPosition(robot.ROTATE_DOWN_POS - 0.2);
        robot.gripperRotateLeftServo.setPosition(robot.ROTATE_DOWN_LEFT_POS + 0.2);
        robot.gripperOrientationServo.setPosition(robot.ORI_MID);
        robot.gripperServo.setPosition(robot.INTAKE_RELEASE_POS);
        if (left) {
            robot.intakeRightMotor.setPower(robot.INTAKE_POWER);
            robot.intakeLeftMotor.setPower(-robot.INTAKE_POWER);
         // gyroTurn(pwr, initAngle, 90, DT_TURN_THRESHOLD_LOW, false, true, 3);
            if (robot.opticalDistanceSensor.getLightDetected() > 0.7) {
                robot.flickerServo.setPosition(robot.FLICKER_SET);
                rateServoIn(2);
                return true;
            } else {
                move(0.1, 0.1, 5, true, 3);
                if (robot.opticalDistanceSensor.getLightDetected() > 0.7) {
                    robot.flickerServo.setPosition(robot.FLICKER_SET);
                    rateServoIn(2);
                    return true;
                } else {
                    robot.intakeRightMotor.setPower(robot.INTAKE_POWER);
                    robot.intakeLeftMotor.setPower(-robot.INTAKE_POWER);
                    return false;
                }
            }
        } else {
            robot.intakeRightMotor.setPower(robot.INTAKE_POWER);
            robot.intakeLeftMotor.setPower(-robot.INTAKE_POWER);
            gyroTurn(pwr, initAngle, -90, DT_TURN_THRESHOLD_LOW, false, true, 3);
            move(0.2, 0.2, 10, true, 3);
            if (robot.opticalDistanceSensor.getLightDetected() > 0.7) {
                robot.flickerServo.setPosition(robot.FLICKER_SET);
                rateServoIn(2);
                return true;
            } else {
                move(0.3, 0.3, 7, true, 3);
                if (robot.opticalDistanceSensor.getLightDetected() > 0.7) {
                    robot.flickerServo.setPosition(robot.FLICKER_SET);
                    rateServoIn(2);
                    return true;
                } else {
                    robot.intakeRightMotor.setPower(0);
                    robot.intakeLeftMotor.setPower(0);
                    return false;
                }
            }

        }

    }
    protected void tankWithGyro(double rightPower, double leftPower, double waitforSec){
        robot.setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runTime.reset();
        while(opMode.opModeIsActive() && runTime.seconds() < waitforSec){
            robot.setPowerToDTMotors(leftPower, rightPower);
            if((robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle >= 90) && (robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle <=  100)){
                break;
            }
            Log.v("BOK", "loop");
        }
        robot.stopMove();

    }
    protected void moveWithBothRangeSensors(double power,
                                            int targetDistanceCm,
                                            int capDistCm,
                                            double waitForSec, AnalogInput rangeSensor, AnalogInput secondRanggeSensor, boolean forward, boolean movingForward) {
        double cmCurrentOne, cmCurrentTwo, cmCurrent, diffFromTarget = targetDistanceCm, pCoeff, wheelPower;
        robot.setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //   AnalogInput rangeSensor = robot.distanceBack;
        opMode.sleep(40);
        secondRanggeSensor.close();
        cmCurrentOne = robot.getDistanceCM(rangeSensor, capDistCm, 0.25, opMode);//0.25
        rangeSensor.close();
        cmCurrentTwo = robot.getDistanceCM(secondRanggeSensor, capDistCm, 0.25, opMode);
        cmCurrent = (cmCurrentOne + cmCurrentTwo) / 2;
        runTime.reset();
        Log.v("BOK", "Sensor 1: " + cmCurrentOne + ", Sensor 2:  " + cmCurrentTwo + ", target: " + targetDistanceCm);
        while(!forward && !movingForward && (cmCurrentTwo < targetDistanceCm || cmCurrentOne < targetDistanceCm) &&opMode.opModeIsActive()&&runTime.seconds()<waitForSec){
            secondRanggeSensor.close();
            cmCurrentOne = robot.getDistanceCM(rangeSensor, capDistCm, 0.25, opMode);//0.25
            rangeSensor.close();
            cmCurrentTwo = robot.getDistanceCM(secondRanggeSensor, capDistCm, 0.25, opMode);
            cmCurrent = (cmCurrentOne + cmCurrentTwo) / 2;
            Log.v("BOK", "BEING DUMB");
        }
        double lastValOne = cmCurrentOne;
        double lastValTwo = cmCurrentTwo;
        //if (!Double.isNaN(cmCurrent))
        if (!forward) {
            diffFromTarget = ((targetDistanceCm - cmCurrentOne) + (targetDistanceCm - cmCurrentTwo))/2;
        } else {
            diffFromTarget = ((cmCurrentOne - targetDistanceCm) + (cmCurrentTwo - targetDistanceCm))/2;
        }


        while (opMode.opModeIsActive() &&
                (Math.abs(diffFromTarget) >= RS_DIFF_THRESHOLD_CM)) {
            //Log.v("BOK", "Distance from wall "+cmCurrent);
            if (runTime.seconds() >= waitForSec) {
                Log.v("BOK", "moveWithRSBack timed out!" + String.format(" %.1f", waitForSec));
                break;
            }

            secondRanggeSensor.close();
            opMode.sleep(100);
            cmCurrentOne = robot.getDistanceCM(rangeSensor, capDistCm, 0.25, opMode);//0.25
            rangeSensor.close();
            opMode.sleep(100);
            cmCurrentTwo = robot.getDistanceCM(secondRanggeSensor, capDistCm, 0.25, opMode);
            if((Math.abs(lastValOne - cmCurrentOne) > 40*(power*10) || Math.abs(lastValTwo- cmCurrentTwo) > 40*(power*10))){
                if(!movingForward) {
                    secondRanggeSensor.close();
                    opMode.sleep(100);
                    cmCurrentOne = robot.getDistanceCM(rangeSensor, capDistCm, 0.25, opMode);//0.25
                    rangeSensor.close();
                    opMode.sleep(100);
                    Log.v("BOK", "Sen not valid: " + 40*(power*10));
                }
            }
            if((Math.abs(lastValOne - cmCurrentOne) > 40*(power*10) || Math.abs(lastValTwo- cmCurrentTwo) > 40*(power*10))){
                if(!movingForward){
                    cmCurrentOne = lastValOne;
                    cmCurrentTwo = lastValTwo;
                    Log.v("BOK", "Sen not valid again: " + cmCurrentOne + ", " + cmCurrentTwo);
                }
            }

            lastValOne = cmCurrentOne;
            lastValTwo = cmCurrentTwo;
            //cmCurrent = (cmCurrentOne + cmCurrentTwo) / 2;
            if (Double.isNaN(cmCurrent) || (cmCurrent >= 255)) // Invalid sensor reading
                continue;

            diffFromTarget = ((targetDistanceCm - cmCurrentOne) + (targetDistanceCm - cmCurrentTwo))/2;
            pCoeff = diffFromTarget / 15;
            wheelPower = Range.clip(power * pCoeff, -Math.abs(power), Math.abs(power));
            if (wheelPower > 0 && wheelPower < 0.4)
                wheelPower = 0.4; // min power to move
            if (wheelPower < 0 && wheelPower > -0.4)
                wheelPower = -0.4;

            //Log.v("BOK", "CM current " + cmCurrent + " diff "+diffFromTarget);
            // back range sensor
            if(!movingForward) {
                if ((diffFromTarget < 0 || forward)) { // we are still far away!
                    robot.setPowerToDTMotors(Math.abs(wheelPower), false /* going back*/);
                }
                if(diffFromTarget > 0 && !forward){
                 //   robot.setPowerToDTMotors(Math.abs(wheelPower), true);
                    Log.v("BOK", "Should be For");
                }
            }
            else if (movingForward) {
                // if diffFromTarget > 0 then wheelPower is +ve
                robot.setPowerToDTMotors(Math.abs(wheelPower), true /* going forward */);
            }
            //Log.v("BOK", "Back current RS: " + cmCurrent +
            //        " Difference: " + diffFromTarget +
            //        " Power: (move fwd) " + wheelPower);
        }
        robot.stopMove();
        robot.setPowerToDTMotors(0);
        if (runTime.seconds() >= waitForSec) {
            Log.v("BOK", "moveWithRangeSensorBack timed out!");
        }
        Log.v("BOK", "Sensor 1: " + cmCurrentOne +", Sensor 2: " + cmCurrentTwo);
    }

    /**
     * dumpMarker
     * Dumps the marker by moving the marker servo.
     */
    protected void dumpMarker() {
        //  robot.markerServo.setPosition(robot.MARKER_SERVO_FINAL);
        opMode.sleep(500);
        //robot.markerServo.setPosition(robot.MARKER_SERVO_INIT);
    }

    /**
     * runAuto
     * Helper method called from CCAutoRedStoneInsideOpMode or CCAutoRedStoneOutsideMode
     *
     * @param inside
     * @param startStone
     */

    /*
     *Blue Stone Inside
     * Route Close to Mid
     *Blue Stone Outside
     * Route Away from Mid
     */
    protected void runAuto(boolean inside, boolean startStone) {

        CCAutoStoneLocation loc = CCAutoStoneLocation.CC_CUBE_UNKNOWN;
        Log.v("BOK", "Angle at runAuto start " +
                robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.DEGREES).thirdAngle);

        // Step 1: find skystone location
        try {
            loc = findCube();
        } catch (java.lang.Exception e) {
            Log.v("BOK", "Exception at findCube()");
        }

        Log.v("BOK", "StoneLoc: " + loc);

        Log.v("BOK", "Color: " + allianceColor + " Inside Route: " +
                inside + " Stating At Stone: " + startStone);
        runTime.reset();

        if (WAIT) {
            while (runTime.seconds() < WAIT_SECONDS && opMode.opModeIsActive()) {

            }

        }
        robot.flickerServo.setPosition(robot.FLICKER_INIT);
        if (loc == CCAutoStoneLocation.CC_CUBE_RIGHT) {
            robot.gripperRotateRightServo.setPosition(robot.ROTATE_DOWN_POS - 0.2);
            robot.gripperRotateLeftServo.setPosition(robot.ROTATE_DOWN_LEFT_POS + 0.2);
            robot.gripperOrientationServo.setPosition(robot.ORI_MID);
            robot.gripperServo.setPosition(robot.INTAKE_RELEASE_POS);
             gyroTurn(0.4, 0, 10, DT_TURN_THRESHOLD_LOW , false, false, 1);
            robot.intakeRightMotor.setPower(-robot.INTAKE_POWER);
          //  robot.intakeRightMotor.setPower(robot.INTAKE_POWER);
            robot.intakeLeftMotor.setPower(robot.INTAKE_POWER);

            //move(0.3, 0.2, 15, true, 3);
            //move(0.12, 0.04, 15, true, 3);
            move(0.2, 0.2, 25, true, 3);
            noStone = !getStone(0.4, 0, false);
            Log.v("BOK", "noStone: " + noStone);



            gyroTurn(0.38, robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, 0, DT_TURN_THRESHOLD_LOW,
                    false, false, 2);
            // move(0.15, 0.15, 15, false, 3);
           // opMode.sleep(500);
            moveWithBothRangeSensors(0.4, 40, 100, 3,
                    robot.distanceLeftBack, robot.distanceRightBack, false, false);
            opMode.sleep(250);
           // move(0.1, 0.1, 2, false, 2);
            gyroTurn(0.47, robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, 90, DT_TURN_THRESHOLD_LOW,
                    false, false, 2);
            //  move(0.15, 0.15, 75, false, 4);
            moveWithBothRangeSensors(0.4, 40, 200, 4, robot.distanceLeftBack, robot.distanceRightBack, false, false);
            opMode.sleep(500);
            //moveWithRangeSensorBack(0.15, 25, 200, 2,
            // robot.distanceLeftBack ,false);
            gyroTurn(0.45, robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, 180, DT_TURN_THRESHOLD_LOW,
                    false, false, 3);
            robot.foundationGripServo.setPosition(robot.FOUNDATION_GRIP_UP);
            moveWithBothRangeSensors(0.4, 70, 100, 2, robot.distanceRightForward, robot.distanceLeftForward, true, false);
            robot.foundationGripServo.setPosition(robot.FOUNDATION_GRIP_DOWN);
            opMode.sleep(500);

            // moveWithRangeSensorBack(0.5, 20, 100, 4, robot.distanceLeftForward, true);
            moveWithBothRangeSensors(0.9, 30, 100, 5, robot.distanceLeftForward, robot.distanceRightForward, true, true);
            tankWithGyro(0.9, 0.9, 5);
            move(0.7, 0.7, 10, false, 4);
            rateServoOut(2);
            robot.gripperServo.setPosition(robot.INTAKE_RELEASE_POS);
            robot.foundationGripServo.setPosition(robot.FOUNDATION_GRIP_UP);
            robot.gripperRotateRightServo.setPosition(robot.ROTATE_DOWN_POS - 0.2);
            robot.gripperRotateLeftServo.setPosition(robot.ROTATE_DOWN_LEFT_POS + 0.2);
            robot.gripperOrientationServo.setPosition(robot.ORI_MID);
            robot.gripperServo.setPosition(robot.INTAKE_RELEASE_POS);
            robot.flickerServo.setPosition(robot.FLICKER_INIT);
            move(0.2, 0.2, 10, true, 2);
            gyroTurn(0.45, robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
                    AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, 60, DT_TURN_THRESHOLD_LOW,
                    false, false, 3);
            move(0.3, 0.3, 30, true, 3);

            //move(1, -1, 100, true, 5);
           // move(0.7, 0.7, 30, false, 3);
/*
            if(!noStone){
                robot.liftLeftMotor.setTargetPosition(100);
                robot.liftRightMotor.setTargetPosition(100);
                robot.liftLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftLeftMotor.setPower(0.7);
                robot.liftRightMotor.setPower(0.7);
                while(robot.liftLeftMotor.isBusy() && opMode.opModeIsActive()){

                }
                robot.liftLeftMotor.setPower(0);
                robot.liftRightMotor.setPower(0);

                rateServoOut();
                robot.gripperServo.setPosition(robot.INTAKE_RELEASE_POS);
                robot.foundationGripServo.setPosition(robot.FOUNDATION_GRIP_UP);

                move(0.1, 0.1, 5, true, 3);

                resetLift();
                robot.liftLeftMotor.setPower(0);
                robot.liftRightMotor.setPower(0);



            }
            else{
                robot.foundationGripServo.setPosition(robot.FOUNDATION_GRIP_UP);
                move(0.1, 0.1, 5, true, 3);
            }
           // move(0.7, 0.9, 60, true, 4);
          //  move(0.9, 0.9, 20, false, 4);
           // gyroTurn(0.6, robot.imu.getAngularOrientation(AxesReference.INTRINSIC,
             //       AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle, 90, DT_TURN_THRESHOLD_LOW,
               //     false, false, 3);
        }

 */


            Log.v("BOK", "Auto Finished");
        }
    }

        public enum CCAutoStoneLocation {
            CC_CUBE_UNKNOWN,
            CC_CUBE_LEFT,
            CC_CUBE_CENTER,
            CC_CUBE_RIGHT
        }

}


