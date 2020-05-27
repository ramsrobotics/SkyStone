package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public abstract class CCHardwareBot {
    // CONSTANTS
    protected static final int OPMODE_SLEEP_INTERVAL_MS_SHORT = 10;

    protected static final double SPEED_COEFF_SLOW = 0.35;
    protected static final double SPEED_COEFF_FAST = 0.8;
    protected static final double SPEED_COEFF_TURN = 0.7;

    protected static final double GAME_STICK_DEAD_ZONE = 0.1;
    protected static final int WAIT_PERIOD = 40; // 40 ms
    //Motors
    private static final String LIFT_LEFT_MOTOR_NAME = "llM";
    private static final String LIFT_RIGHT_MOTOR_NAME = "lrM";
    private static final String INTAKE_LEFT_MOTOR_NAME = "ilM";
    private static final String INTAKE_RIGHT_MOTOR_NAME = "irM";
    //Servos
    private static final String GRIPPER_ROTATE_LEFT_SERVO_NAME = "glS";
    private static final String GRIPPER_ROTATE_RIGHT_SERVO_NAME = "grS";
    private static final String GRIPPER_SERVO_NAME = "giS";
    private static final String FOUNDATION_GRIP_SERVO = "fgS";
    private static final String BLOCK_FLICKER= "bfS";
    private static final String CAP_SERVO = "caS";
    private static final String LEFT_AUTO_GRAB = "laG";
    private static final String LEFT_AUTO_ROT = "laR";
    private static final String RIGHT_AUTO_GRAB = "raG";
    private static final String RIGHT_AUTO_ROT = "raR";
    //Sensors
    private static final String IMU_TOP = "imu";        // IMU
    private static final String DISTANCE_SENSOR_BACK = "dsB";
    private static final String DISTANCE_SENSOR_FRONT = "dsF";
    private static final String DISTANCE_SENSOR_RIGHT = "dsR";
    private static final String DISTANCE_SENSOR_LEFT = " dsL";
    private static final String ODS_BLOCK = "odS";
    protected final double INTAKE_GRAB_POS = 0.83;//0.65
    protected final double INTAKE_MID_POS = 0.4;
    protected final double INTAKE_RELEASE_POS = .96;

    protected final double INTAKE_POWER = 1;
    protected final double REVERSE_POWER = 0.4;

    protected final double ORI_DOWN = 0;//0.2
    protected final double ORI_MID = 0.33;//
    protected final double ORI_UP = 0.7;

    protected final double ROTATE_UP_POS_LEFT = 0.56;
    protected final double ROTATE_DOWN_LEFT_POS = 0.86;
    protected final double ROTATE_GRIP_LEFT_POS = 0.94;
    protected final double ROTATE_MID_LEFT_POS = 0.77;


    protected final double ROTATE_UP_POS = 0.44;
    protected final double ROTATE_DOWN_POS = 0.14;
    protected final double ROTATE_GRIP_POS = 0.06;
    protected final double ROTATE_MID_POS = 0.23;


    protected final double FOUNDATION_GRIP_DOWN = 1;
    protected final double FOUNDATION_GRIP_UP = .8;
    protected final double FOUNDATION_GRIP_INIT = 0;

    protected final double FLICKER_INIT = 0.5;
    protected final double FLICKER_SET = .9;



    protected final double AUTO_GRAB = 0;
    protected final double AUTO_OPEN = 0.7;

    protected final double AUTO_RIGHTI_IN = 0.35;
    protected final double AUTO_RIGHT_DEPLOY = 0.75;

    protected final double CAP_HOLD = 1;
    protected final double CAP_DROP = .45;
    // DC motors
    protected DcMotor liftLeftMotor;
    protected DcMotor liftRightMotor;
    protected DcMotor intakeLeftMotor;
    protected DcMotor intakeRightMotor;
    // Servos
    protected Servo gripperRotateLeftServo;
    protected Servo gripperRotateRightServo;
    protected Servo gripperOrientationServo;
    protected Servo gripperServo;
    protected Servo foundationGripServo;
    protected Servo flickerServo;
    protected Servo capStoneServo;
    protected Servo autoGripLeft;
    protected Servo autoGripRight;
    protected Servo autoROTLeft;
    protected Servo autoROTRight;

    // Sensors
    protected BNO055IMU imu;
    protected AnalogInput distanceLeft;
    protected AnalogInput distanceBack;
    protected AnalogInput distanceForward;
    protected AnalogInput distanceRight;
    protected OpticalDistanceSensor opticalDistanceSensor;
    LinearOpMode opMode; // current opMode
    private Orientation angles;

    // waitForTicks
    private ElapsedTime period = new ElapsedTime();
    private ElapsedTime runTime = new ElapsedTime();

    protected BoKHardwareStatus initHardware(LinearOpMode opMode) {
        this.opMode = opMode;
        // First initialize the drive train
        BoKHardwareStatus rc = initDriveTrainMotors();
        if (rc == BoKHardwareStatus.BOK_HARDWARE_SUCCESS) {
            // Next initialize the other motors and sensors
            rc = initMotorsAndSensors();
        }
        return rc;
    }

    /*
     * The initHardware() method initializes the hardware on the robot including the drive train.
     * It calls the abstract initDriveTrainMotors() and initMotorsAndSensors() methods.
     * Returns BOK_SUCCESS if the initialization is successful, BOK_FAILURE otherwise.
     */

    /*
     * The initMotorsAndSensors() method initializes the motors (other than the drive train) and
     * sensors on the robot.
     */
    private BoKHardwareStatus initMotorsAndSensors() {
        //Motors
        liftLeftMotor = opMode.hardwareMap.dcMotor.get(LIFT_LEFT_MOTOR_NAME);
        if(liftLeftMotor == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        liftRightMotor = opMode.hardwareMap.dcMotor.get(LIFT_RIGHT_MOTOR_NAME);
        if(liftRightMotor == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        intakeLeftMotor = opMode.hardwareMap.dcMotor.get(INTAKE_LEFT_MOTOR_NAME);
        if(intakeLeftMotor == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        intakeRightMotor = opMode.hardwareMap.dcMotor.get(INTAKE_RIGHT_MOTOR_NAME);
        if(intakeRightMotor == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        //Servos
        gripperRotateLeftServo = opMode.hardwareMap.servo.get(GRIPPER_ROTATE_LEFT_SERVO_NAME);
        if (gripperRotateLeftServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        gripperRotateRightServo = opMode.hardwareMap.servo.get(GRIPPER_ROTATE_RIGHT_SERVO_NAME);
        if(gripperRotateRightServo == null){
            return  BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }


        gripperServo = opMode.hardwareMap.servo.get(GRIPPER_SERVO_NAME);
        if(gripperServo == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        foundationGripServo = opMode.hardwareMap.servo.get(FOUNDATION_GRIP_SERVO);
        if(foundationGripServo == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        distanceForward = opMode.hardwareMap.analogInput.get(DISTANCE_SENSOR_FRONT);
        if(distanceForward == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        distanceLeft = opMode.hardwareMap.analogInput.get(DISTANCE_SENSOR_LEFT);
        if(distanceLeft == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        distanceBack = opMode.hardwareMap.analogInput.get(DISTANCE_SENSOR_BACK);
        if(distanceBack == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        distanceRight = opMode.hardwareMap.analogInput.get(DISTANCE_SENSOR_RIGHT);
        if(distanceRight == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        opticalDistanceSensor = opMode.hardwareMap.opticalDistanceSensor.get(ODS_BLOCK);
        if(opticalDistanceSensor == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        flickerServo = opMode.hardwareMap.servo.get(BLOCK_FLICKER);
        if(opticalDistanceSensor == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        capStoneServo = opMode.hardwareMap.servo.get(CAP_SERVO);
        if(capStoneServo == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        autoGripRight = opMode.hardwareMap.servo.get(RIGHT_AUTO_GRAB);
        if(autoGripRight == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        autoROTRight = opMode.hardwareMap.servo.get((RIGHT_AUTO_ROT));
        if(autoROTRight == null){
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

      /*  //Motors
        liftMotor = opMode.hardwareMap.dcMotor.get(LIFT_MOTOR_NAME);
        if (liftMotor == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        //Servos
        inRotateServo = opMode.hardwareMap.servo.get(INTAKE_ROTATE_SERVO_NAME);
        if (inRotateServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        intakeServo = opMode.hardwareMap.servo.get(INTAKE_SERVO_NAME);
        if (intakeServo == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        foundationGripServo = opMode.hardwareMap.servo.get(FOUNDATION_GRIP_SERVO);
        if (liftMotor == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        //Sensors



     distanceBack = opMode.hardwareMap.analogInput.get(DISTANCE_SENSOR_BACK);
        if (distanceBack == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        distanceForward = opMode.hardwareMap.analogInput.get(DISTANCE_SENSOR_FRONT);
        if (distanceForward == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

    */

        //Dc Motor Init
      //  liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Servos initialization
        /*
        if (opMode.getClass().getName().contains("Tele")) {
         //   intakeServo.setPosition(INTAKE_GRAB_POS);
           // inRotateServo.setPosition(ROTATE_UP_POS);
            //foundationGripServo.setPosition(FOUNDATION_GRIP_DOWN);
        } else {
            // Do nothing for Teleop so that the robot hardware does not move during
            // initialization
        }

         */
        imu = opMode.hardwareMap.get(BNO055IMU.class, IMU_TOP);
        if (imu == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }
        return BoKHardwareStatus.BOK_HARDWARE_SUCCESS;
    }

    protected void initializeImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }

    // Initialization of drive train is protected but abstract
    protected abstract BoKHardwareStatus initDriveTrainMotors();

    // Using the drive train is public
    protected abstract void testDTMotors();

   // protected abstract void moveTank(double leftPwr, double rightPwr, boolean rightPos, boolean leftPos);

    protected abstract void resetDTEncoders();

    protected abstract boolean areDTMotorsBusy();

    protected abstract void setPowerToDTMotors(double power);

    protected abstract void setPowerToDTMotors(double power, boolean forward);

    protected abstract void setPowerToDTMotors(double leftPower, double rightPower);

    protected abstract void setDTMotorEncoderTarget(int leftTarget, int rightTarget);

    protected abstract void setPowerToDTMotorsStrafe(double power, boolean right);

    protected abstract void setModeForDTMotors(DcMotor.RunMode runMode);

    protected abstract void setOnHeading(double leftPower, double rightPower);

    // Autonomous driving
    protected abstract int startMove(double leftPower,
                                     double rightPower,
                                     double inches,
                                     boolean backward);

    protected abstract void startEncMove(double leftPower,
                                         double rightPower,
                                         int encCount,
                                         boolean forward);

    protected abstract int startStrafe(double power, double rotations,
                                       boolean right) throws UnsupportedOperationException;

    protected abstract int startStrafeWEnc(double power, double rotations,
                                           boolean right) throws UnsupportedOperationException;

    protected abstract void stopMove();

    protected abstract double getTargetEncCount(double targetDistanceInches);

    protected abstract double getAvgEncCount();

    protected abstract int getLFEncCount();

    protected abstract int getRFEncCount();

    // Teleop driving
    protected abstract void moveRobotTele(double speedCoef);

    /**
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    protected void waitForTick(long periodMs) {
        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    protected double getAngle() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES);
        return angles.thirdAngle;
    }

    /**
     * getDistanceCM
     *
     * @param mb1240
     * @param target
     * @param time
     * @param opMode
     * @return
     */
    protected double getDistanceCM(AnalogInput mb1240, double target, double time, CCAutoOpMode opMode) {
        runTime.reset();
        double dist = (mb1240.getVoltage() / 0.003222);
        while (((dist > target) || (dist == 0)) && (runTime.seconds() <= time) && opMode.opModeIsActive())
            dist = (mb1240.getVoltage() / 0.00322);
        return (runTime.seconds() > time) ? target : dist;
        //return mb1240.getVoltage() / 0.00189;
    }



    // return status
    protected enum BoKHardwareStatus {
        BOK_HARDWARE_FAILURE,
        BOK_HARDWARE_SUCCESS
    }
}
