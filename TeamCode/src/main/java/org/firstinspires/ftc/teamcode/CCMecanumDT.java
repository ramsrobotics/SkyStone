package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Extends CCHardwareBot to implement the Mecanum wheels drive train with 4 DC Motors.
 */
public class CCMecanumDT extends CCHardwareBot {
    // CONSTANTS
    // 134.4 cycles per revolution (CPR); It is a quadrature encoder producing 4 Pulses per Cycle.
    // With 134.4 CPR, it outputs 537.6 PPR. AndyMark Orbital 20 Motor Encoder
    // For 360 degrees wheel turn, motor shaft moves 480 degrees (approx)
    private static final double COUNTS_PER_MOTOR_REV = 103.6;
    private static final double DRIVE_GEAR_REDUCTION = 2;
    private static final double WHEEL_DIAMETER_INCHES = 3.937;

    // CONSTANTS (strings from the robot config)
    private static final String LEFT_BACK_MOTOR_NAME = "lb";
    private static final String LEFT_FRONT_MOTOR_NAME = "lf";
    private static final String RIGHT_BACK_MOTOR_NAME = "rb";
    private static final String RIGHT_FRONT_MOTOR_NAME = "rf";

    // Drive train motors
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;

    // Strafe target
    // Commenting out strafe functionality
    private int leftFrontTarget;
    private int leftBackTarget;
    private int rightFrontTarget;
    private int rightBackTarget;

    /*
     * Implement all the abstract methods
     * Initialize the drive system variables.
     * The initDriveTrainMotors() method of the hardware class does all the work here
     */
    protected BoKHardwareStatus initDriveTrainMotors() {
        leftBack = opMode.hardwareMap.dcMotor.get(LEFT_BACK_MOTOR_NAME);
        if (leftBack == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        leftFront = opMode.hardwareMap.dcMotor.get(LEFT_FRONT_MOTOR_NAME);
        if (leftFront == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        rightBack = opMode.hardwareMap.dcMotor.get(RIGHT_BACK_MOTOR_NAME);
        if (rightBack == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        rightFront = opMode.hardwareMap.dcMotor.get(RIGHT_FRONT_MOTOR_NAME);
        if (rightFront == null) {
            return BoKHardwareStatus.BOK_HARDWARE_FAILURE;
        }

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Drive train is initialized, initialize sensors
        return BoKHardwareStatus.BOK_HARDWARE_SUCCESS;
    }

    /*
     * Set methods:
     * 1. set power
     * 2. set mode
     * 3. set motor encoder target
     */
    protected void setPowerToDTMotors(double left, double right) {
        leftBack.setPower(left);
        rightBack.setPower(right);
        leftFront.setPower(left);
        rightFront.setPower(right);
        opMode.sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
    }


    private void setPowerToDTMotors(double leftFrontPower, double leftBackPower,
                                    double rightFrontPower, double rightBackPower, boolean noSleep) {
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        if (!noSleep) {
            opMode.sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
        }
    }


    protected void setPowerToDTMotors(double power, boolean forward) {
        if (forward) {
            setPowerToDTMotors(power, power, -power, -power, false);
        } else {
            setPowerToDTMotors(-power, -power, power, power, false);
        }
    }

    protected void setPowerToDTMotors(double power) {
        setPowerToDTMotors(power, power, power, power, false);
    }

    protected void setPowerToDTMotorsStrafe(double power, boolean right) {
        if (right) {
            setPowerToDTMotors(power, -power, power , -power, false);
        } else {
            setPowerToDTMotors(-power, power, -power, power, false);
        }
    }

    protected void setOnHeading(double leftPower, double rightPower) {
        setPowerToDTMotors(-leftPower, -leftPower, -rightPower, -rightPower, false);
    }

    protected void setModeForDTMotors(DcMotor.RunMode runMode) {
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        opMode.sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
    }

    /*
     * getTargetEncCount(targetDistanceInches): returns the target encoder count
     * based on the wheel diameter, gear reduction ratio and counts per motor rev.
     */
    protected double getTargetEncCount(double targetDistanceInches) {
        double degreesOfWheelTurn, degreesOfMotorTurn;
        degreesOfWheelTurn = (360.0 / (Math.PI * CCMecanumDT.WHEEL_DIAMETER_INCHES)) *
                targetDistanceInches;
        degreesOfMotorTurn = CCMecanumDT.DRIVE_GEAR_REDUCTION * degreesOfWheelTurn;
        return (CCMecanumDT.COUNTS_PER_MOTOR_REV * degreesOfMotorTurn) / 360.0;
    }

    protected void resetDTEncoders() {
        // all four motors need encoder wires to use RUN_TO_POSITION
        setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeForDTMotors(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void setDTMotorEncoderTarget(int leftTarget, int rightTarget) {
        int currentLeftTarget = leftFront.getCurrentPosition() + leftTarget;
        int currentRightTarget = rightFront.getCurrentPosition() + rightTarget;

        leftFront.setTargetPosition(currentLeftTarget);
        leftBack.setTargetPosition(currentLeftTarget);
        rightFront.setTargetPosition(currentRightTarget);
        rightBack.setTargetPosition(currentRightTarget);

        // Turn On RUN_TO_POSITION
        setModeForDTMotors(DcMotor.RunMode.RUN_TO_POSITION);

        Log.v("BOK", "START: " + leftFront.getCurrentPosition() + ", " + currentLeftTarget + ", " +
                rightFront.getCurrentPosition() + ", " + currentRightTarget);
    }


    private void setDTMotorEncoderTargetStrafe(int leftFrontTarget,
                                               int leftBackTarget,
                                               int rightFrontTarget,
                                               int rightBackTarget) {
        int currentLeftFrontTarget = leftFront.getCurrentPosition() + leftFrontTarget;
        int currentLeftBackTarget = leftBack.getCurrentPosition() + leftBackTarget;
        int currentRightFrontTarget = rightFront.getCurrentPosition() + rightFrontTarget;
        int currentRightBackTarget = rightBack.getCurrentPosition() + rightBackTarget;

        leftFront.setTargetPosition(currentLeftFrontTarget);
        leftBack.setTargetPosition(currentLeftBackTarget);
        rightFront.setTargetPosition(currentRightFrontTarget);
        rightBack.setTargetPosition(currentRightBackTarget);

        // Turn On RUN_TO_POSITION
        setModeForDTMotors(DcMotor.RunMode.RUN_TO_POSITION);

        //Log.v("BOK", "START: LF: " + leftFront.getCurrentPosition() + ", " +
        //        currentLeftFrontTarget + ", LB: " +
        //        leftBack.getCurrentPosition() + ", " + currentLeftBackTarget + ", RF: " +
        //        rightFront.getCurrentPosition() + ",  " + currentRightFrontTarget + " RB: " +
        //        rightBack.getCurrentPosition() + ",  " + currentRightBackTarget);
    }

    /*
     * move() method: setup the robot to move encoder counts
     */
    protected int startMove(double leftPower,
                            double rightPower,
                            double inches,
                            boolean forward) {
        double targetEncCount = getTargetEncCount(inches);
        if (forward) {
            setDTMotorEncoderTarget((int) targetEncCount, (int) -targetEncCount);
            setPowerToDTMotors(leftPower, leftPower, -rightPower, -rightPower, false);
        } else {
            setDTMotorEncoderTarget((int) -targetEncCount, (int) targetEncCount);
            setPowerToDTMotors(-leftPower, -leftPower, rightPower, rightPower, false);
        }
        return (int) targetEncCount;
    }

    protected void startEncMove(double leftPower,
                                double rightPower,
                                int encCounts,
                                boolean forward) {
        if (forward) {
            setDTMotorEncoderTarget(encCounts, -encCounts);
            setPowerToDTMotors(leftPower, leftPower, -rightPower, -rightPower, false);
        } else {
            setDTMotorEncoderTarget(-encCounts, encCounts);
            setPowerToDTMotors(-leftPower, -leftPower, rightPower, rightPower, false);
        }
    }


    protected int startStrafe(double power, double rotations, boolean right) {
        double targetEncCount = (rotations * COUNTS_PER_MOTOR_REV) * DRIVE_GEAR_REDUCTION;
        if (right) {
            leftFrontTarget = (int) targetEncCount;
            leftBackTarget = (int) -targetEncCount;
            rightFrontTarget = (int) targetEncCount;
            rightBackTarget = (int) -targetEncCount;
            setDTMotorEncoderTargetStrafe(leftFrontTarget, leftBackTarget,
                    rightFrontTarget, rightBackTarget);
            setPowerToDTMotors(power, -power, power, -power, false);
        } else {
            leftFrontTarget = (int) -targetEncCount;
            leftBackTarget = (int) targetEncCount;
            rightFrontTarget = (int) -targetEncCount;
            rightBackTarget = (int) targetEncCount;
            setDTMotorEncoderTargetStrafe(leftFrontTarget, leftBackTarget,
                    rightFrontTarget, rightBackTarget);
            setPowerToDTMotors(-power, power, -power, power, false);
        }
        Log.v("BOK", "Target LF " + leftFrontTarget +
                ", LB " + leftBackTarget +
                ", RF" + rightFrontTarget +
                ", RB " + rightBackTarget);
        return (int) targetEncCount;
    }

    protected int startStrafeWEnc(double power, double rotations, boolean right) {
        resetDTEncoders();
        double targetEncCount = (rotations * COUNTS_PER_MOTOR_REV) * DRIVE_GEAR_REDUCTION;
        if (right) {
            setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setPowerToDTMotors(power , -power , power , -power , false);
            //setPowerToDTMotors(1, -1, 1, -1, false);
            //setPowerToDTMotors(11.5/13.0, -(12.5/13.0), 1, -(12/13.0), false);
        } else {
            setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //setPowerToDTMotors(-power*1.1, power*1.25, -power*1.31, power*1.28, false);
            setPowerToDTMotors(-power , power, -power, power, false);
            //setPowerToDTMotors(-power, power*1.25, -power*1.3, power*1.2, false);
        }
        return (int) targetEncCount;
    }


    protected void stopMove() {
        // Stop all motion;
        setPowerToDTMotors(0, 0, 0, 0, false);
        // Turn off RUN_TO_POSITION
        setModeForDTMotors(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected boolean areDTMotorsBusy() {
        //Log.v("BOK", "Current LF " + leftFront.getCurrentPosition() +
        //        ", RF " + rightFront.getCurrentPosition() +
        //        ", LB " + leftBack.getCurrentPosition() +
        //        ", RB " + rightBack.getCurrentPosition());
        return (leftFront.isBusy() &&
                rightFront.isBusy() &&
                leftBack.isBusy() &&
                rightBack.isBusy());
    }

    protected int getLFEncCount() {
        return leftFront.getCurrentPosition();
    }

    protected int getRFEncCount() {
        return rightFront.getCurrentPosition();
    }

    protected double getAvgEncCount() {
        return (Math.abs(rightBack.getCurrentPosition()) +
                Math.abs(rightFront.getCurrentPosition()) +
                Math.abs(leftBack.getCurrentPosition()) +
                Math.abs(leftFront.getCurrentPosition())) / 4.0;
    }

    protected void moveRobotTele(double speedCoef) {
        /*
         * Gamepad1: Driver 1 controls the robot using the left joystick for throttle and
         * the right joystick for steering
         */
        // NOTE: the left joystick goes negative when pushed upwards
        double gamePad1LeftStickY = opMode.gamepad1.left_stick_y;
        double gamePad1LeftStickX = opMode.gamepad1.left_stick_x;
        double gamePad1RightStickY = opMode.gamepad1.right_stick_y;
        double gamePad1RightStickX = opMode.gamepad1.right_stick_x;

        if (speedCoef == SPEED_COEFF_FAST) {
            gamePad1LeftStickX = Math.pow(gamePad1LeftStickX, 3);
            gamePad1LeftStickY = Math.pow(gamePad1LeftStickY, 3);
            gamePad1RightStickY = Math.pow(gamePad1RightStickY, 3);
            gamePad1RightStickX = Math.pow(gamePad1RightStickX, 3);
        }

        double speedCoefLocal = speedCoef;
        double motorPowerLF = 0;
        double motorPowerLB = 0;
        double motorPowerRF = 0;
        double motorPowerRB = 0;

        //Log.v("BOK","moveRobot: " + String.format("%.2f", gamePad1LeftStickY) + ", " +
        //        String.format("%.2f", gamePad1LeftStickX) + ", " +
        //        String.format("%.2f", gamePad1RightStickX));

        // Run mecanum wheels

        if (((Math.abs(gamePad1LeftStickY) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickY) < -GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickX) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1LeftStickX) < -GAME_STICK_DEAD_ZONE) || (Math.abs(gamePad1RightStickY) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1RightStickY) < -GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1RightStickX) > GAME_STICK_DEAD_ZONE) ||
                (Math.abs(gamePad1RightStickX) < -GAME_STICK_DEAD_ZONE))) {
            motorPowerLF = -gamePad1LeftStickY - (-gamePad1LeftStickX);
            motorPowerLB = -gamePad1LeftStickY - gamePad1LeftStickX;
            motorPowerRF = gamePad1RightStickY - (-gamePad1RightStickX);
            motorPowerRB = gamePad1RightStickY - gamePad1RightStickX;
            //Log.v("BOK","LF:" + String.format("%.2f", motorPowerLF*speedCoef) +
            //       "LB: " + String.format("%.2f", motorPowerLB*speedCoef) +
            //        "RF: " + String.format("%.2f", motorPowerRF*speedCoef) +
            //        "RB: " + String.format("%.2f", motorPowerRB*speedCoef));
        } /*else if ((gamePad1RightStickX > GAME_STICK_DEAD_ZONE) ||
                (gamePad1RightStickX < -GAME_STICK_DEAD_ZONE)) {
            // Right joystick is for turning

            //first and last
            motorPowerLF = gamePad1RightStickX;
            motorPowerLB = gamePad1RightStickX;
            motorPowerRF = gamePad1RightStickX;
            motorPowerRB = gamePad1RightStickX;

            speedCoefLocal = SPEED_COEFF_TURN;
            */
            //Log.v("BOK","Turn: LF:" + String.format("%.2f", motorPowerLF) +
            //        "LB: " + String.format("%.2f", motorPowerLB) +
            //        "RF: " + String.format("%.2f", motorPowerRF) +
            //        "RB: " + String.format("%.2f", motorPowerRB));

        setPowerToDTMotors((motorPowerLF * speedCoefLocal),
                (motorPowerLB * speedCoefLocal),
                (motorPowerRF * speedCoefLocal),
                (motorPowerRB * speedCoefLocal), true);
        //Log.v("BOK", "Moving in MechanumDT");
    }

    protected void testDTMotors() {
        leftFront.setTargetPosition(1000);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setPower(0.5);
        Log.v("BOK", "leftFront set power");
        while (opMode.opModeIsActive() && leftFront.isBusy()) {
            //Log.v("BOK", "LF enc at: " + leftFront.getCurrentPosition());
        }
        leftFront.setPower(0);
        //Log.v("BOK", "leftFront finished");

        rightFront.setTargetPosition(1000);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setPower(0.5);
        Log.v("BOK", "rightFront set power");
        while (opMode.opModeIsActive() && rightFront.isBusy()) {
            //Log.v("BOK", "RF enc at: " + rightFront.getCurrentPosition());
        }
        rightFront.setPower(0);
        //Log.v("BOK", "rightFront finished");

        leftBack.setTargetPosition(1000);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setPower(0.5);
        Log.v("BOK", "leftBack set power");
        while (opMode.opModeIsActive() && leftBack.isBusy()) {
            //Log.v("BOK", "LB enc at: " + leftBack.getCurrentPosition());
        }
        leftBack.setPower(0);
        Log.v("BOK", "leftBack finished");

        rightBack.setTargetPosition(1000);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setPower(0.5);
        Log.v("BOK", "rightBack set power");
        while (opMode.opModeIsActive() && rightBack.isBusy()) {
            //Log.v("BOK", "RF enc at: " + rightBack.getCurrentPosition());
        }
        rightBack.setPower(0);
        //Log.v("BOK", "rightBack finished");
    }


    protected void moveTank(double leftPwr, double righPwr, boolean rightPos, boolean leftPos) {

    }
}