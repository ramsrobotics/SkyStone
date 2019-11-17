package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class simpleAutoLin extends LinearOpMode {

    private static final double   COUNTS_PER_MOTOR_REV    = 537.6;
    private static final double   DRIVE_GEAR_REDUCTION    = 0.5;
    private static final double   WHEEL_DIAMETER_INCHES   = 4.0;
    protected static final int OPMODE_SLEEP_INTERVAL_MS_SHORT  = 10;

    // CONSTANTS (strings from the robot config)
    private static final String LEFT_BACK_MOTOR_NAME   = "lb";
    private static final String LEFT_FRONT_MOTOR_NAME  = "lf";
    private static final String RIGHT_BACK_MOTOR_NAME  = "rb";
    private static final String RIGHT_FRONT_MOTOR_NAME = "rf";

    // Drive train motors
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    protected ElapsedTime runTime  = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        leftBack = hardwareMap.dcMotor.get(LEFT_BACK_MOTOR_NAME);


        leftFront = hardwareMap.dcMotor.get(LEFT_FRONT_MOTOR_NAME);


        rightBack = hardwareMap.dcMotor.get(RIGHT_BACK_MOTOR_NAME);


        rightFront = hardwareMap.dcMotor.get(RIGHT_FRONT_MOTOR_NAME);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        // Run the autonomous operation, if hardware and software is initialized
        // else do nothing
        if (opModeIsActive()) {

            resetDTEncoders();
            startMove(0.3, 0.3, 24, true);

            runTime.reset();
            while (opModeIsActive() &&
                    /*(robot.getDTCurrentPosition() == false) &&*/
                    areDTMotorsBusy()) {
                if (runTime.seconds() >= 5) {
                    Log.v("BOK", "move timed out!" + String.format(" %.1f", 5));
                    break;
                }
            }

            stopMove();
        }
    }
    protected void setPowerToDTMotors(double left, double right)
    {
        leftBack.setPower(left);
        rightBack.setPower(right);
        leftFront.setPower(left);
        rightFront.setPower(right);
        sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
    }

    private void setPowerToDTMotors(double leftFrontPower, double leftBackPower,
                                    double rightFrontPower, double rightBackPower, boolean noSleep)
    {
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        if (!noSleep) {
            sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
        }
    }

    protected void setPowerToDTMotors(double power, boolean forward)
    {
        if (forward) {
            setPowerToDTMotors(power, power, -power, -power, false);
        } else {
            setPowerToDTMotors(-power, -power, power, power, false);
        }
    }

    protected void setPowerToDTMotors(double power)
    {
        setPowerToDTMotors(power, power, power, power, false);
    }

    /*protected void setPowerToDTMotorsStrafe(double power, boolean right)
    {
        if (right) {
            setPowerToDTMotors(power*1.6, -power*0.7, power*0.7, -power*1.6, false);
        }
        else {
            setPowerToDTMotors(-power*1.6, 0.7*power, 0.7*-power, power*1.6, false);
        }
    }*/

    protected void setOnHeading(double leftPower, double rightPower)
    {
        setPowerToDTMotors(-leftPower, -leftPower, -rightPower, -rightPower, false);
    }

    protected void setModeForDTMotors(DcMotor.RunMode runMode)
    {
        leftBack.setMode(runMode);
        rightBack.setMode(runMode);
        leftFront.setMode(runMode);
        rightFront.setMode(runMode);
        sleep(OPMODE_SLEEP_INTERVAL_MS_SHORT);
    }

    /*
     * getTargetEncCount(targetDistanceInches): returns the target encoder count
     * based on the wheel diameter, gear reduction ratio and counts per motor rev.
     */
    protected double getTargetEncCount(double targetDistanceInches)
    {
        double degreesOfWheelTurn, degreesOfMotorTurn;
        degreesOfWheelTurn = (360.0 / (Math.PI * WHEEL_DIAMETER_INCHES)) *
                targetDistanceInches;
        degreesOfMotorTurn = DRIVE_GEAR_REDUCTION * degreesOfWheelTurn;
        return (COUNTS_PER_MOTOR_REV * degreesOfMotorTurn) / 360.0;
    }

    protected void resetDTEncoders()
    {
        // all four motors need encoder wires to use RUN_TO_POSITION
        setModeForDTMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModeForDTMotors(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDTMotorEncoderTarget(int leftTarget, int rightTarget)
    {
        int currentLeftTarget = leftFront.getCurrentPosition() + leftTarget;
        int currentRightTarget = rightFront.getCurrentPosition() + rightTarget;

        leftFront.setTargetPosition(currentLeftTarget);
        leftBack.setTargetPosition(currentLeftTarget);
        rightFront.setTargetPosition(currentRightTarget);
        rightBack.setTargetPosition(currentRightTarget);

        // Turn On RUN_TO_POSITION
        setModeForDTMotors(DcMotor.RunMode.RUN_TO_POSITION);

        Log.v("BOK", "START: " + leftFront.getCurrentPosition() +", " + currentLeftTarget + ", " +
                rightFront.getCurrentPosition() + ", " + currentRightTarget);
    }

    /*
    private void setDTMotorEncoderTargetStrafe(int leftFrontTarget,
                                               int leftBackTarget,
                                              int rightFrontTarget,
                                              int rightBackTarget)
    {
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
    } */

    /*
     * move() method: setup the robot to move encoder counts
     */
    protected int  startMove(double leftPower,
                            double rightPower,
                            double inches,
                            boolean forward)
    {
        double targetEncCount = getTargetEncCount(inches);
        if (forward) {
            setDTMotorEncoderTarget((int) targetEncCount, (int) -targetEncCount);
            setPowerToDTMotors(leftPower, leftPower, -rightPower, -rightPower, false);
        }
        else {
            setDTMotorEncoderTarget((int) -targetEncCount, (int) targetEncCount);
            setPowerToDTMotors(-leftPower, -leftPower, rightPower, rightPower, false);
        }
        return (int)targetEncCount;
    }

    protected void startEncMove(double leftPower,
                                double rightPower,
                                int encCounts,
                                boolean forward)
    {
        if (forward) {
            setDTMotorEncoderTarget(encCounts, -encCounts);
            setPowerToDTMotors(leftPower, leftPower, -rightPower, -rightPower, false);
        }
        else {
            setDTMotorEncoderTarget(-encCounts, encCounts);
            setPowerToDTMotors(-leftPower, -leftPower, rightPower, rightPower, false);
        }
    }

    /*
    protected int startStrafe(double power, double rotations, boolean right)
    {
        double targetEncCount = (rotations*COUNTS_PER_MOTOR_REV) * DRIVE_GEAR_REDUCTION;
        if (right) {
            leftFrontTarget = (int) targetEncCount;
            leftBackTarget = (int) -targetEncCount;
            rightFrontTarget = (int) targetEncCount;
            rightBackTarget = (int)-targetEncCount;
            setDTMotorEncoderTargetStrafe(leftFrontTarget, leftBackTarget,
                    rightFrontTarget, rightBackTarget);
            setPowerToDTMotors(power, -power, power, -power, false);
        }
        else {
            leftFrontTarget = (int) -targetEncCount;
            leftBackTarget = (int) targetEncCount;
            rightFrontTarget = (int) -targetEncCount;
            rightBackTarget = (int)targetEncCount;
            setDTMotorEncoderTargetStrafe(leftFrontTarget, leftBackTarget,
                    rightFrontTarget, rightBackTarget);
            setPowerToDTMotors(-power, power, -power, power, false);
        }
        Log.v("BOK", "Target LF " + leftFrontTarget +
                ", LB " + leftBackTarget +
                ", RF" + rightFrontTarget +
                ", RB " + rightBackTarget);
        return (int)targetEncCount;
    }

    protected int startStrafeWEnc(double power, double rotations, boolean right)
    {
        resetDTEncoders();
        double targetEncCount = (rotations*COUNTS_PER_MOTOR_REV) * DRIVE_GEAR_REDUCTION;
        if (right) {
            setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setPowerToDTMotors(power*1.15, -power*1.25, power*1.3, -power*1.2, false);
            //setPowerToDTMotors(1, -1, 1, -1, false);
            //setPowerToDTMotors(11.5/13.0, -(12.5/13.0), 1, -(12/13.0), false);
        }
        else {
            setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //setPowerToDTMotors(-power*1.1, power*1.25, -power*1.31, power*1.28, false);
            setPowerToDTMotors(-(power*2), power, -power, (power*2), false);
            //setPowerToDTMotors(-power, power*1.25, -power*1.3, power*1.2, false);
        }
        return (int)targetEncCount;
    }
    */

    protected void stopMove()
    {
        // Stop all motion;
        setPowerToDTMotors(0, 0, 0, 0, false);
        // Turn off RUN_TO_POSITION
        setModeForDTMotors(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected boolean areDTMotorsBusy()
    {
        //Log.v("BOK", "Current LF " + leftFront.getCurrentPosition() +
        //        ", RF " + rightFront.getCurrentPosition() +
        //        ", LB " + leftBack.getCurrentPosition() +
        //        ", RB " + rightBack.getCurrentPosition());
        return (leftFront.isBusy() &&
                rightFront.isBusy() &&
                leftBack.isBusy() &&
                rightBack.isBusy());
    }

    protected int getLFEncCount()
    {
        return leftFront.getCurrentPosition();
    }

    protected int getRFEncCount()
    {
        return rightFront.getCurrentPosition();
    }

    protected double getAvgEncCount()
    {
        return (Math.abs(rightBack.getCurrentPosition()) +
                Math.abs(rightFront.getCurrentPosition()) +
                Math.abs(leftBack.getCurrentPosition()) +
                Math.abs(leftFront.getCurrentPosition()))/4.0;
    }


    protected void testDTMotors()
    {
        leftFront.setTargetPosition(1000);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setPower(0.5);
        Log.v("BOK", "leftFront set power");
        while(opModeIsActive() && leftFront.isBusy()){
            //Log.v("BOK", "LF enc at: " + leftFront.getCurrentPosition());
        }
        leftFront.setPower(0);
        //Log.v("BOK", "leftFront finished");

        rightFront.setTargetPosition(1000);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setPower(0.5);
        Log.v("BOK", "rightFront set power");
        while(opModeIsActive() && rightFront.isBusy()){
            //Log.v("BOK", "RF enc at: " + rightFront.getCurrentPosition());
        }
        rightFront.setPower(0);
        //Log.v("BOK", "rightFront finished");

        leftBack.setTargetPosition(1000);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setPower(0.5);
        Log.v("BOK", "leftBack set power");
        while(opModeIsActive() && leftBack.isBusy()){
            //Log.v("BOK", "LB enc at: " + leftBack.getCurrentPosition());
        }
        leftBack.setPower(0);
        Log.v("BOK", "leftBack finished");

        rightBack.setTargetPosition(1000);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setPower(0.5);
        Log.v("BOK", "rightBack set power");
        while(opModeIsActive() && rightBack.isBusy()){
            //Log.v("BOK", "RF enc at: " + rightBack.getCurrentPosition());
        }
        rightBack.setPower(0);
        //Log.v("BOK", "rightBack finished");
    }
}
