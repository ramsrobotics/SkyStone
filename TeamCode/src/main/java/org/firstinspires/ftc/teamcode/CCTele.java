package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import com.sun.tools.javac.comp.Todo;

/**
 * Created by Krishna Saxena on 10/3/2017.
 */

public class CCTele
{

    // CONSTANTS
    //private static final double GAME_STICK_DEAD_ZONE_LEFT_STICK = 0.3;
    private static final double GAME_TRIGGER_DEAD_ZONE = 0.2;
    private static final double INTAKE_MOTOR_CAP_SPEED = 0.9;
    private static final double Kp = 0.7, Ki = 0.525, Kd = 0.2;
    double lastErr = 0, sumErr = 0, dErrDT = 0, pid = 0, lastTime = 0, vTarget, powerSetPoint;
    int inPos, endPos, lastPos;

    private CCHardwareBot robot;
    private LinearOpMode opMode;
    private double speedCoef = robot.SPEED_COEFF_FAST;
    private boolean end_game = false;
    private boolean isLiftingIntakeArm = false;
    private boolean hasMovedIntakeArm = false;

    ElapsedTime intakeArmRunTime;

    public enum BoKTeleStatus
    {
        BOK_TELE_FAILURE,
        BOK_TELE_SUCCESS
    }

    public BoKTeleStatus initSoftware(LinearOpMode opMode,
                                      CCHardwareBot robot)
    {
        this.opMode = opMode;
        this.robot = robot;
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     //   robot.dumperSlideMotor.setPower(0);
       // robot.hangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // robot.hangMotor.setPower(0);
        //intakeArmRunTime = new ElapsedTime();
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    public BoKTeleStatus runSoftware() {
     /*   // Constants
        double DUMPER_LIFT_POWER = 0.5;
        double DUMPER_ROTATE_DECR = 0.02;
        double HANG_LIFT_POWER = 0.9;

        double nextPos = robot.DUMPER_ROTATE_SERVO_INIT;
        int currentIntakeArmPosition = 0;

        boolean hangHookRaised = false;
        boolean liftUp = false;
        boolean dump = false;
        boolean intakeArmDown = false;
        boolean isRunningIntakeArmPID = false;

        // Initialization after Play is pressed!
        robot.dumperRotateServo.setPosition(robot.DUMPER_RECEIVE_SERVO);

      */

        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            // GAMEPAD 1 CONTROLS:
            // Left & Right stick: Drive
            // A:                  Go in fast mode
            // Y:                  Go in slow mode

            moveRobot();

            if (opMode.gamepad1.y) {
                speedCoef = robot.SPEED_COEFF_SLOW;
            }
            if (opMode.gamepad1.a) {
                speedCoef = robot.SPEED_COEFF_FAST;
            }

            // GAMEPAD 2 CONTROLS
            // Left Joystick:          Slide Up and Down and move out and back

           //

            // Right Joystick:         Intake sweeper motor







            robot.liftMotor.setPower(-opMode.gamepad2.left_stick_y);


            if(opMode.gamepad2.a){
                robot.intakeServo.setPosition(robot.INTAKE_GRAB_POS);
            }
            if(opMode.gamepad2.b){
                robot.intakeServo.setPosition(robot.INTAKE_RELEASE_POS);
            }

            if(opMode.gamepad2.dpad_up){
                robot.inRotateServo.setPosition(robot.ROTATE_UP_POS);
            }
            if(opMode.gamepad2.dpad_down){
                robot.inRotateServo.setPosition(robot.ROTATE_DOWN_POS);
            }




/*
            if (opMode.gamepad2.x && !end_game) {
                end_game = true;
                Log.v("BOK", "End Game started");
                // Make sure that the intake arm is folded up

                // Make sure that the dumper lift is down
                robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                robot.dumperSlideMotor.setTargetPosition(0);
                robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER);
            }

            if (opMode.gamepad2.b && end_game) {
                end_game = false;

                Log.v("BOK", "End Game reverted");
            }

            if (!end_game) {
                moveIntake(); // Intake sweeper

                if (opMode.gamepad2.dpad_up && !liftUp) {
                    liftUp = true;
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                    robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dumperSlideMotor.setTargetPosition(robot.DUMPER_SLIDE_FINAL_POS);
                    robot.dumperSlideMotor.setPower(0.7);
                }
                if (opMode.gamepad2.dpad_down && liftUp) {
                    //robot.plateTilt.setPosition(robot.PLATE_TILT_LOW);
                    //robot.dumperTiltServo.setPosition(robot.DUMPER_TILT_SERVO_INIT);
                    robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dumperSlideMotor.setTargetPosition(10);
                    robot.dumperSlideMotor.setPower(-0.7);
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                    nextPos = robot.DUMPER_ROTATE_SERVO_INIT;
                    dump = false;
                    liftUp = false;
                }
                else { // Neither Dpad Up or Dpad Down is pressed
                    // Hold the lift's last position, but there is a minimum so that the
                    // string remains tight.
                    if (liftUp && !robot.dumperSlideMotor.isBusy()) {
                        robot.dumperSlideMotor.setPower(0);
                        robot.dumperSlideMotor.setTargetPosition(
                                robot.dumperSlideMotor.getCurrentPosition());
                        robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.dumperSlideMotor.setPower(DUMPER_LIFT_POWER);
                    }
                }

                // Dump minerals
                if (opMode.gamepad2.left_bumper && !dump) {
                    dump = opMode.gamepad2.left_bumper;
                    //Log.v("BOK", "LEFT bumper" + nextPos);
                }
                if (dump && (nextPos > robot.DUMPER_ROTATE_SERVO_FINAL)) {
                    nextPos -= DUMPER_ROTATE_DECR;
                    robot.dumperRotateServo.setPosition(nextPos);
                    //Log.v("BOK", "decrement" + nextPos);
                }
                if (opMode.gamepad2.right_bumper && dump) {
                    dump = false;
                    nextPos = robot.DUMPER_ROTATE_SERVO_INIT;
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                }

                // Intake arm control
                if (opMode.gamepad2.a && !isLiftingIntakeArm){
                    if(!isRunningIntakeArmPID){
                        isRunningIntakeArmPID = true;
                        intakeArmDown = true;
                        endPos = 1000;
                        vTarget = 0.5;
                        powerSetPoint = 0.5;
                        resetIntakeArmVars();
                    }
                }
                else if (opMode.gamepad2.y && !isLiftingIntakeArm) {
                    robot.dumperRotateServo.setPosition(robot.DUMPER_RECEIVE_SERVO);
                    if(!isRunningIntakeArmPID){
                        isRunningIntakeArmPID = true;
                        intakeArmDown = false;
                        endPos = 0;
                        vTarget = -0.7;
                        powerSetPoint = -0.7;
                        resetIntakeArmVars();
                    }
                }
                if (isRunningIntakeArmPID && intakeArmDown){
                     if(inPos < endPos) {
                        inPos = robot.intakeArmMotor.getCurrentPosition();
                        double time = intakeArmRunTime.milliseconds();
                        double dT = time - lastTime;
                        double vEnc = (inPos - lastPos) / dT;
                        double err = vEnc - vTarget;
                        sumErr = 0.67 * sumErr + err * dT;
                        dErrDT = (err - lastErr) / dT;
                        pid = Kp * err + Ki * sumErr + Kd * dErrDT;
                        double powerApp = powerSetPoint - pid;
                        powerApp = Range.clip(powerApp, 0.0, 1.0);
                        robot.intakeArmMotor.setPower(powerApp);
                        lastErr = err;
                        lastTime = time;
                        lastPos = inPos;
                        //Log.v("BOK", "Intake arm posD " + inPos + " moving at " + powerApp);
                    }
                    else {
                        isRunningIntakeArmPID = false;
                        currentIntakeArmPosition = 1100;
                    }
                }
                else if (isRunningIntakeArmPID && !intakeArmDown) {
                    if (inPos > endPos) {
                        inPos = robot.intakeArmMotor.getCurrentPosition();
                        double time = intakeArmRunTime.milliseconds();
                        double dT = time - lastTime;
                        double vEnc = (inPos - lastPos) / dT;
                        double err = vEnc - vTarget;
                        sumErr = 0.67 * sumErr + err * dT;
                        dErrDT = (err - lastErr) / dT;
                        pid = Kp * err + Ki * sumErr + Kd * dErrDT;
                        double powerApp = powerSetPoint - pid;
                        powerApp = Range.clip(powerApp, -1.0, 0.0);
                        robot.intakeArmMotor.setPower(powerApp);
                        lastErr = err;
                        lastTime = time;
                        lastPos = inPos;
                        //Log.v("BOK", "Intake arm posU " + inPos + " moving at " + powerApp);
                    } else {
                        isRunningIntakeArmPID = false;
                        currentIntakeArmPosition = -100;
                    }
                }
                else {
                    if (!isLiftingIntakeArm && hasMovedIntakeArm) {
                        robot.intakeArmMotor.setTargetPosition(currentIntakeArmPosition);
                        robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.intakeArmMotor.setPower(0.5);
                    }
                }

                if (isLiftingIntakeArm){
                    //Log.v("BOK", "Setting -ve power to inA");
                    robot.intakeArmMotor.setPower(-0.6);
                }

                if ((opMode.gamepad2.left_trigger > GAME_TRIGGER_DEAD_ZONE)
                        && !isLiftingIntakeArm) {
                    isLiftingIntakeArm = true;
                    Log.v("BOK", "Tele: Started lifting Intake ARM");
                }

                if ((opMode.gamepad2.right_trigger > GAME_TRIGGER_DEAD_ZONE)
                        && isLiftingIntakeArm) {
                    hasMovedIntakeArm = true;
                    isLiftingIntakeArm = false;
                    robot.intakeArmMotor.setPower(0);
                    robot.intakeArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Log.v("BOK", "Tele: Initialized Intake ARM");
                }
            } // !end_game
            else {
                // End game
                // Raise or lower the hook
                if (opMode.gamepad2.dpad_up && !hangHookRaised) {
                    robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_INIT);
                    hangHookRaised = true;
                }
                if (opMode.gamepad2.dpad_down && hangHookRaised) {
                    robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_FINAL);
                    hangHookRaised = false;
                }
                // Hanging lift control
                if (opMode.gamepad2.left_stick_y < -GAME_TRIGGER_DEAD_ZONE) {
                    // Raise the dumper lift
                    //robot.hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.hangMotor.setPower(HANG_LIFT_POWER);
                    //Log.v("BOK", "Hanging Lift Pos UP " +
                    //        robot.hangMotor.getCurrentPosition());
                } else if (opMode.gamepad2.left_stick_y > GAME_TRIGGER_DEAD_ZONE) {
                    // Lower the lift
                    //robot.hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.hangMotor.setPower(-HANG_LIFT_POWER);
                    //Log.v("BOK", "Hanging Lift Pos DN " +
                    //        robot.hangMotor.getCurrentPosition());
                } else {
                    robot.hangMotor.setPower(0);
                }
            }
            robot.waitForTick(robot.WAIT_PERIOD);
        }

 */

        opMode.telemetry.update();
        }
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    private void moveRobot() {
        robot.moveRobotTele(speedCoef);
    }

    private void moveIntake()
    {
        /*
         * Gamepad2: Driver 2 controls the intake servo using the left joystick for speed
         */
        // NOTE: the left joystick goes negative when pushed upwards
        double gamePad2LeftStickY = -opMode.gamepad2.left_stick_y*INTAKE_MOTOR_CAP_SPEED;
        //robot.intakeMotor.setPower(gamePad2LeftStickY);
    }
/*
    private void resetIntakeArmVars(){
        lastErr = 0; sumErr = 0; dErrDT = 0; pid = 0; lastTime = 0;
        intakeArmRunTime.reset();
        inPos = robot.intakeArmMotor.getCurrentPosition();
        lastPos = inPos;
        robot.intakeArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

 */
}
