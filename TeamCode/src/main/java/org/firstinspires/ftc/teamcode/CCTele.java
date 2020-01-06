package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


public class CCTele {

    // CONSTANTS
    //private static final double GAME_STICK_DEAD_ZONE_LEFT_STICK = 0.3;
    private static final double GAME_TRIGGER_DEAD_ZONE = 0.2;
    private static final double INTAKE_MOTOR_CAP_SPEED = 0.9;
    private static final double Kp = 0.7, Ki = 0.525, Kd = 0.2;
    boolean closeGamepad = false;
    private CCHardwareBot robot;
    private LinearOpMode opMode;
    private double speedCoef = CCHardwareBot.SPEED_COEFF_FAST;
    private boolean end_game = false;
    private boolean isLiftingIntakeArm = true;
    private boolean hasMovedIntakeArm = false;
    private boolean intServo = false;
    private boolean intGripServo = false;
    private boolean moveUp = false;
    private boolean servoReady = false;
    private boolean servoDown = false;
    private boolean servoPlace = false;
    private boolean resetLift = false;
    int counter = 0;

    public BoKTeleStatus initSoftware(LinearOpMode opMode,
                                      CCHardwareBot robot) {
        this.opMode = opMode;
        this.robot = robot;
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.liftLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    public BoKTeleStatus runSoftware() {
        //robot.intakeServo.setPosition(robot.INTAKE_GRAB_POS);
        //robot.inRotateServo.setPosition(robot.ROTATE_DOWN_POS);
        //robot.foundationGripServo.setPosition(robot.FOUNDATION_GRIP_DOWN);
      //  robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            // GAMEPAD 1 CONTROLS:
            // Left & Right stick: Drive
            // A:                  Go in fast mode
            // Y:                  Go in slow mode

            moveRobot();

            if (opMode.gamepad1.y) {
                speedCoef = CCHardwareBot.SPEED_COEFF_SLOW;
                robot.flickerServo.setPosition(robot.FLICKER_INIT);
                counter = 0;
            }
            if (opMode.gamepad1.a) {
                speedCoef = CCHardwareBot.SPEED_COEFF_FAST;
            }

            // GAMEPAD 2 CONTROLS
            //Left Stick:       Control Lift
            //A:                Grab with the claw
            //B:                Release claw
            //X:                Move foundation grip down
            //Y:                Move foundation grip up

            if (opMode.gamepad2.a){
                robot.intakeRightMotor.setPower(robot.INTAKE_POWER);
                robot.intakeLeftMotor.setPower(-robot.INTAKE_POWER);


            }

            if(opMode.gamepad2.b){
                robot.intakeRightMotor.setPower(0);
                robot.intakeLeftMotor.setPower(0);
            }


            if (robot.liftLeftMotor.getCurrentPosition() <= 50&&!isLiftingIntakeArm) {
                resetLift = true;
                closeGamepad = true;
            }
            if(resetLift){
                robot.liftLeftMotor.setTargetPosition(0);
                robot.liftRightMotor.setTargetPosition(0);
                robot.liftLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftLeftMotor.setPower(-0.4);
                robot.liftRightMotor.setPower(-0.4);
                if(!robot.liftLeftMotor.isBusy() && !robot.liftRightMotor.isBusy()) {
                    resetLift = false;
                }
            }
            Log.v("BOK", "RIGHT ENC: " + robot.liftRightMotor.getCurrentPosition());
            Log.v("BOK", "LEFT ENC: " + robot.liftLeftMotor.getCurrentPosition());
            if(opMode.gamepad2.left_stick_y == 0 && (!robot.liftRightMotor.isBusy())&&(!robot.liftLeftMotor.isBusy())){
                robot.liftLeftMotor.setPower(0);
                robot.liftRightMotor.setPower(0);
            }

            if (-opMode.gamepad2.left_stick_y > GAME_TRIGGER_DEAD_ZONE) {

                robot.liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftLeftMotor.setPower(-opMode.gamepad2.left_stick_y);
                robot.liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftRightMotor.setPower(-opMode.gamepad2.left_stick_y);


                isLiftingIntakeArm = true;
                hasMovedIntakeArm = true;
                closeGamepad = false;
                Log.v("BOK", "UP");
            }
            if ((-opMode.gamepad2.left_stick_y < GAME_TRIGGER_DEAD_ZONE) && !closeGamepad ){

                robot.liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftLeftMotor.setPower(-opMode.gamepad2.left_stick_y * 0.2);
                robot.liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftRightMotor.setPower(-opMode.gamepad2.left_stick_y * 0.2);


                hasMovedIntakeArm = true;
                isLiftingIntakeArm = false;

            }
           // Log.v("BOK", "Gamepad: " + opMode.gamepad2.left_stick_y);
            if(opMode.gamepad2.dpad_up){
               // robot.gripperRotateLeftServo.setDirection(Servo.Direction.REVERSE);
                //robot.gripperOrientationServo.setPosition(0.3);
                robot.gripperRotateRightServo.setPosition(0.7);
                robot.gripperRotateLeftServo.setPosition(0.3);
                robot.gripperServo.setPosition(robot.INTAKE_GRAB_POS);
                intServo = true;


            }
            if(intServo){
                double rightGrip = robot.gripperRotateRightServo.getPosition();
                double leftGrip = robot.gripperRotateLeftServo.getPosition();
                double oriPos = robot.gripperOrientationServo.getPosition();
                if(rightGrip <= robot.ROTATE_UP_POS && leftGrip >= robot.ROTATE_UP_POS_LEFT && oriPos >= robot.ORI_UP){
                    robot.gripperOrientationServo.setPosition(robot.ORI_UP);
                    robot.gripperRotateLeftServo.setPosition(robot.ROTATE_UP_POS_LEFT);
                    robot.gripperRotateRightServo.setPosition(robot.ROTATE_UP_POS);
                    intServo = false;
                }
                if(!(rightGrip <= robot.ROTATE_UP_POS)) {
                    robot.gripperRotateRightServo.setPosition(rightGrip - 0.04);
                }
                if (!(leftGrip >= robot.ROTATE_UP_POS_LEFT)) {
                    robot.gripperRotateLeftServo.setPosition(leftGrip + 0.04);
                }
                if(!(oriPos >= robot.ORI_UP)) {
                    robot.gripperOrientationServo.setPosition(oriPos + 0.07);
                }

            }
            if(opMode.gamepad2.dpad_down) {

                robot.gripperRotateRightServo.setPosition(robot.ROTATE_DOWN_POS - 0.2);
                robot.gripperRotateLeftServo.setPosition(robot.ROTATE_DOWN_LEFT_POS + 0.2);
                robot.gripperOrientationServo.setPosition(robot.ORI_MID);
                robot.gripperServo.setPosition(robot.INTAKE_RELEASE_POS);
                robot.flickerServo.setPosition(robot.FLICKER_INIT);

            }
            if(robot.opticalDistanceSensor.getLightDetected() > 0.20){
                counter++;

            }
            if(robot.opticalDistanceSensor.getLightDetected() > 0.2 && counter > 6){
                counter = 0;
                robot.flickerServo.setPosition(robot.FLICKER_SET);
            }
            Log.v("BOK", "ODS: " + robot.opticalDistanceSensor.getLightDetected());
            if(opMode.gamepad2.x){
                resetLift = true;
            }
            if(opMode.gamepad2.right_bumper){
                intGripServo = true;
                robot.gripperRotateRightServo.setPosition(robot.ROTATE_DOWN_POS);
                robot.gripperRotateLeftServo.setPosition(robot.ROTATE_DOWN_LEFT_POS);
                robot.gripperOrientationServo.setPosition(robot.ORI_DOWN);
                //robot.gripperServo.setPosition(robot.INTAKE_GRAB_POS);
                robot.intakeRightMotor.setPower(0);
                robot.intakeLeftMotor.setPower(0);
            }
            if(intGripServo){
                double gripPos = robot.gripperServo.getPosition();
                double rightGrip = robot.gripperRotateRightServo.getPosition();
                double leftGrip = robot.gripperRotateLeftServo.getPosition();
                double oriPos = robot.gripperOrientationServo.getPosition();
                if(gripPos >= robot.INTAKE_GRAB_POS){
                    robot.gripperRotateRightServo.setPosition(robot.ROTATE_DOWN_POS);
                    robot.gripperRotateLeftServo.setPosition(robot.ROTATE_DOWN_LEFT_POS);
                   robot.gripperOrientationServo.setPosition(robot.ORI_DOWN);
                    robot.intakeRightMotor.setPower(0);
                    robot.intakeLeftMotor.setPower(0);
                    intGripServo = false;
                }

                  if(!(leftGrip <= robot.ROTATE_DOWN_LEFT_POS)) {
                     // robot.gripperRotateLeftServo.setPosition(leftGrip - 0.1);
                    }
                    if(!(rightGrip >= robot.ROTATE_DOWN_POS)) {
                        //robot.gripperRotateRightServo.setPosition(rightGrip + 0.1);
                    }
                    if(!(gripPos >= robot.INTAKE_GRAB_POS)) {
                       robot.gripperServo.setPosition(gripPos + 0.06);
                    }
                    if(!(oriPos <= robot.ORI_DOWN)) {
                      // robot.gripperOrientationServo.setPosition(oriPos - 0.05);
                    }

            }
            if(opMode.gamepad2.left_bumper){
                robot.gripperServo.setPosition(robot.INTAKE_RELEASE_POS);
            }
            if(opMode.gamepad2.y){
                robot.intakeLeftMotor.setPower(robot.INTAKE_POWER);
                robot.intakeRightMotor.setPower(-robot.INTAKE_POWER);
            }
            if(-opMode.gamepad2.right_stick_y > GAME_TRIGGER_DEAD_ZONE){
                double currentRight = robot.gripperRotateRightServo.getPosition();
                double currentLeft = robot.gripperRotateLeftServo.getPosition();
                double currentOrientation = robot.gripperOrientationServo.getPosition();
                //robot.gripperRotateRightServo.setPosition(currentRight - 0.03);
                //robot.gripperRotateLeftServo.setPosition(currentLeft + 0.03);
                robot.gripperOrientationServo.setPosition(currentOrientation + 0.02);
            }

            if(-opMode.gamepad2.right_stick_y < -GAME_TRIGGER_DEAD_ZONE){
                Log.v("BOK", "G2 right stick: " + -opMode.gamepad2.right_stick_y);
                double currentRight = robot.gripperRotateRightServo.getPosition();
                double currentLeft = robot.gripperRotateLeftServo.getPosition();
                double currentOrientation = robot.gripperOrientationServo.getPosition();
                //robot.gripperRotateRightServo.setPosition(currentRight + 0.03);
                //robot.gripperRotateLeftServo.setPosition(currentLeft - 0.03);
                robot.gripperOrientationServo.setPosition(currentOrientation - 0.01);
            }
            if(opMode.gamepad2.right_trigger > GAME_TRIGGER_DEAD_ZONE){
                robot.foundationGripServo.setPosition(robot.FOUNDATION_GRIP_DOWN);
            }
            if(opMode.gamepad2.left_trigger > GAME_TRIGGER_DEAD_ZONE){
                robot.foundationGripServo.setPosition(robot.FOUNDATION_GRIP_UP);
            }




            opMode.telemetry.update();
        }
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    private void moveRobot() {
        robot.moveRobotTele(speedCoef);
    }

    public enum BoKTeleStatus {
        BOK_TELE_FAILURE,
        BOK_TELE_SUCCESS
    }
}
