package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    private boolean servoGate = true;
    private boolean resetLift = false;
    private boolean armDone = false;
    int counter = 0;
    boolean next = false;


    public BoKTeleStatus initSoftware(LinearOpMode opMode,
                                      CCHardwareBot robot) {
        this.opMode = opMode;
        this.robot = robot;
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.liftLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.liftLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.capStoneServo.setPosition(robot.CAP_HOLD);
        robot.foundationGripServo.setPosition(robot.FOUNDATION_GRIP_DOWN);
        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    public BoKTeleStatus runSoftware() {

        robot.autoROTRight.setPosition(robot.AUTO_RIGHTI_IN + 0.08);
        robot.autoGripRight.setPosition(robot.AUTO_GRAB);
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
            if(opMode.gamepad1.right_bumper){
                robot.capStoneServo.setPosition(robot.CAP_DROP);
            }
            if(opMode.gamepad1.right_trigger > GAME_TRIGGER_DEAD_ZONE){
                robot.capStoneServo.setPosition(robot.CAP_HOLD);
            }

            // GAMEPAD 2 CONTROLS
            //Left Stick:       Control Lift
            //A:                Grab with the claw
            //B:                Release claw
            //X:                Move foundation grip down
            //Y:                Move foundation grip up

            if (opMode.gamepad2.a){
                robot.intakeRightMotor.setPower(-robot.INTAKE_POWER);
                robot.intakeLeftMotor.setPower(robot.INTAKE_POWER);
            }
            if(opMode.gamepad2.b){
                robot.intakeRightMotor.setPower(0);
                robot.intakeLeftMotor.setPower(0);
            }
            if (robot.liftLeftMotor.getCurrentPosition() >=50 &&!isLiftingIntakeArm) {
                resetLift = true;
                closeGamepad = true;
            }
            if(resetLift){
                robot.liftLeftMotor.setTargetPosition(0);
                robot.liftRightMotor.setTargetPosition(0);
                robot.liftLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftLeftMotor.setPower(0.3);
                robot.liftRightMotor.setPower(0.3);
                if(!robot.liftLeftMotor.isBusy() && !robot.liftRightMotor.isBusy()) {
                    resetLift = false;
                }
            }
            if(opMode.gamepad2.left_stick_y == 0 && (!robot.liftRightMotor.isBusy())&&(!robot.liftLeftMotor.isBusy())){
                robot.liftLeftMotor.setPower(0);
                robot.liftRightMotor.setPower(0);
            }
            Log.v("BOK", "RIGHT ENC: " + robot.liftRightMotor.getCurrentPosition());
            Log.v("BOK", "LEFT ENC: " + robot.liftLeftMotor.getCurrentPosition());

            if (-opMode.gamepad2.left_stick_y > GAME_TRIGGER_DEAD_ZONE) {
                robot.liftLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.liftLeftMotor.setPower(-1);
                robot.liftRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.liftRightMotor.setPower(-1);
                isLiftingIntakeArm = true;
                hasMovedIntakeArm = true;
                closeGamepad = false;
            }
            if ((-opMode.gamepad2.left_stick_y < -GAME_TRIGGER_DEAD_ZONE) && !closeGamepad ){

                robot.liftLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftLeftMotor.setPower(0.1);
                robot.liftRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftRightMotor.setPower(0.1);
                hasMovedIntakeArm = true;
                isLiftingIntakeArm = false;

            }
            if(opMode.gamepad2.dpad_up){
                armDone = false;
                intGripServo = false;
                intServo = true;
                robot.flickerServo.setPosition(robot.FLICKER_INIT);
                //robot.gripperRotateLeftServo.setPosition(robot.ROTATE_UP_POS_LEFT);
               // robot.gripperRotateRightServo.setPosition(robot.ROTATE_UP_POS);
                robot.gripperServo.setPosition(robot.INTAKE_GRAB_POS);
            }
            if(intServo){
                double gripPos = robot.gripperServo.getPosition();
                double rightGrip = robot.gripperRotateRightServo.getPosition();
                double leftGrip = robot.gripperRotateLeftServo.getPosition();
                double flick = robot.flickerServo.getPosition();


                if(rightGrip >= robot.ROTATE_UP_POS){
                    robot.gripperServo.setPosition(robot.INTAKE_GRAB_POS);
                    robot.gripperRotateRightServo.setPosition(robot.ROTATE_UP_POS);
                    robot.gripperRotateLeftServo.setPosition(robot.ROTATE_UP_POS_LEFT);
                    intGripServo = false;
                    intServo = false;
                }

                if((leftGrip >= robot.ROTATE_UP_POS_LEFT)) {
                     robot.gripperRotateLeftServo.setPosition(leftGrip - 0.04);
                }
                if((rightGrip <= robot.ROTATE_UP_POS)) {
                    robot.gripperRotateRightServo.setPosition(rightGrip + 0.04);
                }

            }
            if(opMode.gamepad2.dpad_down) {
                intGripServo = false;
                servoGate = true;

                robot.gripperRotateRightServo.setPosition(robot.ROTATE_DOWN_POS);
                robot.gripperRotateLeftServo.setPosition(robot.ROTATE_DOWN_LEFT_POS);
               // robot.gripperOrientationServo.setPosition(robot.ORI_MID);
                robot.gripperServo.setPosition(robot.INTAKE_GRAB_POS+0.05);
                robot.flickerServo.setPosition(robot.FLICKER_INIT);
                counter = 0;

            }
            if(robot.opticalDistanceSensor.getRawLightDetected() > 800 && servoGate){
                counter++;
            }

            if(robot.opticalDistanceSensor.getRawLightDetected() > 800 && counter >= 7){

                //robot.gripperOrientationServo.setPosition(robot.ORI_DOWN);
                robot.gripperServo.setPosition(robot.INTAKE_RELEASE_POS);
                robot.intakeRightMotor.setPower(0);
                robot.flickerServo.setPosition(robot.FLICKER_SET);
                robot.intakeLeftMotor.setPower(0);
                servoGate = false;
                counter = 0;
            }
            if(opMode.gamepad2.x){
                resetLift = true;
            }
            if(opMode.gamepad2.right_bumper){
                counter = 0;
                armDone = true;
                robot.gripperRotateRightServo.setPosition(robot.ROTATE_GRIP_POS);
                robot.gripperRotateLeftServo.setPosition(robot.ROTATE_GRIP_LEFT_POS);
               // robot.gripperOrientationServo.setPosition(robot.ORI_DOWN);
               // robot.gripperServo.setPosition(robot.INTAKE_GRAB_POS);
                robot.intakeRightMotor.setPower(0);
                robot.flickerServo.setPosition(robot.FLICKER_SET);
                robot.intakeLeftMotor.setPower(0);
                servoGate = false;
            }
            if(armDone){
                double rightGrip = robot.gripperRotateRightServo.getPosition();
                if(rightGrip <= robot.ROTATE_GRIP_POS){
                    intGripServo = true;
                    armDone = false;
                }

            }
            Log.v("BOK", "ODS: " + robot.opticalDistanceSensor.getLightDetected());

            if(intGripServo){
                double gripPos = robot.gripperServo.getPosition();
                double rightGrip = robot.gripperRotateRightServo.getPosition();
                double leftGrip = robot.gripperRotateLeftServo.getPosition();
                double flick = robot.flickerServo.getPosition();


                if(gripPos <= robot.INTAKE_GRAB_POS){
                    robot.gripperServo.setPosition(robot.INTAKE_GRAB_POS);
                    robot.flickerServo.setPosition(robot.FLICKER_INIT);
                    robot.flickerServo.setPosition(robot.FLICKER_INIT);
                    intGripServo = false;
                }

                if(!(leftGrip <= robot.ROTATE_DOWN_LEFT_POS)) {
                    // robot.gripperRotateLeftServo.setPosition(leftGrip - 0.1);
                }
                if(!(rightGrip >= robot.ROTATE_DOWN_POS)) {
                    //robot.gripperRotateRightServo.setPosition(rightGrip + 0.1);
                }
                if(!(flick <= robot.FLICKER_INIT+0.1)){
                    robot.flickerServo.setPosition(flick - 0.06);
                }
                if(!(gripPos <= robot.INTAKE_GRAB_POS)) {
                    Log.v("BOK", "GRIP: " + gripPos);
                    robot.gripperServo.setPosition(gripPos - 0.01);
                }


            }
            Log.v("BOK", "Sevo Pos Left: " + robot.gripperRotateLeftServo.getPosition() + " Servo Pos RightL " + robot.gripperRotateRightServo.getPosition());

            if(opMode.gamepad2.left_bumper){

                servoGate = true;
                robot.gripperServo.setPosition(robot.INTAKE_RELEASE_POS);
            }
            if(opMode.gamepad2.y){
                intGripServo = false;
                servoGate = true;

                robot.gripperRotateRightServo.setPosition(robot.ROTATE_DOWN_POS);
                robot.gripperRotateLeftServo.setPosition(robot.ROTATE_DOWN_LEFT_POS);
                // robot.gripperOrientationServo.setPosition(robot.ORI_MID);
                robot.gripperServo.setPosition(robot.INTAKE_GRAB_POS+0.05);
                robot.flickerServo.setPosition(robot.FLICKER_INIT);
                counter = 0;
                intGripServo = false;
                robot.flickerServo.setPosition(robot.FLICKER_INIT);
                counter = 0;
                robot.intakeLeftMotor.setPower(-0.6);
                robot.intakeRightMotor.setPower(0.6);
            }
            if(-opMode.gamepad2.right_stick_y > GAME_TRIGGER_DEAD_ZONE){
                Log.v("BOK", "G2 right stick: " + -opMode.gamepad2.right_stick_y);

                double currentRight = robot.gripperRotateRightServo.getPosition() - 0.01;
                double currentLeft = robot.gripperRotateLeftServo.getPosition() + 0.01;
             //   robot.gripperRotateRightServo.setPosition(currentRight);
             //   robot.gripperRotateLeftServo.setPosition(currentLeft);

            }
            if(opMode.gamepad2.right_trigger > GAME_TRIGGER_DEAD_ZONE){
                robot.foundationGripServo.setPosition(robot.FOUNDATION_GRIP_DOWN);
            }
            if(opMode.gamepad2.left_trigger > GAME_TRIGGER_DEAD_ZONE){
                robot.foundationGripServo.setPosition(robot.FOUNDATION_GRIP_UP);
            }
            if(opMode.gamepad2.dpad_left){
                intGripServo = false;

                robot.flickerServo.setPosition(robot.FLICKER_INIT);
            }
            Log.v("BOK", "Right: " + robot.gripperRotateRightServo.getPosition() + ", Left: " + robot.gripperRotateLeftServo.getPosition());
            if(opMode.gamepad2.dpad_right){
                robot.gripperServo.setPosition(robot.INTAKE_GRAB_POS);
            }
            opMode.telemetry.addData("ODS: ", robot.opticalDistanceSensor.getRawLightDetected());

            if(opMode.gamepad2.right_stick_button){
                robot.gripperRotateRightServo.setPosition(robot.ROTATE_MID_POS);
                robot.gripperRotateLeftServo.setPosition(robot.ROTATE_MID_LEFT_POS);
                counter = 0;
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
