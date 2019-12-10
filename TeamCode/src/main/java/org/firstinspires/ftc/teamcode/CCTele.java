package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


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
    private boolean isLiftingIntakeArm = false;
    private boolean hasMovedIntakeArm = false;

    public BoKTeleStatus initSoftware(LinearOpMode opMode,
                                      CCHardwareBot robot) {
        this.opMode = opMode;
        this.robot = robot;
        robot.setModeForDTMotors(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return BoKTeleStatus.BOK_TELE_SUCCESS;
    }

    public BoKTeleStatus runSoftware() {

        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // run until the end of the match (driver presses STOP)
        while (opMode.opModeIsActive()) {
            // GAMEPAD 1 CONTROLS:
            // Left & Right stick: Drive
            // A:                  Go in fast mode
            // Y:                  Go in slow mode

            moveRobot();

            if (opMode.gamepad1.y) {
                speedCoef = CCHardwareBot.SPEED_COEFF_SLOW;
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
            if (robot.liftMotor.getCurrentPosition() < 5) {
                closeGamepad = true;
            }
            if (!robot.liftMotor.isBusy() && !isLiftingIntakeArm && !hasMovedIntakeArm) {
                robot.liftMotor.setTargetPosition(robot.liftMotor.getCurrentPosition());
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                isLiftingIntakeArm = false;
                robot.liftMotor.setPower(0.4);
            }
            if (-opMode.gamepad2.left_stick_y > GAME_TRIGGER_DEAD_ZONE) {
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(-opMode.gamepad2.left_stick_y);
                isLiftingIntakeArm = true;
                hasMovedIntakeArm = true;
                closeGamepad = false;
            }
            if ((-opMode.gamepad2.left_stick_y < GAME_TRIGGER_DEAD_ZONE) && !closeGamepad) {
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.liftMotor.setPower(-opMode.gamepad2.left_stick_y * 0.6);
                hasMovedIntakeArm = true;
                isLiftingIntakeArm = false;
            }
            if (opMode.gamepad2.left_stick_y == 0 && !isLiftingIntakeArm) {
                robot.liftMotor.setPower(0);
            }
            if (opMode.gamepad2.a) {
                robot.intakeServo.setPosition(robot.INTAKE_GRAB_POS);
            }
            if (opMode.gamepad2.b) {
                robot.intakeServo.setPosition(robot.INTAKE_RELEASE_POS);
            }

            if (opMode.gamepad2.dpad_up) {
                robot.inRotateServo.setPosition(robot.ROTATE_UP_POS);
            }
            if (opMode.gamepad2.dpad_down) {
                robot.inRotateServo.setPosition(robot.ROTATE_DOWN_POS);
            }
            if (opMode.gamepad2.x) {
                robot.foundationGripServo.setPosition(robot.FOUNDATION_GRIP_DOWN);
            }
            if (opMode.gamepad2.y) {
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
