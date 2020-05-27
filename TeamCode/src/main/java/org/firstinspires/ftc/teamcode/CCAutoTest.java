package org.firstinspires.ftc.teamcode;


public class CCAutoTest extends CCAutoCommon {
    @Override
    public void runSoftware() {
        if (opMode.opModeIsActive()) {
            // Test sensor servo & sensor
            opMode.telemetry.addData("Test: ", "Distance sensor");
            opMode.telemetry.update();
            // robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_FINAL);
            opMode.sleep(500);

            while (opMode.opModeIsActive() && !opMode.gamepad1.x) {
                robot.autoROTRight.setPosition(robot.AUTO_RIGHT_DEPLOY);
                robot.autoGripRight.setPosition(robot.AUTO_GRAB);
               // robot.autoTopServo.setPosition(robot.AUTO_TOP_INIT);
                //robot.autoBotServo.setPosition(0.6);
                double distLB = robot.getDistanceCM(robot.distanceLeft, 250, 2, opMode);
               // robot.distanceLeftBack.close();
                double distLF = robot.getDistanceCM(robot.distanceForward, 250, 2, opMode);
              //  robot.distanceLeftForward.close();
                double distRB = robot.getDistanceCM(robot.distanceBack, 250 , 2, opMode);
                double distRF = robot.getDistanceCM(robot.distanceRight, 250, 2 , opMode);
                double distB = robot.opticalDistanceSensor.getLightDetected();
                opMode.telemetry.addData("Distance Left: ", distLB);
                opMode.telemetry.addData("Distance For: ", distLF);
                opMode.telemetry.addData("Distance Back: ", distRB);
                opMode.telemetry.addData("Distance Right: ", distRF);
                opMode.telemetry.addData("ODS: ", distB);
                opMode.telemetry.update();
            }
            opMode.sleep(500);

            // robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_INIT);
            opMode.sleep(1000);

        }

    }
/*
    // Constructor
    public CCAutoTest()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware()
    {
        boolean[] arrayTests = {
                false, // 4 DT motors
                true, // Dumper lift motor & servo
                true, // Intake arm motor & intake motor
                true, // Hanging lift motor & servo
                true, // flicker servo
                true, // marker servo
                true, // distance sensor & servo
                false,  // autonomous test
                false}; // take picture


        if (arrayTests[0]) {
            // move (test the 4 DC motors
            opMode.telemetry.addData("Test: ", "Moving forward");
            opMode.telemetry.update();
            robot.testDTMotors();//move(0.5, 0.5, 12, true, 6);
            opMode.telemetry.addData("Test: ", "Moving forward complete");
            opMode.telemetry.update();
            opMode.sleep(2000);
        } // if (arrayTests[0])

        while (opMode.opModeIsActive() && arrayTests[1] && !opMode.gamepad1.a) {
        }

        if (arrayTests[1] && opMode.opModeIsActive()) {
            // test Dumper lift motor
            opMode.telemetry.addData("Test: ", "Moving dumper lift");
            opMode.telemetry.update();
            robot.dumperSlideMotor.setTargetPosition(robot.DUMPER_SLIDE_FINAL_POS);
            robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.dumperSlideMotor.setPower(0.9);
            opMode.sleep(2000);

            boolean dump = false;
            double nextPos = robot.DUMPER_ROTATE_SERVO_INIT - 0.05;
            while (opMode.opModeIsActive() && !opMode.gamepad1.x) {
                //Log.v("BOK", "Dumper slide: " + robot.dumperSlideMotor.getCurrentPosition());
                if (opMode.gamepad1.y && !dump) {
                    dump = opMode.gamepad1.y;
                    //robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_FINAL);
                }
                if (dump && (nextPos > robot.DUMPER_ROTATE_SERVO_FINAL)) {
                    robot.dumperRotateServo.setPosition(nextPos);
                    nextPos -= 0.02;
                }
                if (opMode.gamepad1.b && dump) {
                    dump = false;
                    nextPos = robot.DUMPER_ROTATE_SERVO_INIT;
                    robot.dumperRotateServo.setPosition(robot.DUMPER_ROTATE_SERVO_INIT);
                }
            }

            robot.dumperSlideMotor.setTargetPosition(0);
            robot.dumperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.dumperSlideMotor.setPower(0.9);

            //Log.v("BOK", "Dumper slide done: " + robot.dumperSlideMotor.getCurrentPosition());
            opMode.telemetry.addData("Test: ", "Moving dumper lift complete");
            opMode.telemetry.update();
            opMode.sleep(2000);
        } // if (arrayTests[1])


        while (opMode.opModeIsActive() && arrayTests[2] && !opMode.gamepad1.a) {
        }

        if (arrayTests[2] && opMode.opModeIsActive()) {
            // Test intake arm motor and intake motor
            opMode.telemetry.addData("Test: ", "Intake arm motor");
            opMode.telemetry.update();
            dropIntakeArmAndExtend();
            robot.intakeMotor.setPower(0.7);
            opMode.sleep(2000);
            robot.intakeMotor.setPower(0);
            opMode.telemetry.addData("Test: ", "Intake arm complete");
            opMode.telemetry.update();
            opMode.sleep(2000);
        }

        while (opMode.opModeIsActive() && arrayTests[3] && !opMode.gamepad1.a) {
        }

        if (arrayTests[3] && opMode.opModeIsActive()) {
            // Test hanging lift and hang hook servo
            opMode.telemetry.addData("Test: ", "Intake arm motor");
            opMode.telemetry.update();

            robot.hangMotor.setTargetPosition(robot.HANG_LIFT_HIGH_POS);
            robot.hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hangMotor.setPower(0.5);
            while (opMode.opModeIsActive() && robot.hangMotor.isBusy()) {

            }
            robot.hangMotor.setPower(0);
            robot.hangHookServo.setPosition(robot.HANG_HOOK_SERVO_FINAL);
            opMode.telemetry.addData("Test: ", "Moving hanging complete");
            opMode.telemetry.update();
            opMode.sleep(1000);
        }

        while (opMode.opModeIsActive() && arrayTests[4] && !opMode.gamepad1.a) {
        }

        if (arrayTests[4] && opMode.opModeIsActive()) {
            // Test flicker servo
            opMode.telemetry.addData("Test: ", "Flicker servo");
            opMode.telemetry.update();
            robot.samplerServo.setPosition(robot.SAMPLER_SERVO_FINAL);
            opMode.sleep(500);
            robot.samplerServo.setPosition(robot.SAMPLER_SERVO_INIT);
            opMode.sleep(1000);
        }

        while (opMode.opModeIsActive() && arrayTests[5] && !opMode.gamepad1.a) {
        }

        if (arrayTests[5] && opMode.opModeIsActive()) {
            // Test marker servo
            opMode.telemetry.addData("Test: ", "Marker servo");
            opMode.telemetry.update();
            dumpMarker();
            opMode.sleep(1000);
        }

        while (opMode.opModeIsActive() && arrayTests[6] && !opMode.gamepad1.a) {
        }

        if (arrayTests[6] && opMode.opModeIsActive()) {
            // Test sensor servo & sensor
            opMode.telemetry.addData("Test: ", "Distance sensor");
            opMode.telemetry.update();
            robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_FINAL);
            opMode.sleep(500);

            while (opMode.opModeIsActive() && !opMode.gamepad1.x) {
                double dist = robot.getDistanceCM(robot.distanceBack, 150, 2);
                opMode.telemetry.addData("Distance: ", dist);
                opMode.telemetry.update();
            }
            opMode.sleep(500);

            robot.distanceRotateServo.setPosition(robot.DISTANCE_ROTATE_SERVO_INIT);
            opMode.sleep(1000);

        }
        opMode.telemetry.addData("Test: ", "All tests complete");
        opMode.telemetry.update();

        while (opMode.opModeIsActive() && arrayTests[7] && !opMode.gamepad1.a) {
        }

        if (arrayTests[7] && opMode.opModeIsActive()) {
            // Autonomous tests
            //robot.moveIntakeArmPID(400, 0.25, 0.2, 10);
            //gyroTurn(DT_TURN_SPEED_LOW, 0, 45, DT_TURN_THRESHOLD_LOW, false, false, 4);

            //strafe(0.5, 3, false, 6);
            //dropIntakeArmAndExtend();
            //followHeadingPIDBack(0, -0.3, 30, false, 6);
            followHeadingPID(0, 0.5, 45, true, 5);
        }

        while (opMode.opModeIsActive() && arrayTests[8] && !opMode.gamepad1.a) {
        }

        if (arrayTests[8] && opMode.opModeIsActive()) {

            // CameraDevice.getInstance().setFlashTorchMode(true);

            int numPics = 0;
            boolean x = false;
            while (opMode.opModeIsActive()) {
                if (opMode.gamepad1.x && !x) {
                    takePicture("SamplingImage_" + numPics + ".png");
                    numPics++;

                }
                x = opMode.gamepad1.x;
            }
        } // arrayTests[8]
    }

 */
}
