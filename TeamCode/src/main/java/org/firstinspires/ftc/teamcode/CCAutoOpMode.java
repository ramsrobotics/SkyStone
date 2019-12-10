package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class CCAutoOpMode extends LinearOpMode {
    protected CCAuto autoImpl; // use interface (polymorphism)


    @Override
    public void runOpMode() throws InterruptedException {
        CCHardwareBot robot = new CCMecanumDT();

        boolean status = false;


        /*
         * Initialize the drive train and the robot subsystems variables.
         * The initHardware() method of the hardware class does all the work here
         */
        if (CCHardwareBot.BoKHardwareStatus.BOK_HARDWARE_FAILURE == robot.initHardware(this)) {
            telemetry.addData("Status", "ERROR: Hardware NOT initialized");
            telemetry.update();
        } else {
            // Send telemetry message to update hardware status
            telemetry.addData("Status", "Hardware initialized");
            telemetry.update();

            if (CCAuto.BoKAutoStatus.BOK_AUTO_FAILURE ==
                    autoImpl.initSoftware(this, robot)) {
                telemetry.addData("Status", "ERROR: Software NOT initialized");
                telemetry.update();
            } else {
                status = true; // Hardware and software initialized!
                // Send telemetry message to update status
                telemetry.addData("Status", "Initialization complete. READY!");
                telemetry.update();
            }
        }

        // Wait for the game to start (driver presses PLAY)

        waitForStart();

        // Run the autonomous operation, if hardware and software is initialized
        // else do nothing
        if (opModeIsActive()) {
            CCAuto.runTimeOpMode.reset(); // time the opMode to decide if we have time
            autoImpl.runSoftware();
            while (opModeIsActive()) {
            }
        }
    }
}
