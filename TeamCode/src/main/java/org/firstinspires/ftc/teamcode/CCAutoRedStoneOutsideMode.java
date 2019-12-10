package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Registers the opMode with the driver station.
 * It uses CCMecanumDT and CCAutoRedStoneOutside objects
 */
@Autonomous(name = "CC Auto RED Stone Outside", group = "CCAutoRed")

public class CCAutoRedStoneOutsideMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        autoImpl = new CCAutoRedStoneOutside(); // use interface (polymorphism)
        super.runOpMode();
    }
}