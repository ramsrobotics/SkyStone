package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Krishna Saxena on 10/3/2017.
 * Registers the opMode with the driver station.
 * It uses CCMecanumDT and CCAutoRedStoneInside objects
 */
@Autonomous(name="CC Auto RED Stone Inside", group="CCAutoRed")

public class CCAutoRedStoneInsideOpMode extends CCAutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new CCAutoRedStoneInside(); // use interface (polymorphism)
        super.runOpMode();
    }
}