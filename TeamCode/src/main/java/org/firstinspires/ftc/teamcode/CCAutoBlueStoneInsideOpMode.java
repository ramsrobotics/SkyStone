package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Krishna Saxena on 10/3/2017.
 * Registers the opMode with the driver station.
 * It uses CCMecanumDT and CCAutoBlueStoneInside objects
 */
@Autonomous(name="CC Auto BLUE Stone Inside", group="CCAutoBlue")

public class CCAutoBlueStoneInsideOpMode extends CCAutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new CCAutoBlueStoneInside(); // use interface (polymorphism)
        super.runOpMode();
    }
}