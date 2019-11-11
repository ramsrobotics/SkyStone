package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Krishna Saxena on 10/3/2017.
 * Registers the opMode with the driver station.
 * It uses CCMecanumDT and CCAutoRedCrater objects
 */
@Autonomous(name="CC Simple Auto", group="Auto")
@Disabled
public class CCAutoRedCraterOpMode extends CCAutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
     //   autoImpl = new CCAutoRedCrater(); // use interface (polymorphism)
        super.runOpMode();
    }
}