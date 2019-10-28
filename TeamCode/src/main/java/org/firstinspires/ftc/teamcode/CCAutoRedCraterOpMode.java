package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Krishna Saxena on 10/3/2017.
 * Registers the opMode with the driver station.
 * It uses CCMecanumDT and CCAutoRedCrater objects
 */
@Autonomous(name="BoK Auto Crater", group="BoKRed")
//@Disabled
public class CCAutoRedCraterOpMode extends CCAutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
     //   autoImpl = new CCAutoRedCrater(); // use interface (polymorphism)
        super.runOpMode();
    }
}