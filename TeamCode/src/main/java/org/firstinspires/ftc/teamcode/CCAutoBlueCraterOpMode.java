package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Krishna Saxena on 10/3/2017.
 * Registers the opMode with the driver station.
 * It uses CCMecanumDT and CCAutoBlueCrater objects
 */
@Autonomous(name="CC Auto BLUE Stone", group="BoKBlue")

public class CCAutoBlueCraterOpMode extends CCAutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new CCAutoBlueCrater(); // use interface (polymorphism)
        super.runOpMode();
    }
}