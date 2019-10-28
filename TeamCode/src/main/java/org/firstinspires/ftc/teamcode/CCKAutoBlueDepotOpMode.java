package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Krishna Saxena on 10/3/2017.
 * Registers the opMode with the driver station.
 * It uses CCMecanumDT and CCAutoBlueDepot objects
 */
@Autonomous(name="BoK Auto BLUE Depot", group="BoKBlue")
@Disabled
public class CCKAutoBlueDepotOpMode extends CCAutoOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new CCAutoBlueDepot(); // use interface (polymorphism)
        super.runOpMode();
    }
}