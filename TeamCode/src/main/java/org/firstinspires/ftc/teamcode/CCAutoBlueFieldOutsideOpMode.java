package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CC Auto BLUE Field Outside", group = "CCAutoBlue")

public class CCAutoBlueFieldOutsideOpMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new CCAutoBlueFieldOutside(); // use interface (polymorphism)
        super.runOpMode();
    }
}
