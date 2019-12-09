package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CC Auto BLUE Stone Outside", group = "CCAutoBlue")
public class CCAutoBlueStoneOutsideOpMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new CCAutoBlueStoneOutside(); // use interface (polymorphism)
        super.runOpMode();
    }
}
