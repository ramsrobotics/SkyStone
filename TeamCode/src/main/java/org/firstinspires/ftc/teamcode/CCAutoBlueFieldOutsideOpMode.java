package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "CC Auto BLUE Field Outside", group = "CCAutoBlue")
@Disabled
public class CCAutoBlueFieldOutsideOpMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        autoImpl = new CCAutoBlueFieldOutside(); // use interface (polymorphism)
        super.runOpMode();
    }
}
