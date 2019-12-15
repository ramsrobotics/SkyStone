package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "CC Auto RED Field Outside", group = "CCAutoRed")
@Disabled
public class CCAutoRedFieldOutsideOpMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        autoImpl = new CCAutoRedFieldOutside(); // use interface (polymorphism)
        super.runOpMode();
    }
}
