package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "CC Auto RED Field Inside", group = "CCAutoRed")
@Disabled
public class CCAutoRedFieldInsideOpMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        autoImpl = new CCAutoRedFieldInside(); // use interface (polymorphism)
        super.runOpMode();
    }
}
