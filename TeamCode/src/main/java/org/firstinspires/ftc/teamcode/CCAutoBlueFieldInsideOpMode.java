package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "CC Auto BLUE Field Inside", group = "CCAutoBlue")
@Disabled
public class CCAutoBlueFieldInsideOpMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        autoImpl = new CCAutoBlueFieldInside(); // use interface (polymorphism)
        super.runOpMode();
    }
}
