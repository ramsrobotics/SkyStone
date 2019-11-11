package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="BoK Auto Test", group="BOKTest")
@Disabled
public class CCAutoTestOpMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new CCAutoTest(); // use interface (polymorphism)
        super.runOpMode();
    }
}
