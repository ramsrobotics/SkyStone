package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BoK Auto Test", group="BOKTest")
//@Disabled
public class CCAutoTestOpMode extends CCAutoOpMode {
    @Override
    public void runOpMode() throws InterruptedException
    {
        autoImpl = new CCAutoTest(); // use interface (polymorphism)
        super.runOpMode();
    }
}
