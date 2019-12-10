package org.firstinspires.ftc.teamcode;

public class CCAutoBlueFieldInside extends CCAutoCommon {
    public CCAutoBlueFieldInside()
    {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware() {
        runAuto(true, false);
    }
}
