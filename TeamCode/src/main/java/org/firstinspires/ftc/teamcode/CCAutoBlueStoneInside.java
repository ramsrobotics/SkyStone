package org.firstinspires.ftc.teamcode;


public class CCAutoBlueStoneInside extends CCAutoCommon {
    // Constructor
    public CCAutoBlueStoneInside() {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware() {
        runAuto(true, true);
    }
}
