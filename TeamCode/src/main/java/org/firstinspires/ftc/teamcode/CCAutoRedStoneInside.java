package org.firstinspires.ftc.teamcode;

public class CCAutoRedStoneInside extends CCAutoCommon {

    // Constructor
    public CCAutoRedStoneInside() {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware() {
        runAuto(true, true, false);
    }
}