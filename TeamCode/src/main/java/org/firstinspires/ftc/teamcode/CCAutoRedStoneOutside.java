package org.firstinspires.ftc.teamcode;

public class CCAutoRedStoneOutside extends CCAutoCommon {
    // Constructor
    public CCAutoRedStoneOutside() {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_RED;
    }

    @Override
    public void runSoftware() {
        runAuto(false, true, false);
    }
}
