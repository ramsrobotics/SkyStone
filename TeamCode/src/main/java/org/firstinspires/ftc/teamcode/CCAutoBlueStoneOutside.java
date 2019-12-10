package org.firstinspires.ftc.teamcode;

public class CCAutoBlueStoneOutside extends CCAutoCommon {
    // Constructor
    public CCAutoBlueStoneOutside() {
        allianceColor = BoKAllianceColor.BOK_ALLIANCE_BLUE;
    }

    @Override
    public void runSoftware() {
        runAuto(false, true);
    }
}
