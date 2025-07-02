package org.firstinspires.ftc.teamcode.util;

public enum ElevatorPosition {
    DOWN(0),
    INTAKE(105),
    UPTHING(2180), UPPERBASKET(6010);
    private int position;

    private ElevatorPosition(int position) {
        this.position = position;
    }

    public static ElevatorPosition nextHighest(int setPoint) {
        for (ElevatorPosition elPos : values()) {
            if (elPos.getPosition() > setPoint) {
                return elPos;
            }
        }
        return UPPERBASKET;
    }
    public static ElevatorPosition nextLowest(int setPoint) {
        ElevatorPosition[] elVals = values();
        for (int i = elVals.length - 1; i >= 0; i--) {
            ElevatorPosition pos = elVals[i];
            if (pos.getPosition() < setPoint) {
                return pos;
            }
        }
        return DOWN;
    }

    public int getPosition() {
        return position;
    }
}
