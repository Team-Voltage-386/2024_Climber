package frc.robot.components;

public class MotorGoToPositionInfo {
    private double m_lastSpeed;
    private double m_lastTime;

    public MotorGoToPositionInfo(double lastSpeed, double lastTime) {
        m_lastSpeed = lastSpeed;
        m_lastTime = lastTime;
    }

    public double getLastSpeed() {
        return this.m_lastSpeed;
    }

    public void setLastSpeed(double lastSpeed) {
        this.m_lastSpeed = lastSpeed;
    }

    public double getLastTime() {
        return this.m_lastTime;
    }

    public void setLastTime(double lastTime) {
        this.m_lastTime = lastTime;
    }
}
