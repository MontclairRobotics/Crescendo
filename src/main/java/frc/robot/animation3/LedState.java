package frc.robot.animation3;

public enum LedState {
    ALLIANCE(Animations.allianceAnimation()),
    RAINBOW(Animations.rainbowAnimation());

    private final Runnable animation;

    public Runnable getAnimation() {
        return animation;
    }
    private LedState(Runnable targetAnimation) {
        this.animation = targetAnimation;
    }
}
