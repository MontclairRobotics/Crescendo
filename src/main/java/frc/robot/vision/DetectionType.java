package frc.robot.vision;

public enum DetectionType { // TODO: UPDATE PIPELINES
  DRIVER(-1),
  NOTE(1),
  APRIL_TAG(0);

  private final int pipe;

  private DetectionType(int pipe) {
    this.pipe = pipe;
  }

  public int getPipe() {
    return pipe;
  }
}
