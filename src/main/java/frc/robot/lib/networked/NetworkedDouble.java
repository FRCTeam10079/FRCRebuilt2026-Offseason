// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.networked;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import java.util.EnumSet;

/*
 * Simplifies usage of doubles in NT
 * Used in this project to simplify code
 *
 * I didn't realize til after making this but NetworkTableEntry might work better here
 */
public class NetworkedDouble {
  private DoubleTopic doubleTopic;
  private DoublePublisher doublePublisher;
  private DoubleSubscriber doubleSubscriber;

  private NetworkTableListener doubleListener;

  // this value is read and written to from multiple threads
  private volatile boolean newData = false;

  /**
   * Creates a networkedDouble object
   *
   * @param topic_string the name of the topic
   * @param default_value the default double value to set the topic to
   */
  public NetworkedDouble(String topic_string, double default_value) {
    NetworkTableInstance defaultNT = NetworkTableInstance.getDefault();
    this.doubleTopic = defaultNT.getDoubleTopic(topic_string);

    this.doublePublisher = doubleTopic.publish();
    this.doubleSubscriber = doubleTopic.subscribe(default_value);

    this.doublePublisher.set(default_value);

    // ONLY detects REMOTE value updates
    // not designed to detect local code changes
    doubleListener = NetworkTableListener.createListener(
        doubleTopic, EnumSet.of(NetworkTableEvent.Kind.kValueRemote), (NetworkTableEvent event) -> {
          // legit just a lambda to make newData true
          // hopefully thread safe ♥
          this.newData = true;
        });
  }

  /**
   * Returns whether or not new data is available
   *
   * <p>Once this function returns true, it will not return true until new data is found
   *
   * @return
   */
  public boolean available() {
    if (!this.newData) {
      return false;
    }
    this.newData = false;
    return true;
  }

  /** Closes active listeners */
  public void close() {
    doubleListener.close();
  }

  /**
   * Sets the value via the publisher
   *
   * @param value the value to set
   */
  public void set(double value) {
    this.doublePublisher.set(value);
  }

  /**
   * Gets the value from the subscriber
   *
   * @return the value recieved from the subscriber
   */
  public double get() {
    return this.doubleSubscriber.get();
  }
}
