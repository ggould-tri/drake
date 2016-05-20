Sailboat Example
================

ggould-tri has a Ragazza 1m RC sailboat that he is modeling in Drake.  This is
where the current state of that model lives.

Notes
-----

 * The boat is a jibhead bermuda rig with a deep, heavy keel.
   * It is approximately 1 meter long and 2 meters tall (keel to crane).
   * The rudder is deep and vertical (stable at high speed, poor at low speed).
   * The mainsail is a non-rigid racing airfoil design with semi-rigid head.
 * The boat has two actuators.
   * An angle-commanded servo controls the rudder via a four-bar linkage.
   * An angle-commanded servo winch controls both main and jib sheets.
     * Limits max angle in either direction the mainsail and jib may swing.
* There is no sensor package yet.
   * Initial target package is ahrs, gnss, and wind direction.
     * NOTE:  Magentometers must be heel compensated!
   * Windspeed is trickier.
   * Strain guages on the sheets would be very, very nice to have.
   * In addition the RC controller inputs will be available.
 * Staged development plan:
   # Assisted teleop (command rudder, autonomously trim sail).
   # Plan-free autonomy (command heading, autonomously steer and trim)
   # Navigate-to-position (for simple time trials)
   # Navigate-to-pose (for solo racing)
   # Navigate-to-pose-at-time (for complex racing)

