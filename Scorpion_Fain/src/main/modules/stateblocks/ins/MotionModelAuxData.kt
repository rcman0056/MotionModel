package modules.stateblocks.ins //What do I include???????????????????????????/

import golem.matrix.*
//import navutils.containers.NavSolution

/**
 * Passes into a [Pinson15Block] the set of current values from a running INS mechanization to linearize about.
 *
 * @param force NED specific force vector
 * @param navSolution the current solution from the INS mechanization
 */
data class MotionModelAuxData(var airspeed: Double,
                              var airspeed_time: Double,
                              var pitchrate: Double,
                              var yawrate: Double,
                              var yaw_pitch_rate_time: Double,
                              var roll: Double,
                              var pitch :Double,
                              var roll_pitch_time: Double){



}
/*
* @param airspeed in m/s
* @param pitchrate in rad/s
* @param yawrate in rad/s
* @param roll in rads
* @param pitch in rads
*/
