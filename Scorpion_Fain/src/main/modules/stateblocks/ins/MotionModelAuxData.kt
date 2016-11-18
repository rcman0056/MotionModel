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
                              var pitchrate: Double,
                              var yawrate: Double,
                              var roll: Double,
                              var pitch: Double){



}
/*
* @param airspeed in m/s
* @param pitchrate in rad/s
* @param yawrate in rad/s
* @param roll in rads
* @param pitch in rads
*/
