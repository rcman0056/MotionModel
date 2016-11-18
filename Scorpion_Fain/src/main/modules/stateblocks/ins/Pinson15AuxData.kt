package modules.stateblocks.ins

import golem.matrix.*
import navutils.containers.NavSolution

/**
 * Passes into a [Pinson15Block] the set of current values from a running INS mechanization to linearize about.
 *
 * @param force NED specific force vector
 * @param navSolution the current solution from the INS mechanization
 */
data class Pinson15AuxData(var navSolution: NavSolution, var force: Matrix<Double>)
