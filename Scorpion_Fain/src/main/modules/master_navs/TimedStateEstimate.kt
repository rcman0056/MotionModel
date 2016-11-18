package modules.master_navs

import golem.containers.*
import golem.matrix.*

/**
 * Class that wraps up a StateBlock estimate and uncertainty with a time
 * so they can be sorted/searched and returned to the user.
 *
 * @param time: The time for which the error estimates are valid.
 * @param estimate: Error estimate.
 * @param sigma: 1-sigma uncertainty in state values.
 */
data class TimedStateEstimate(val time: Time,
                              val estimate: Matrix<Double>,
                              val sigma: Matrix<Double>) :
        Comparable<TimedStateEstimate> {
    /**
     * Function that compares 2 instantiations of this class based on time.
     * Allows use of sort() and binarySearch() methods.
     * @param other: Another instantiation of this class.
     *
     * @return: -1 if other is younger than this instance, 1 if older,
     * and 0 if they are the same age.
     */
    override fun compareTo(other: TimedStateEstimate): Int {
        if (other.time.time > this.time.time) {
            return -1
        } else if (other.time.time < this.time.time) {
            return 1
        } else {
            return 0
        }
    }
}