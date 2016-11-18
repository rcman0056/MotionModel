package modules.master_navs

import golem.*
import golem.containers.*
import golem.matrix.*
import golem.util.logging.asYaml
import golem.util.logging.log
import navutils.containers.NavSolution
import navutils.containers.Pose
import navutils.containers.Vector3
import navutils.correctDcmWithTilt
import navutils.eastToDeltaLon
import navutils.northToDeltaLat
import java.util.*
import insect.containers.INSSolution

/**
 * Example MasterNav implementation that works with the Pinson15 StateBlock
 * and the InertialNED class. User feeds in reference trajectory points and
 * estimated errors. This class stores and combines these inputs to produce
 * a corrected result.
 *
 * @param sec: Number of seconds of data to store for both the reference and
 * estimated errors.
 */
class MasterNavNED(val sec: Double = Double.MAX_VALUE) : MasterNav() {

    /**
     * Store an error state estimate for use in calculating a solution. All
     * fields are converted to column vectors if necessary.
     *
     * @param t: Time of validity for estimate.
     * @param e: 15x1 error state estimate from a Pinson15 block.
     * @param s: 15x1 error state sigma from a Pinson15 block.
     *
     * @return: Unit
     */
    override fun giveErrorState(t: Time, e: Matrix<Double>, s: Matrix<Double>) {
        removeIdenticalErrTime(t)
        errorList.add(errorWithCols(TimedStateEstimate(t, e, s)))
        errorList.sort()
        trimError()
    }

    /**
     * Performs a binary search for the error estimates at time t.time.
     *
     * @param t: Requested solution time.
     * @return: Error state vector at time t.time. If an exact match is not
     * available, the returned errors will be linearly interpolated from the
     * available values immediately before and after requested time. If the
     * requested time is outside the available range of data, the errors
     * returned will be the earliest or latest available, as appropriate.
     * All appropriate fields in returned value are column vectors.
     */
    override fun getErrorStateAtTime(t: Time): TimedStateEstimate {
        if (errorList.isEmpty())
            throw ArrayIndexOutOfBoundsException("No error state estimates" +
                                                 " available at time ${t.time}.")
        var ind = errorBinarySearch(t)
        if (ind < 0) {
            return linearInterpolateEstimates(ind, t)
        } else (return errorList[ind])
    }

    /**
     * Returns a solution at ins.navSolution.pose.time, which is a additive
     * combination of the input inertial solution (ins) and possibly
     * interpolated Pinson15 error state values.
     *
     * @return: Corrected inertial solution, or the uncorrected inertial
     * solution if no error states are available at the requested time.
     * All appropriate fields in returned value are column vectors.
     */
    override fun getSolution(ins: INSSolution): NavSolution {
        return getSolution(ins.navSolution)
    }

    /**
     * Returns a solution at ins.pose.time, which is a additive
     * combination of the input inertial solution (ins) and possibly
     * interpolated Pinson15 error state values.
     *
     * @return: Corrected inertial solution, or the uncorrected inertial
     * solution if no error states are available at the requested time.
     * All appropriate fields in returned value are column vectors.
     */
     fun getSolution(ins: NavSolution): NavSolution {
        try {
            var errors = getErrorStateAtTime(ins.pose.time)
            checkErrorTimes(ins.pose.time)
            return applyErrors(ins, errors)
        } catch(e: ArrayIndexOutOfBoundsException) {
            this.log {
                asYaml("MasterNavError",
                        "UncorrectedSolution" to e)
            }
            return ins
        }
    }

    /**
     * Checks if the MasterNav possesses the required range of error state
     * estimates to calculate a meaningful corrected solution.
     *
     * @param t: The time to check
     * @return: True if a call to getSolution(t) will provide a valid
     * result, meaning that the requested time falls within the
     * time range of provided error state estimates.
     */
    fun solutionValid(t: Time): Boolean {
        val errRange = getErrorStateTimeRange()
        if (errRange != null) {
            return errRange[0] <= t.time && errRange[1] >= t.time
        }else {
            return false
        }
    }

    /**
     * Corrects a WGS position with NED frame position error estimates.
     *
     * @param pos: WGS position (latitude (rad), longitude (rad),
     * altitude (m)), 3x1.
     * @param err: NED position error estimates (North (m), East (m),
     * Down (m)), 3x1.
     *
     * @return Corrected WGS position.
     */
    fun correctPosition(pos: Vector3, err: Matrix<Double>): Vector3 {
        val correctedLat = pos[0] + northToDeltaLat(err[0], pos[0])
        val correctedLon = pos[1] + eastToDeltaLon(err[1], pos[0])
        val correctedAlt = pos[2] - err[2]
        return Vector3(correctedLat, correctedLon, correctedAlt)
    }

    /**
     * Corrects NED frame velocity values with NED frame additive error
     * estimates.
     *
     * @param vel: Calculated velocity value (m/s), 3x1.
     * @param err: Estimate of error in vel (m/s), 3x1.
     *
     * @return: Corrected velocity values.
     */
    fun correctVelocity(vel: Matrix<Double>,
                        err: Matrix<Double>): Matrix<Double> {
        return vel + err
    }

    /**
     * Corrects the inertial estimated rotation matrix (body to nav) with
     * estimated tilt errors between the estimated NED frame and the true NED
     * frame.
     *
     * @param cbn: The 3x3 rotation DCM stored in a NavSolution produced by
     * insect.InertialNED.
     * @param tilts: 3x1 or 1x3 vector of tilt errors between the estimated
     * NED frame and the 'true' NED frame, as estimated by in a Pinson15 block.
     *
     * @return: The corrected Nav to Body DCM.
     */
    fun correctDcm(cbn: Matrix<Double>, tilts: Matrix<Double>): Matrix<Double> {
        return correctDcmWithTilt(cbn.T, tilts).T
    }

    /**
     * Converts estimate and sigma fields of a TimedStateEstimate to column vectors.
     *
     * @param err: Input TimedStateEstimate.
     *
     * @return: Same as input with estimate and sigma  forced to be cols.
     */
    private fun errorWithCols(err: TimedStateEstimate): TimedStateEstimate {
        return err.copy(estimate = err.estimate.asColVector(),
                        sigma = err.sigma.asColVector())
    }

    /**
     * Allows for the removal of an error state with a time identical to
     * one that is proposed to be added. As this should be called with
     * every potential addition, there should never be more than 1 possible
     * conflict.
     *
     * @param t: Time of error vector proposed to be added.
     */
    private fun removeIdenticalErrTime(t: Time) {
        if (!errorList.isEmpty()) {
            if (t.time <= errorList[errorList.size - 1].time.time) {
                var ind = errorBinarySearch(t)
                if (ind >= 0) {
                    errorList.removeAt(ind)
                }
            }
        }
    }

    /**
     * Performs a binary search of the error state list with a dummy
     * value.
     *
     * @param t: Time to search for.
     *
     * @return: Index of the matching time if found, or -insertion_point - 1
     * if an exact match not found (from Collections.binarySearch)
     */
    private fun errorBinarySearch(t: Time): Int {
        return errorList.binarySearch(TimedStateEstimate(t, zeros(1), zeros(1)))
    }

    /**
     * Retrieves the valid time range of stored error estimate points.
     *
     * @return: Earliest and latest times available, or null if no
     * points available.
     */
    private fun getErrorStateTimeRange(): DoubleArray? {
        if (errorList.isEmpty())
            return null
        return doubleArrayOf(errorList[0].time.time,
                             errorList[errorList.size - 1].time.time)

    }

    /**
     * Get the closest available error estimate. This function
     * should only be called when a binary search fails to find an exact
     * time match.
     *
     * @param ind: Index returned from a failed binary search of the error
     * estimates for one matching time. As this is from a failed search,
     * the index should be negative.
     * @param time: Requested time of solution.
     *
     * @return: Error estimates valid at time, or as close as possible to it.
     * A prior call to checkErrorTimes will generate a warning if the value
     * used is not exact or properly interpolated.
     */
    private fun linearInterpolateEstimates(ind: Int, time: Time):
            TimedStateEstimate {
        var indicies = adjustInterpolationIndices(ind, errorList.size)
        if (indicies.size == 1) {
            return errorList[indicies[0]]
        } else {
            var early = errorList[indicies[0]]
            var late = errorList[indicies[1]]
            var ratio = (time.time - early.time.time) /
                        (late.time.time - early.time.time)
            return early.copy(time, early.estimate + (late.estimate -
                                                      early.estimate) * ratio, early.sigma +
                                                                               (late.sigma - early.sigma) * ratio)
        }
    }

    /**
     * Takes the index from a failed binary search of a sorted list and
     * determines the indices of the most valid values to use in interpolation.
     *
     * @param ind: Index returned by failed binary search.
     * @param checkSize: The size of the list that was searched.
     *
     * @return: An IntArray containing 0 if the search failed because the
     * requested value was before any data, checkSize - 1 if it was after,
     * and the indices of the two elements to interpolate over in all other
     * cases.
     */
    private fun adjustInterpolationIndices(ind: Int, checkSize: Int): IntArray {
        val before = -ind - 2
        val after = before + 1
        if (after == checkSize)
            return intArrayOf(checkSize - 1)
        if (before < 0)
            return intArrayOf(0)
        return intArrayOf(before, after)
    }

    /**
     * Applies the error state estimates to a NavSolution object. Inputs must
     * should valid at the same time. Will not be the case if providing an
     * 'out of range' solution.
     *
     * @param n: Reference trajectory point.
     * @param b: Time-stamped error state estimates.
     *
     * @return: Corrected NavSolution.
     */
    private fun applyErrors(n: NavSolution, b: TimedStateEstimate):
            NavSolution {
        val pos = correctPosition(n.pose.pos, b.estimate[0..2, 0])
        val vel = correctVelocity(n.vel, b.estimate[3..5, 0])
        val dcm = correctDcm(n.pose.rotMat, b.estimate[6..8, 0])
        val pose = Pose(dcm, pos, n.pose.time)
        return NavSolution(pose, vel)
    }

    /**
     * Removes measurements that break the maximum time allowed when compared
     * to the latest available.
     */
    private fun trimError() {
        if (!errorList.isEmpty()) {
            var errMax = errorList[errorList.size - 1].time.time
            errorList = ArrayList(errorList.filter { it ->
                abs(it.time.time - errMax) <= sec
            })
        }
    }

    /**
     * Checks the time of a requested errors estimate and displays warnings
     * about how far off a solution might be.
     *
     * @param t: Requested solution time.
     * @return: Unit
     */
    private fun checkErrorTimes(t: Time) {
        val errRange = getErrorStateTimeRange()
        produceRangeWarnings(t, "error estimates", errRange)
    }

    /**
     * Checks if time is within given range and produces warnings if not.
     *
     * @param t: Requested solution time.
     * @param desc: Describes the data param r was derived from.
     * @param r: 2-length array of valid times, earliest and latest available,
     * in that order.
     *
     * @return: Unit
     */
    private fun produceRangeWarnings(t: Time, desc: String, r: DoubleArray?) {
        if (r != null) {
            var bad: Double? = null
            if (t.time < r[0]) {
                bad = r[0]
            } else if (t.time > r[1]) {
                bad = r[1]
            }
            if (bad != null) {
                this.log {
                    asYaml("MasterNavError",
                            "ApplyingNonExactErrorStateEstimates" to "Requested $desc at" +
                                    " ${t.time} unavailable, using point at $bad \n")
                }
            }
        }
    }
}

