package modules.master_navs

import golem.containers.*
import golem.matrix.*
import insect.containers.INSSolution
import navutils.containers.NavSolution
import java.util.*

/**
 * This class acts as server for navigation data that may be used by many
 * resources (MeasurementProcessors, StateBlocks, plotting, etc). It stores
 * a time-history of the states that estimate the error in a reference.
 * Other members can then request either the error states, or provide an
 * inertial solution and obtain the combined corrected solution at
 * any point in time, including between measurements.
 * Implementations are specific to the combination of trajectory source and
 * the error states used.
 */
abstract class MasterNav {

    /* Holds the error state estimate, uncertainty sigma, and time. */
    protected var errorList = ArrayList<TimedStateEstimate>()

    /**
     * Stores an error state estimate and uncertainty for use in calculating
     * a corrected solution.
     *
     * @param t: Time of validity for estimate.
     * @param e: nx1 error state estimate.
     * @param s: nx1 error state sigma.
     *
     * @return: Unit
     */
    abstract fun giveErrorState(t: Time, e: Matrix<Double>, s: Matrix<Double>)

    /**
     * Returns the error state estimate and uncertainty sigma at time t,
     * interpolating if necessary.
     *
     * @param t: Requested solution time.
     * @return: Error state vector and sigma at requested time, wrapped in
     * a TimeStampedStateBlockEstimate.
     */
    abstract fun getErrorStateAtTime(t: Time): TimedStateEstimate

    /**
     * Returns a solution at ins.navSolution.pose.time, which is a combination
     * of the ins solution and error states at the given time.
     * @param ins: Time-tagged, uncorrected INS solution.
     * @return NavSolution: ins.navSolution with closest applicable error
     * states applied, or just ins.navSolution if no error states are available.
     */
    abstract fun getSolution(ins: INSSolution): NavSolution
}

