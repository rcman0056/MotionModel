package utils

import golem.*
import golem.matrix.*
import scorpion.filters.sensor.interfaces.LinearizedFilter

/**
 * A helper to quickly run a filter and collect its output. We often end up re-writing the same for loop
 * that takes a set of data in and runs the filter iteratively, collecting the output matrices at each time
 * index into an output array of estimates/covariances. This function does that for you.
 *
 * @param f A function that takes a time in and then runs the filter's update command.
 * @param dt The delta-t between times passed into f
 * @param startTime The time to initially pass into f
 * @param endTime The last time to pass into f
 * @param blockName The name of the StateBlock to record the outputs of (only accepts one for now)
 *
 * @return a RunResults instance that captures all the estimates/covariances of the chosen StateBlock
 *
 */
fun LinearizedFilter.collectRun(blockName: String,
                                startTime: Double,
                                endTime: Double,
                                dt: Double,
                                f: (time: Double) -> Unit): RunResults {
    // TODO: Accept more than one block name to return.
    var time = arange(startTime, endTime, dt)
    var numStates = this.getStateBlock(blockName)?.numStates ?: throw Exception("Block not found.")
    var out = RunResults(rawStates = zeros(numStates, time.numCols()),
                         rawCovDiags = zeros(numStates, time.numCols()),
                         rawTimes = time)


    time.forEachIndexed { i, t ->
        f(t)
        var cov = this.getStateBlockCovariance(blockName) ?: throw Exception("Block not found.")
        var state = this.getStateBlockEstimate(blockName) ?: throw Exception("Block not found.")
        (0..numStates - 1).forEach {
            out.rawCovDiags[it, i] = cov[it, it]
            out.rawStates[it, i] = state[it]
        }
    }
    return out
}

/**
 * A container for the results of a filter run.
 *
 * @param rawStates The raw dump of all the states at all times.
 * @param rawCovDiags The raw dump of all the covariances of all states at all times.
 * @param rawTimes The time indices for the other raw outputs.
 */
class RunResults(var rawStates: Matrix<Double>, var rawCovDiags: Matrix<Double>, var rawTimes: Matrix<Double>) {
    /**
     * Get the time series for a particular state
     */
    fun state(state: Int): Matrix<Double> {
        return rawStates[state, 0..rawStates.numCols() - 1]
    }

    /**
     * Get the time series for a particular term in the covariance matrix
     */
    fun covDiag(state: Int): Matrix<Double> {
        return rawCovDiags[state, 0..rawCovDiags.numCols() - 1]
    }
}
