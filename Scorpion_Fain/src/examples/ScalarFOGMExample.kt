import golem.*
import golem.containers.*
import modules.measurements.DirectMeasurementProcessor
import modules.stateblocks.common.FOGMState
import scorpion.filters.sensor.StandardSensorEKF
import scorpion.filters.sensor.containers.Measurement
import utils.collectRun

object ScalarFOGMExample {
    @JvmStatic
    fun main(args: Array<String>) {
        var block = FOGMState(label = "myblock",
                              biasSigma = 10.0,
                              timeConstant = 50.0)

        var filter = StandardSensorEKF()
        val processor = DirectMeasurementProcessor(label = "myprocessor",
                                                   HValue = 1.0,
                                                   blockLabel = "myblock")
        filter.addStateBlock(block)
        filter.addMeasurementProcessor(processor)

        filter.setStateBlockEstimate("myblock", mat[0])
        filter.setStateBlockCovariance("myblock", mat[100])

        var output = filter.collectRun("myblock",
                                       startTime = .1,
                                       endTime = 600.0,
                                       dt = .10) { t ->

            filter.propagate(Time(t))

            if (abs(t % 100.0) < 0.001) {
                filter.update(Measurement("myprocessor", Time(t), Time(t), mat[10], null, mat[100.0]))
            }

        }

        figure(1)
        val state = output.state(state = 0)
        val t = output.rawTimes
        plot(t, state + sqrt(output.covDiag(state = 0)), "k", "Pos Cov Bound")
        plot(t, state - sqrt(output.covDiag(state = 0)), "k", "Neg Cov Bound")
        plot(t, state, "b", "State Value")
        xlabel("Time (s)")
        ylabel("Sigma (m)")
        title("Position State Sigma")

        figure(2)
        plot(t, output.state(state = 0), "k", "North Position")
        xlabel("Time (s)")
        ylabel("Value (m)")
        title("Position Solution")

    }
}