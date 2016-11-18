import golem.*
import golem.containers.*
import modules.measurements.BiasedAltitudeMeasurementProcessor
import modules.measurements.PositionMeasurementProcessor
import modules.stateblocks.common.FOGMState
import modules.stateblocks.ins.Pinson15AuxData
import modules.stateblocks.ins.Pinson15NEDBlock
import modules.stateblocks.ins.getImuModelHG9900
import navutils.containers.NavSolution
import navutils.containers.Pose
import navutils.containers.Vector3
import scorpion.buffers.Buffer
import scorpion.filters.sensor.StandardSensorEKF
import scorpion.filters.sensor.containers.Measurement
import utils.collectRun

/**
 *  Copy of StraightFlightExample that includes a baro w/ bias.
 */
object StraightFlightBaroBiasExample {
    @JvmStatic
    fun main(args: Array<String>) {
        val model = getImuModelHG9900()
        val block = Pinson15NEDBlock(label = "pinson15",
                                     imuModel = model)
        val baroBlock = FOGMState(label = "baroBias", biasSigma = 1.0,
                                  timeConstant = 3600.0)

        val filter = StandardSensorEKF(Time(0.0), buffer = Buffer(standard = TimeStandard.WALL))

        filter.addStateBlock(block)
        filter.addStateBlock(baroBlock)


        val initCov = zeros(15, 15)
        initCov[0..2, 0..2] = eye(3) * pow(100.0, 2)
        initCov[3..5, 3..5] = eye(3) * 1.0
        initCov[6..8, 6..8] = eye(3) * .001
        filter.setStateBlockCovariance(label = "pinson15",
                                       covariance = initCov)
        filter.setStateBlockCovariance(label = "baroBias", covariance = mat[1.0])


        val bias = 100.0

        filter.addMeasurementProcessor(BiasedAltitudeMeasurementProcessor(
                "altimeter", pinsonLabel = "pinson15", biasLabel = "baroBias"))
        filter.addMeasurementProcessor(PositionMeasurementProcessor(
                "gps++", pinsonLabel = "pinson15"))

        filter.giveStateBlockAuxData(label = "pinson15",
                                     data = Pinson15AuxData(navSolution = NavSolution(
                                             pose = Pose(rotMat = eye(3),
                                                         pos = Vector3(0.0, 0.0, 0.0),
                                                         time = Time(0.0)),
                                             vel = mat[1, 0, 0].T),
                                                            force = mat[0.0, 0.0, 9.8]))

        val time = arange(.1, 600.0, 0.10)
        val biasEst = zeros(1, time.numCols())
        val biasSig = zeros(1, time.numCols())
        var ind = 0
        val output = filter.collectRun("pinson15",
                                       startTime = .1,
                                       endTime = 600.0,
                                       dt = .10) { t ->

            if (abs(t % 1.0) < 0.001) {
                filter.update(Measurement("altimeter", Time(t), Time(t), mat[bias], null, mat[1.0]))
            }
            if (abs(t % 10.0) < 0.0001) {
                filter.update(Measurement("gps++", Time(t), Time(t), mat[t, 0, 0].T, null, eye(3) * 3.0))
            } else {
                filter.propagate(Time(t))
            }
            biasEst[0, ind] = filter.getStateBlockEstimate("baroBias")[0, 0]
            biasSig[0, ind] = sqrt(filter.getStateBlockCovariance(
                    "baroBias")[0, 0])
            ind += 1
        }
        figure(1)
        val t = output.rawTimes
        plot(t, sqrt(output.covDiag(state = 0)), "k", "North error")
        plot(t, sqrt(output.covDiag(state = 1)), "b", "East error")
        plot(t, sqrt(output.covDiag(state = 2)), "g", "Down error")
        xlabel("Time (s)")
        ylabel("Error (m)")
        title("Position Error")

        figure(2)
        plot(t, sqrt(output.covDiag(state = 3)), "k", "North error")
        plot(t, sqrt(output.covDiag(state = 4)), "b", "East error")
        plot(t, sqrt(output.covDiag(state = 5)), "g", "Down error")
        xlabel("Time (s)")
        ylabel("Error (m/s)")
        title("Velocity Error")

        figure(3)
        plot(t, sqrt(output.covDiag(state = 6)), "k", "Roll error")
        plot(t, sqrt(output.covDiag(state = 7)), "b", "Pitch error")
        plot(t, sqrt(output.covDiag(state = 8)), "g", "Yaw error")
        xlabel("Time (s)")
        ylabel("Error (rad)")
        title("Tilt Error")

        figure(4)
        plot(t, output.state(state = 0), "k", "North Position")
        plot(t, output.state(state = 1), "b", "East Position")
        plot(t, output.state(state = 2), "g", "Down Position")
        xlabel("Time (s)")
        ylabel("Position (m)")
        title("Position Solution")

        figure(5)
        plot(t, output.state(state = 3), "k", "North error")
        plot(t, output.state(state = 4), "b", "East error")
        plot(t, output.state(state = 5), "g", "Down error")
        xlabel("Time (s)")
        ylabel("Position (m/s)")
        title("Velocity Solution")

        figure(6)
        plot(t, output.state(state = 6), "k", "Roll")
        plot(t, output.state(state = 7), "b", "Pitch")
        plot(t, output.state(state = 8), "g", "Yaw")
        xlabel("Time (s)")
        ylabel("Angle (rad)")
        title("Tilt Solution")

        figure(7)
        plot(t, biasEst, "k", "BaroBias")
        plot(t, biasSig, "r", "Bias Sigma")
        plot(t, -biasSig, "r")
        xlabel("Time (s)")
        ylabel("Estimated Bias")
        title("Bias State (True = $bias)")
    }
}