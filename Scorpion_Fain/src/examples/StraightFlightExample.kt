import golem.*
import golem.containers.*
import modules.measurements.AltitudeMeasurementProcessor
import modules.measurements.PositionMeasurementProcessor
import modules.stateblocks.ins.Pinson15AuxData
import modules.stateblocks.ins.Pinson15NEDBlock
import modules.stateblocks.ins.getImuModelHG9900
import navutils.containers.NavSolution
import navutils.containers.Pose
import navutils.containers.Vector3
import org.knowm.xchart.BitmapEncoder
import scorpion.buffers.Buffer
import scorpion.filters.sensor.StandardSensorEKF
import scorpion.filters.sensor.containers.Measurement
import utils.collectRun

/**
 *  An example filter with an aircraft flying 1m/s north straight and level.
 */
object StraightFlightExample {
    @JvmStatic
    fun main(args: Array<String>) {
        val model = getImuModelHG9900()
        val block = Pinson15NEDBlock(label = "pinson15",
                                     imuModel = model)

        val filter = StandardSensorEKF(Time(0.0, TimeStandard.WALL),
                                       buffer = Buffer(standard = TimeStandard.WALL))


        filter.addStateBlock(block)


        val initCov = zeros(15, 15)
        initCov[0..2, 0..2] = eye(3) * pow(100.0, 2)
        initCov[3..5, 3..5] = eye(3) * 1.0
        initCov[6..8, 6..8] = eye(3) * .001
        filter.setStateBlockCovariance(label = "pinson15",
                                       covariance = initCov)

        filter.addMeasurementProcessor(AltitudeMeasurementProcessor("altimeter", blockLabel = "pinson15"))
        filter.addMeasurementProcessor(PositionMeasurementProcessor("gps++", pinsonLabel = "pinson15"))


        // For this simple example we will linearize INS about a fixed point
        val pose = Pose(rotMat = eye(3),
                        pos = Vector3(0.0, 0.0, 0.0),
                        time = Time(0.0))
        val navSol = NavSolution(pose = pose,
                                 vel = mat[1, 0, 0].T)
        filter.giveStateBlockAuxData(label = "pinson15",
                                     data = Pinson15AuxData(navSolution = navSol,
                                                            force = mat[0.0, 0.0, 9.8]))

        val output = filter.collectRun("pinson15",
                                       startTime = .1,
                                       endTime = 600.0,
                                       dt = .10) { t ->

            filter.propagate(Time(t))
            if (abs(t % 1.0) < 0.001) {
                filter.update(Measurement("altimeter", Time(t), Time(t), mat[100], null, mat[1.0]))
            }
            if (abs(t % 10.0) < 0.0001) {
                filter.update(Measurement("gps++", Time(t), Time(t), mat[t, 0, 0].T, null, eye(3) * 1.001))
            }
        }
        figure(1)
        val t = output.rawTimes
        plot(t, sqrt(output.covDiag(state = 0)), "k", "North error")
        plot(t, sqrt(output.covDiag(state = 1)), "b", "East error")
        //    plot(t,output.covDiag(state=0).mapMat { deltaLatToNorth(it, approxLat = 0.0) }, "k", "North error")
        //    plot(t,output.covDiag(state=1).mapMat { deltaLatToNorth(it, approxLat = 0.0) }, "b", "East error")
        plot(t, sqrt(output.covDiag(state = 2)), "g", "Down error")
        xlabel("Time (s)")
        ylabel("Error (m)")
        title("Position State Sigmas")

        figure(2)
        plot(t, sqrt(output.covDiag(state = 3)), "k", "North error")
        plot(t, sqrt(output.covDiag(state = 4)), "b", "East error")
        plot(t, sqrt(output.covDiag(state = 5)), "g", "Down error")
        xlabel("Time (s)")
        ylabel("Error (m/s)")
        title("Velocity State Sigmas")

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

        Thread.sleep(50)
        var c = figures[1]!!.first
        BitmapEncoder.saveBitmapWithDPI(c, "./PosErr", BitmapEncoder.BitmapFormat.PNG, 300);

        Thread.sleep(50)
        c = figures[2]!!.first
        BitmapEncoder.saveBitmapWithDPI(c, "./VelErr", BitmapEncoder.BitmapFormat.PNG, 300);

        Thread.sleep(50)
        c = figures[3]!!.first
        BitmapEncoder.saveBitmapWithDPI(c, "./TiltErr", BitmapEncoder.BitmapFormat.PNG, 300);

        Thread.sleep(50)
        c = figures[4]!!.first
        BitmapEncoder.saveBitmapWithDPI(c, "./PosSoln", BitmapEncoder.BitmapFormat.PNG, 300);

        Thread.sleep(50)
        c = figures[5]!!.first
        BitmapEncoder.saveBitmapWithDPI(c, "./VelSoln", BitmapEncoder.BitmapFormat.PNG, 300);

        Thread.sleep(50)
        c = figures[6]!!.first
        BitmapEncoder.saveBitmapWithDPI(c, "./TiltSoln", BitmapEncoder.BitmapFormat.PNG, 300);

    }
}