import golem.*
import golem.containers.*
import golem.matrix.*
import insect.InertialNED
import modules.master_navs.MasterNavNED
import modules.measurements.AltitudeMeasurementProcessor
import modules.measurements.PositionMeasurementProcessor
import modules.stateblocks.ins.Pinson15AuxData
import modules.stateblocks.ins.Pinson15NEDBlock
import modules.stateblocks.ins.getImuModelHG9900
import navutils.containers.NavSolution
import navutils.containers.Pose
import navutils.containers.Vector3
import navutils.deltaLatToNorth
import navutils.deltaLonToEast
import scorpion.buffers.Buffer
import scorpion.filters.sensor.StandardSensorEKF
import scorpion.filters.sensor.containers.Measurement
import java.util.ArrayList
import java.util.Random

object MasterNavExample {

    @JvmStatic
    fun main(args: Array<String>) {
        val eRate = 7.292115e-5

        // Set up filter with Pinson state block and processors
        val pl = "pinson15"
        val gl = "gps++"
        val al = "altimeter"
        val filter = StandardSensorEKF(curTime = Time(0.0), buffer = Buffer(standard = TimeStandard.WALL))
        val model = getImuModelHG9900()
        val block = Pinson15NEDBlock(label = pl, imuModel = model)
        filter.addStateBlock(block)

        val initCov = zeros(15, 15)
        initCov[0..2, 0..2] = eye(3) * pow(3.0, 2)
        initCov[3..5, 3..5] = eye(3) * 0.1
        initCov[6..8, 6..8] = eye(3) * .001
        initCov[9..11, 9..11] = eye(3) * .001
        initCov[12..14, 12..14] = eye(3) * .001
        filter.setStateBlockCovariance(label = pl, covariance = initCov)

        filter.addMeasurementProcessor(AltitudeMeasurementProcessor(al, pl))
        filter.addMeasurementProcessor(PositionMeasurementProcessor(gl, pl))

        // Set up inertial
        val lat = 0.5
        val lon = -1.0
        val alt = 1000.0
        val pose = Pose(rotMat = eye(3), pos = Vector3(lat, lon, alt),
                        time = Time(0.0))
        val vel = mat[0, 0, 0].T
        val ins = InertialNED(pose, vel)

        // Set up MasterNav, only keeping 10 sec of data at a time.
        var master = MasterNavNED(10.0)

        master.giveErrorState(filter.curTime, filter.getStateBlockEstimate(pl),
                              sqrt(filter.getStateBlockCovariance(pl).diag()))

        // Recording
        val mechOut: ArrayList<NavSolution> = ArrayList()
        val filtStateOut: ArrayList<Matrix<Double>> = ArrayList()
        val filtSigOut: ArrayList<Matrix<Double>> = ArrayList()
        val solOut: ArrayList<NavSolution> = ArrayList()
        mechOut.add(ins.solution)
        solOut.add(ins.solution)
        filtStateOut.add(filter.getStateBlockEstimate(pl))
        filtSigOut.add(sqrt(filter.getStateBlockCovariance(pl).diag()))

        val ran = Random()
        val dv = mat[0.0, -3.13224089e-6, -0.978912010509615]
        val dth = mat[cos(lat) * eRate, 0.0, -sin(lat) * eRate] * 0.1
        for (k in 1..36000) {
            val t = k * 0.1
            // Mechanize and feed master nav
            ins.mechanize(dv, dth, Time(t))

            // TODO: MasterNav could be used to store force as well-
            // just use inertial directly for now
            filter.giveStateBlockAuxData(label = pl,
                                         data = Pinson15AuxData(navSolution =
                                                                ins.getSolution(Time(t)).navSolution,
                                                                force = ins.getSolution(Time(t)).force))
            filter.propagate(Time(t))
            master.giveErrorState(Time(t), filter.getStateBlockEstimate(pl),
                                  sqrt(filter.getStateBlockCovariance(pl).diag()))

            // Measurements must be in error form
            if (abs(t % 1.0) < 0.001) {
                var pos = ins.getSolution(Time(t)).navSolution.pose.pos
                var dDown = pos[2] - alt
                var meas = mat[dDown]

                filter.update(Measurement(al,
                                          Time(t), Time(t), meas, null, mat[0.01]))
                master.giveErrorState(Time(t), filter.getStateBlockEstimate(pl),
                                      sqrt(filter.getStateBlockCovariance(pl).diag()))
            }
            if (abs(t % 10.0) < 0.0001) {
                var pos = ins.getSolution(Time(t)).navSolution.pose.pos
                var dLat = deltaLatToNorth(lat - pos[0], lat)
                var dLon = deltaLonToEast(lon - pos[1], lat)
                var dDown = pos[2] - alt
                var meas = mat[dLat + ran.nextGaussian() * 3.0,

                        dLon + ran.nextGaussian() * 3.0,
                        dDown + ran.nextGaussian() * 3.0].T
                filter.update(Measurement(gl, Time(t), Time(t), meas, null, eye(3) * 3.0))
                master.giveErrorState(Time(t), filter.getStateBlockEstimate(pl),
                                      sqrt(filter.getStateBlockCovariance(pl).diag()))
            }
            mechOut.add(ins.getSolution(Time(t)).navSolution)
            filtStateOut.add(master.getErrorStateAtTime(Time(t)).estimate)
            filtSigOut.add(master.getErrorStateAtTime(Time(t)).sigma)
            solOut.add(master.getSolution(ins.getSolution(Time(t))))

        }

        //Init output structures
        val timeOut = DoubleArray(mechOut.size)
        val timeOutNorm = DoubleArray(mechOut.size)
        val latOut = DoubleArray(mechOut.size)
        val lonOut = DoubleArray(mechOut.size)
        val altOut = DoubleArray(mechOut.size)
        val pnOut = DoubleArray(mechOut.size)
        val peOut = DoubleArray(mechOut.size)
        val pdOut = DoubleArray(mechOut.size)
        val vnOut = DoubleArray(mechOut.size)
        val veOut = DoubleArray(mechOut.size)
        val vdOut = DoubleArray(mechOut.size)
        val latSigOut = DoubleArray(mechOut.size)
        val lonSigOut = DoubleArray(mechOut.size)
        val altSigOut = DoubleArray(mechOut.size)
        val vnSigOut = DoubleArray(mechOut.size)
        val veSigOut = DoubleArray(mechOut.size)
        val vdSigOut = DoubleArray(mechOut.size)


        //Combine mechanized estimates (LLA) and state values (NED), and
        // calculate errors w.r.t. truth (LLA)
        for (i in mechOut.indices) {
            timeOut[i] = solOut.get(i).pose.time.time
            timeOutNorm[i] = timeOut[i] - timeOut[0]
            latOut[i] = solOut.get(i).pose.pos[0]
            lonOut[i] = solOut.get(i).pose.pos[1]
            altOut[i] = solOut.get(i).pose.pos[2]
            pnOut[i] = deltaLatToNorth(latOut[i] - latOut[0], latOut[0])
            peOut[i] = deltaLonToEast(lonOut[i] - lonOut[0], latOut[0])
            pdOut[i] = -(altOut[i] - altOut[0])
            vnOut[i] = solOut.get(i).vel[0]
            veOut[i] = solOut.get(i).vel[1]
            vdOut[i] = solOut.get(i).vel[2]
            latSigOut[i] = filtSigOut.get(i)[0]
            lonSigOut[i] = filtSigOut.get(i)[1]
            altSigOut[i] = filtSigOut.get(i)[2]
            vnSigOut[i] = filtSigOut.get(i)[3]
            veSigOut[i] = filtSigOut.get(i)[4]
            vdSigOut[i] = filtSigOut.get(i)[5]
        }

        figure(1)
        plot(timeOut, pnOut, "k", "North Pos")
        xlabel("Time (s)")
        ylabel("North Pos (m)")
        title("Position Solution")

        figure(2)
        plot(timeOut, peOut, "b", "East Pos")
        xlabel("Time (s)")
        ylabel("East Pos(m)")
        title("Position Solution")

        figure(3)
        plot(timeOut, pdOut, "k", "Est. Altitude")
        xlabel("Time (s)")
        ylabel("Down Pos (m)")
        title("Position Solution")
    }
}