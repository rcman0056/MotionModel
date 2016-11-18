import golem.*
import golem.containers.*
import modules.stateblocks.ins.Pinson15AuxData
import modules.stateblocks.ins.Pinson15NEDBlock
import modules.stateblocks.ins.getImuModelHG1700
import navutils.containers.NavSolution
import navutils.containers.Pose
import navutils.containers.Vector3
import navutils.deltaLatToNorth
import navutils.deltaLonToEast
import org.knowm.xchart.BitmapEncoder

/**
 *  A standalone test of P matrix propagation (i.e. without using the scorpion framework).
 *  Useful to verify correct operation once these types are pumped into scorpion.
 */
object StandaloneCovExample {
    @JvmStatic
    fun main(args: Array<String>) {
        val maxTime = 600
        val model = getImuModelHG1700()
        val block = Pinson15NEDBlock(label = "pinson15",
                                     imuModel = model)


        val initCov = zeros(15, 15)
        initCov[0..2, 0..2] = eye(3) * .1 / 6353000.0
        initCov[3..5, 3..5] = eye(3) * .01

        val auxData = Pinson15AuxData(navSolution = NavSolution(pose = Pose(rotMat = eye(3),
                                                                            pos = Vector3(0.0, 0.0, 0.0),
                                                                            time = Time(0.0)),
                                                                vel = zeros(3, 1)),
                                      force = mat[0.0, 0.0, 9.8])

        block.receiveAuxData(auxData)
        val dt = 1.0
        val F = block.generateFPinson15(pos = auxData.navSolution.pose.pos,
                                        vel = auxData.navSolution.vel,
                                        force = auxData.force,
                                        imuModel = model,
                                        Cns = eye(3))
        val Q = block.generateQPinson15(imuModel = model)

        val Phi = expm(F * dt)
        val Qd = (Phi * Q * Phi.T + Q) * dt / 2.0
        // Alternative Qd calculation method
        //var Qd = calcVanLoan(F, eye(15,15), Q, dt)
        val output = zeros(15, maxTime)

        var P = initCov


        for (i in 1..maxTime - 1) {
            P = Phi * P * Phi.T + Qd
            for (idx in 0..15 - 1)
                output[idx, i] = P[idx, idx]
        }

        figure(1)
        val t = create(0..maxTime - 1)
        //    plot(t,output[0,0..maxTime-1], "k", "North error")
        //    plot(t,output[1,0..maxTime-1], "b", "East error")
        //    plot(t,output[2,0..maxTime-1], "g", "Down error")
        plot(t, output[0, 0..maxTime - 1].mapMat { deltaLatToNorth(it, approxLat = 0.0) }, "k", "North error")
        plot(t, output[1, 0..maxTime - 1].mapMat { deltaLonToEast(it, approxLat = 0.0) }, "b", "East error")
        xlabel("time (s)")
        ylabel("Error (m)")
        title("Position Error")

        figure(2)
        plot(t, output[3, 0..maxTime - 1], "k", "North error")
        plot(t, output[4, 0..maxTime - 1], "b", "East error")
        plot(t, output[5, 0..maxTime - 1], "g", "Down error")
        xlabel("time (s)")
        ylabel("Error (m/s)")
        title("Velocity Error")

        Thread.sleep(500)
        var c = figures[1]!!.first
        BitmapEncoder.saveBitmapWithDPI(c, "./PosErr", BitmapEncoder.BitmapFormat.PNG, 300);

        Thread.sleep(500)
        c = figures[2]!!.first
        BitmapEncoder.saveBitmapWithDPI(c, "./VelErr", BitmapEncoder.BitmapFormat.PNG, 300);
    }
}