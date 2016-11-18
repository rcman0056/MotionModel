

import exlcm.pixhawk

import golem.*

import lcm.lcm.LCM
import lcm.lcm.LCMDataInputStream
import lcm.lcm.LCMSubscriber

import scorpion.filters.sensor.StandardSensorEKF


// Main Function
object FAIN_Main {

    @JvmStatic
    fun main(args: Array<String>) {
        var test = LCM.getSingleton()
        test.subscribe("PIXHAWK1", FAIN_Subscribe()) //This subscribes to LCM message abd is kept open as long as main is func is running


        while(true){
            //setup filter items
        }
    }
}

//Subscriber
class FAIN_Subscribe(): LCMSubscriber {

    // comments
    override fun messageReceived(p0: LCM, channel: String, p2: LCMDataInputStream){
        var pixhawk1 = (pixhawk(p2))

        var aux = MotionModelAuxData(pixhawk1.airspeed[1],...)//add rest of vars


     //println(pixhawk1.airspeed[1].toString()+ '\t' + pixhawk1.airspeed[0].toString())
        }
    }

