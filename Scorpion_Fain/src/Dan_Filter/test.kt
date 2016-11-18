import com.jcg.DATADATE
import com.jcg.RECEIVER_TRUTH_FILE
import golem.*
import navutils.rpyToDcm
import java.io.File

/**
 * Created by ucav on 8/22/2016.
 */



fun main(args: Array<String>){
  var DATADATE="0927"
  val LEADER_TRUTH_FILE ="../Kotlin/data/Truth/BlueMax/Receiver_".plus(DATADATE).plus("_udo.tdf")

  val truthListR=readTdf(RECEIVER_TRUTH_FILE,"\t",true)
  var truthR=convertArrayListToArray(truthListR)
 /* var dtr=Math.PI/180
  var TankerEst=mat[0,3,0].T*dtr
  var ReceiverEst=mat[0, -2.5, 0].T*dtr
  var Rpy=mat[0,-6,0].T*dtr
  var PRpy=eye(3)*.1*pow(dtr,2)

  var Cbn_T=rpyToDcm(TankerEst)
  var Cbn_R=rpyToDcm(ReceiverEst)
    var test=StereoRpyWithCovToTilt(Cbn_T,Cbn_R,Rpy,PRpy)

    var stop=test.tilt_vec/dtr
    var stop2=0*/

}