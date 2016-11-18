/* Requires python deps
package otherlang.embedpython

import org.python.util.PythonInterpreter

fun maina(args: Array<String>)
{
    println("Starting embedded python")
    var interp = PythonInterpreter()
    interp.execfile("C:/msys64/home/kyon/projects/science/scorpion/src/scorpion.examples.main/python/example.py")

}
*/