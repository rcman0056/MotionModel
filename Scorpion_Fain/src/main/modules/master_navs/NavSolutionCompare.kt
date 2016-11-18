package modules.master_navs

import navutils.containers.NavSolution
import java.util.*

/**
 * Comparator instantiation to be used when sorting or searching
 * NavSolution objects based on time.
 */
class NavSolutionCompare() : Comparator<NavSolution> {
    /**
     * Function that compares 2 instantiations of this class based on time.
     * Allows use of sort() and binarySearch() methods.
     *
     * @param check: NavSolution
     * @param target: NavSolution
     * @return -1 if check is younger than target, 1 if it is older, and 0 if
     * they are the same age.
     */
    override fun compare(check: NavSolution, target: NavSolution): Int {
        if (check.pose.time.time < target.pose.time.time) {
            return -1
        } else if (check.pose.time.time > target.pose.time.time) {
            return 1
        } else {
            return 0
        }
    }
}