import golem.Golem as golem
import scorpion

xhat = golem.create([1,2,3]).transpose()
P = golem.eye(3)
Phi = golem.create([[1,0,0],
                    [0,2,0],
                    [0,0,2]])
Qd = golem.eye(3)

KF = scorpion.filters.raw.SimpleKF(xhat, P)
KF.propagate(Phi, Qd)

print KF.getXhat()
print KF.getP()
