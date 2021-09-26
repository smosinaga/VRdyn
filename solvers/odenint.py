import numpy as np
from scipy.integrate import solve_ivp
import sys

g_cons = 9.81

class solution(object):
    def __repr__(self): return 'Solution_element'
    def __init__(self, modelData, tyreModel, carModel, sbModel):
        
        self.data = modelData["conf"]; self.ndof = carModel.ndof
        
        self.solveNUM( carModel, tyreModel , sbModel )
        self.solveVerticalForces( carModel, tyreModel , sbModel )
   
    def systemDef(self,t, X, carModel, tyreModel , sbModel ):
        AA = carModel.AA; BB = carModel.BB; Ainv = np.linalg.inv(AA)
        DD = np.zeros( 2*self.ndof )
        
        v0 = carModel.data["v0"]; acc = carModel.data["acc"]

        if carModel.type == "Car2DOF":
            pos = v0 * t + 0.5 * acc * t**2
            Fz1 = carModel.verticalStaticForces(X)[0]
            
            DD[1] = - tyreModel.verticalDynForce(X[1], pos, sbModel, Fz1) - Fz1
        
        elif carModel.type == "Car4DOF" or carModel.type == "Car7DOF":
            a1 = carModel.data["a1"]; a2 = carModel.data["a2"]; d = a1 + a2
            if acc == 0:
                tdiff = -d/v0
            else:
                tdiff = -v0/acc + (1/acc) * (v0**2 + 2*acc*v0*t + acc**2*t**2 - 2*acc*d)**0.5 
            
            pos1 = v0 * t + 0.5 * acc * t**2
            pos2 = v0 * (t+tdiff) + 0.5 * acc * (t+tdiff)**2
            
            if carModel.type == "Car4DOF":
                Fz1 = carModel.verticalStaticForces(X)[0]; Fz2 = carModel.verticalStaticForces(X)[1]
                
                DD[2] = - tyreModel.verticalDynForce(X[2], pos1, sbModel, Fz1) - Fz1
                DD[3] = - tyreModel.verticalDynForce(X[3], pos2, sbModel, Fz2) - Fz2
            
            elif carModel.type == "Car7DOF":
                Fz1 = carModel.verticalStaticForces(X)[0]; Fz2 = carModel.verticalStaticForces(X)[1]
                Fz3 = carModel.verticalStaticForces(X)[2]; Fz4 = carModel.verticalStaticForces(X)[3]
                
                DD[3] = - tyreModel.verticalDynForce(X[3], pos1, sbModel, Fz1) - Fz1
                DD[4] = - tyreModel.verticalDynForce(X[4], pos1, sbModel, Fz2) - Fz2
                DD[5] = - tyreModel.verticalDynForce(X[5], pos2, sbModel, Fz3) - Fz3
                DD[6] = - tyreModel.verticalDynForce(X[6], pos2, sbModel, Fz4) - Fz4
                
        else:
            print("Error: DOFs of the system are not consistent"); sys.exit()
       
        XP =  np.dot( Ainv, (DD - np.dot(BB,X) ) )  
        return XP
        
    def solveNUM(self,  carModel, tyreModel , sbModel ):
        t0 = 0; tf = self.data["t_end"]; t_step = self.data["t_step"] 
        t_span = (t0,tf); 
        y0 = np.zeros( 2 * carModel.ndof )
        method = self.data["solver"]
        t_eval = np.arange(t0, tf, t_step);
        ODE = lambda t,X: self.systemDef(t,X, carModel, tyreModel , sbModel )
        
        solu = solve_ivp(ODE, t_span, y0, method, t_eval, max_step = 10*t_step )
        
        self.sol = solu

    def solveVerticalForces(self,  carModel, tyreModel , sbModel ):
  
        #solving static forces
        self.Fvstatic = carModel.verticalStaticForces( self.sol.y )
                
        v0 = carModel.data["v0"]; acc = carModel.data["acc"]; 
        t = self.sol.t; X = self.sol.y; nEl = len(t)
        
        #solving dynamic and total forces and 
        if carModel.type == "Car2DOF":
            pos = v0 * t + 0.5 * acc * t**2
            
            Fvt1 = np.zeros(nEl)
            for idx in range(nEl):
                Fz1 = carModel.verticalStaticForces(X)[0]
                Fvt1[idx] = - tyreModel.verticalDynForce(X[1,idx], pos[idx], sbModel, Fz1)
            
            self.Fvtot = [Fvt1,]
            
        elif carModel.type == "Car4DOF" or carModel.type == "Car7DOF":
            a1 = carModel.data["a1"]; a2 = carModel.data["a2"]; d = a1 + a2
            
            if acc == 0:
                tdiff = -d/v0
            else:
                tdiff = -v0/acc + (1/acc) * (v0**2 + 2*acc*v0*t + acc**2*t**2 - 2*acc*d)**0.5 
            
            pos1 = v0 * t + 0.5 * acc * t**2
            pos2 = v0 * (t+tdiff) + 0.5 * acc * (t+tdiff)**2
            
            if carModel.type == "Car4DOF":
                Fz1 = carModel.verticalStaticForces(X)[0]; Fz2 = carModel.verticalStaticForces(X)[1]
                Fvt1 = np.zeros(nEl); Fvt2 = np.zeros(nEl);
                
                for idx in range(nEl):
                    Fvt1[idx] = - tyreModel.verticalDynForce(X[2,idx], pos1[idx], sbModel, Fz1)
                    Fvt2[idx] = - tyreModel.verticalDynForce(X[3,idx], pos2[idx], sbModel, Fz2)
                    
                self.Fvtot = [Fvt1, Fvt2]
                
            elif carModel.type == "Car7DOF":    
                Fz1 = carModel.verticalStaticForces(X)[0]; Fz2 = carModel.verticalStaticForces(X)[1]
                Fz3 = carModel.verticalStaticForces(X)[2]; Fz4 = carModel.verticalStaticForces(X)[3]
                Fvt1 = np.zeros(nEl); Fvt2 = np.zeros(nEl); Fvt3 = np.zeros(nEl); Fvt4 = np.zeros(nEl)
                for idx in range(nEl):
                    Fvt1[idx] = - tyreModel.verticalDynForce(X[3,idx], pos1[idx], sbModel, Fz1)
                    Fvt2[idx] = - tyreModel.verticalDynForce(X[4,idx], pos1[idx], sbModel, Fz2)
                    Fvt3[idx] = - tyreModel.verticalDynForce(X[5,idx], pos2[idx], sbModel, Fz3)
                    Fvt4[idx] = - tyreModel.verticalDynForce(X[6,idx], pos2[idx], sbModel, Fz4)
                
                self.Fvtot = [Fvt1, Fvt2, Fvt3, Fvt4]
            
