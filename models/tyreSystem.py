from scipy import optimize
from scipy import integrate

# =============================================================================
# Parent class
# =============================================================================

class tyreSystem(object):
    def __repr__(self): return 'Tyre_Model_System'
    def __init__(self, modelData):
        self.data = modelData["tyreData"] #Data must be a dictonary
        self.patchRes = int( modelData["conf"]["patchRes"] )
    
# =============================================================================
# Subclasses - Tyre model types
# =============================================================================

class SPC(tyreSystem):
    def __repr__(self): return 'Tyre_SPC_Model_System'
    def __init__(self,modelData):
        tyreSystem.__init__(self, modelData)
        
    def verticalDynForce(self, xu, pos, sbEl, Fz):
        Kt = self.data["tyreK"]; y = sbEl.profileFun(pos)

        return Kt * (xu - y)
        
# =============================================================================   
# =============================================================================

class RRC(tyreSystem):
    def __init__(self,modelData):
        def __repr__(self): return 'Tyre_RRC_Model_System'
        tyreSystem.__init__(self, modelData)     

    def verticalDynForce(self, xu, pos, sbEl, Fz):
        Kt = self.data["tyreK"]; r0 = self.data["r0"]

        objFun = lambda X: - sbEl.profileFun(X + pos) - ( r0**2 - X**2 )**0.5  
        xPos= optimize.fminbound(objFun, -r0, r0, maxfun = self.patchRes)
        ye = - objFun(xPos) - r0
                
        return Kt * (xu - ye)

# =============================================================================   
# =============================================================================

class FRC(tyreSystem):
    def __repr__(self): return 'Tyre_FRC_Model_System'
    def __init__(self,modelData):
        tyreSystem.__init__(self, modelData)     
    
    def solveStiffDistribution(self, aLen, pp, pp0, Fz):
        Kt = self.data["tyreK"]; 
        
        if self.data["kDist"] == "uniform":
            self.kt = lambda X: Kt/(aLen)
            
        if self.data["kDist"] == "nonuniform":
            kdis = lambda X: ( 1 - (X/aLen)**6)
            kamp, err =  integrate.quad( kdis, -aLen, aLen, limit = self.patchRes )
            
            self.kt = lambda X: (Kt/kamp) * kdis(X)
            
    def verticalDynForce(self, xu, pos, sbEl, Fz):
        r0 = self.data["r0"]; 
        
        pp = lambda X: sbEl.profileFun(pos + X) + (abs( r0**2 - X**2 ))**0.5 - r0 - xu#Eq12a guo1998
        pp0 = lambda X: 1 if pp(X) >= 0 else 0  #positive part of pp0
        
        aLen, err = integrate.quad(pp0, -r0 , r0, limit = self.patchRes)
        
        if aLen < 1E-6:
            Fz = 0    
        else:
            self.solveStiffDistribution(aLen, pp, pp0, Fz)
            pp1 = lambda X: - self.kt(X) * pp(X) * pp0(X)
                        
            Fz, err = integrate.quad(pp1, -r0 , r0, limit = self.patchRes)

        return Fz
        