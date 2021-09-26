# =============================================================================
# Parent class
# =============================================================================

class sbSystem(object):
    def __repr__(self): return 'Speed_Bump_System'
    def __init__(self, modelData):
        self.data = modelData["sbData"] #Data must be a dictonary
    
    def profileFun(self,x):
        raise NotImplementedError
    

# =============================================================================
# Subclasses - Speed Bump types
# =============================================================================

class perfectStep(sbSystem):
    def __init__(self,modelData):
        sbSystem.__init__(self, modelData)
        
    def profileFun(self, x):
        L = self.data["L"]; h = self.data["h"]; xshift = self.data["xshift"];
        
        if x < xshift:
            ycor = 0
        elif x <= L + xshift:
            ycor = h
        else:
            ycor = 0 
        return ycor
    
class trapezoidal(sbSystem):
    def __init__(self,modelData):
        sbSystem.__init__(self, modelData)
        
    def profileFun(self, x): 
        L1 = self.data["L1"]; L2 = self.data["L2"]; L3 = self.data["L3"]
        h = self.data["h"]; xshift = self.data["xshift"];
       
        if x < xshift:
            ycor = 0
        elif x <= L1 + xshift:
            ycor = (h/L1) * (x - xshift)
        elif x > L1 and x <= L1 + L2 + xshift:
            ycor = h
        elif x <= L1 + L2 + L3 + xshift:
            ycor = -(h/L3)*(x - xshift) + h*(L1+L2+L3)/L3
        else:
            ycor = 0 
        return ycor
    
class convex(sbSystem):
    def __init__(self,modelData):
        sbSystem.__init__(self, modelData)
        
    def profileFun(self, x):
        r = self.data["r"]; h = self.data["h"]; xshift = self.data["xshift"]; 

        x0 = (2*r*h - h**2) **0.5
        
        if x < xshift:
            ycor = 0
        elif x >= xshift and x <= 2*x0 + xshift:
            ycor = ( r**2 - (x-x0-xshift)**2 )**0.5 - r + h
        else:
            ycor = 0 
        return ycor
    
class triangle(sbSystem):
    def __init__(self,modelData):
        sbSystem.__init__(self, modelData)
        
    def profileFun(self, x): 
        L1 = self.data["L1"]; L2 = self.data["L2"]; 
        h = self.data["h"]; xshift = self.data["xshift"]; 
        
        if x < xshift:
            ycor = 0
        if x >= xshift and x < L1 + xshift:
            ycor = (h/L1) * (x - xshift)
        elif x >= L1 + xshift and x < L1 + L2 + xshift:
            ycor = -(h/L2)*(x - xshift) + h*(L1/L2) + h           
        else:
            ycor = 0 
        return ycor
    
class ramp(sbSystem):
    def __init__(self,modelData):
        sbSystem.__init__(self, modelData)
        
    def profileFun(self, x):
        L = self.data["L"]; h = self.data["h"]; xshift = self.data["xshift"]; 
        
        if x < xshift:
            ycor = 0
        if x >= xshift and x < L + xshift:
            ycor = (h/L) * (x - xshift)  
        else:
            ycor = 0 
        return ycor
