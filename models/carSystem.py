import numpy as np

g_cons = 9.81

# =============================================================================
# Parent class
# =============================================================================

class carSystem(object):
    def __repr__(self): return 'Car_System'
    def __init__(self, modelData):
        self.data = modelData["carData"] #Data must be a dictonary
        self.emptyMatrices() 
        self.massMatrix() #mass matrix assembly
        self.stiffMatrix() #stiffness matrix assembly
        self.dampingMatrix() #damping matrix assembly       
        self.stateSpace()
        
    def emptyMatrices(self):
        self.MM = np.zeros( [self.ndof , self.ndof] )
        self.KK = np.zeros( [self.ndof , self.ndof] )
        self.CC = np.zeros( [self.ndof , self.ndof] )
    
    def stateSpace(self):
        ZZ = np.zeros( (self.ndof, self.ndof) ) #empty matrix
        MM = self.MM
        KK = self.KK
        CC = self.CC 
        
        self.AA = np.hstack(( np.vstack((CC,MM)) , np.vstack((MM,ZZ)) ))
        self.BB = np.hstack(( np.vstack((KK,ZZ)) , np.vstack((ZZ,-MM)) ))
        
        
# =============================================================================
# Subclasses - Discrete Systems
# =============================================================================

class Car2DOF(carSystem):
    def __repr__(self): return 'Quarter_Car_Model_2DOF'
       
    def __init__(self,modelData):
        self.ndof = 2 #num degree of freedom
        carSystem.__init__(self, modelData)
        self.type = "Car2DOF"

        
    def massMatrix(self):
        ms = 0.25*self.data['carMass']
        mu = self.data['wheelMass']
        
        self.MM[0] = np.array([ ms,   0.0 ])
        self.MM[1] = np.array([ 0.0,   mu ])
        
    def stiffMatrix(self):
        ks = self.data['suspensionK'] #stiff suspension
        
        self.KK[0] = np.array([  ks,  -ks ])
        self.KK[1] = np.array([ -ks,   ks ])
    
    def dampingMatrix(self):
        cs = self.data['suspensionC'] #damping suspension
        
        self.CC[0] = np.array([ cs,   -cs ])
        self.CC[1] = np.array([ -cs,   cs ])

    def verticalStaticForces(self, X):
        ms = 0.25*self.data['carMass']; mu = self.data['wheelMass']
        
        val = (ms + mu) * g_cons
        try:
            Fvs = np.full(len (X[0]), val)  #static forces do not depend on X for this model
        except:
            Fvs = val
        return [Fvs,]

# ============================================================================= 
# =============================================================================

class Car4DOF(carSystem):
    def __repr__(self): return 'Bicycle_Car_Model_4DOF'
       
    def __init__(self,modelData):
        self.ndof = 4 #num degree of freedom
        carSystem.__init__(self, modelData)
        self.type = "Car4DOF"
        
    def massMatrix(self):
        ms = 0.5*self.data['carMass']
        m1 = self.data['wheelMass'] #Front wheel mass
        m2 = self.data['wheelMass'] #Rear wheel mass
        Iz = self.data['pitchInertia']
        
        self.MM[0] = np.array([ ms,   0.0,   0.0,   0.0 ])
        self.MM[1] = np.array([ 0.0,  Iz,    0.0,   0.0 ])
        self.MM[2] = np.array([ 0.0,  0.0,   m1,    0.0 ])
        self.MM[3] = np.array([ 0.0,  0.0,   0.0,   m2  ])
           
        
    def stiffMatrix(self):
        k1 = self.data['suspensionK'] #front suspension stiff
        k2 = self.data['suspensionK'] #rear suspension stiff
        a1 = self.data['a1']; a2 = self.data['a2']
        
        self.KK[0] = np.array([k1+k2,        a2*k2-a1*k1,        -k1,    -k2    ])
        self.KK[1] = np.array([a2*k2-a1*k1,  a1**2*k1+a2**2*k2, a1*k1,  -a2*k2 ])
        self.KK[2] = np.array([-k1,          a1*k1,              k1,     0.0    ])
        self.KK[3] = np.array([-k2,          -a2*k2,             0.0,    k2     ])
    
    def dampingMatrix(self):
        c1 = self.data['suspensionC']#front suspension damping
        c2 = self.data['suspensionC']#rear suspension damping
        a1 = self.data['a1']; a2 = self.data['a2']
        
        self.CC[0] = np.array([c1+c2,        a2*c2-a1*c1,        -c1,    -c2     ])
        self.CC[1] = np.array([a2*c2-a1*c1,  c1*a1**2+c2*a2**2,  a1*c1,  -a2*c2  ])
        self.CC[2] = np.array([-c1,          a1*c1,              c1,     0.0     ])
        self.CC[3] = np.array([-c2,          -a2*c2,             0.0,    c2      ])
    
    def verticalStaticForces(self, X):
        ms = self.data['carMass']
        m1 = self.data['wheelMass']; m2 = self.data['wheelMass']
        a1 = self.data['a1']; a2 = self.data['a2']; h = self.data['h']; ltot = a1 + a2
        acc = self.data['acc']
        
        #eq 2.121 and 2.122 from jazar + wheelMass (last term)
        Fvs1 = 0.5 * ms * g_cons * (a2/ltot) * np.cos(X[1]) \
              - 0.5 * ms * g_cons * (h/ltot) * np.sin(X[1]) \
              - 0.5 * ms * acc * (h/ltot) \
              + m1 * g_cons
              
        Fvs2 = 0.5 * ms * g_cons * (a1/ltot) * np.cos(X[1]) \
              + 0.5 * ms * g_cons * (h/ltot) * np.sin(X[1]) \
              + 0.5 * ms * acc * (h/ltot) \
              + m2 * g_cons
              
        return [Fvs1, Fvs2]
    
# =============================================================================
# =============================================================================

class Car7DOF(carSystem):
    def __repr__(self): return 'Full_Car_Model_7DOF'
       
    def __init__(self,modelData):
        self.ndof = 7 #num degree of freedom
        carSystem.__init__(self, modelData)
        self.type = "Car7DOF"
        
    def massMatrix(self):
        ms = self.data['carMass']
        mf = self.data['wheelMass'] #Front wheel mass
        mr = self.data['wheelMass'] #Rear wheel mass
        Ix = self.data['frontInertia'] #Longuitudinal mass moment
        Iz = self.data['pitchInertia'] #Lateral mass moment
        
        self.MM[0] = np.array([ ms,   0.0,   0.0,   0.0,   0.0,   0.0,   0.0  ])
        self.MM[1] = np.array([ 0.0,  Ix,    0.0,   0.0,   0.0,   0.0,   0.0  ])
        self.MM[2] = np.array([ 0.0,  0.0,   Iz,    0.0,   0.0,   0.0,   0.0  ])
        self.MM[3] = np.array([ 0.0,  0.0,   0.0,   mf,    0.0,   0.0,   0.0  ])
        self.MM[4] = np.array([ 0.0,  0.0,   0.0,   0.0,   mf,    0.0,   0.0  ])
        self.MM[5] = np.array([ 0.0,  0.0,   0.0,   0.0,   0.0,   mr,    0.0  ])
        self.MM[6] = np.array([ 0.0,  0.0,   0.0,   0.0,   0.0,   0.0,   mr   ])
           
        
    def stiffMatrix(self):
        kf = self.data['suspensionK'] #front suspension stiff
        kr = self.data['suspensionK'] #rear suspension stiff
        a1 = self.data['a1']; a2 = self.data['a2']
        b1 = self.data['b1']; b2 = self.data['b2']
        kRf = self.data['antiRollBarK']; kRr = self.data['antiRollBarK']
        
        w = b1 + b2; kR = kRf
        k11 = 2*kf + 2*kr; k21 = k12 = b1*kf - b2*kf - b1*kr + b2*kr
        k31 = k13 = 2*a2*kr - 2*a1*kf
        k22 = kR + b1**2*kf + b2**2*kf + b1**2*kr + b2**2*kr
        k32 = k23 = a1*b2*kf - a1*b1*kf - a2*b1*kr + a2*b2*kr
        k42 = k24 = -b1*kf - kR/w
        k52 = k25 = b2*kf + kR/w
        k33 = 2*kf*a1**2 + 2*kr*a2**2
        k44 = kf + kR/w**2
        k55 = kf + kR/w**2
        
        self.KK[0] = np.array([k11,  k12,     k13,     -kf,       -kf,       -kr,     -kr    ])
        self.KK[1] = np.array([k21,  k22,     k23,     k24,       k25,       b1*kr,   -b2*kr ])
        self.KK[2] = np.array([k31,  k32,     k33,     a1*kf,     a1*kf,     -a2*kr,  -a2*kr ])
        self.KK[3] = np.array([-kf,  k42,     a1*kf,   k44,       -kR/w**2,  0,       0      ])
        self.KK[4] = np.array([-kf,  k52,     a1*kf,   -kR/w**2,  k55,       0,       0      ])
        self.KK[5] = np.array([-kr,  b1*kr,   -a2*kr,  0,         0,         kr,      0      ])
        self.KK[6] = np.array([-kr,  -b2*kr,  -a2*kr,  0,         0,         0,       kr     ])
        
        
    def dampingMatrix(self):
        cf = self.data['suspensionC']#front suspension damping
        cr = self.data['suspensionC']#rear suspension damping
        a1 = self.data['a1']; a2 = self.data['a2']
        b1 = self.data['b1']; b2 = self.data['b2']
        
        c11 = 2*cf + 2*cr
        c21 = c12 = b1*cf - b2*cf - b1*cr + b2*cr
        c31 = c13 = 2*a2*cr - 2*a1*cf
        c22 = b1**2*cf + b2**2*cf + b1**2*cr + b2**2*cr
        c32 = c23 = a1*b1*cf - a1*b1*cf - a2*b1*cr + a2*b2*cr
        c33 = 2*cf*a1**2 + 2*cr*a2**2
        
        self.CC[0] = np.array([c11,  c12,     c13,     -cf,    -cf,    -cr,     -cr    ])
        self.CC[1] = np.array([c21,  c22,     c23,     -b1*cf, b2*cf,  b1*cr,   -b2*cr ])
        self.CC[2] = np.array([c31,  c32,     c33,     a1*cf,  a1*cf,  -a2*cr,  -a2*cr ])
        self.CC[3] = np.array([-cf,  -b1*cf,  a1*cf,   cf,     0,      0,       0      ])
        self.CC[4] = np.array([-cf,  b2*cf,   a1*cf,   0,      cf,     0,       0      ])
        self.CC[5] = np.array([-cr,  b1*cr,   -a2*cr,  0,      0,      cr,      0      ])
        self.CC[6] = np.array([-cr,  -b2*cr,  -a2*cr,  0,      0,      0,       cr     ])
    
    def verticalStaticForces(self, X):
        ms = self.data['carMass']
        m1 = self.data['wheelMass']; m2 = self.data['wheelMass']
        a1 = self.data['a1']; a2 = self.data['a2']; h = self.data['h']; ltot = a1 + a2
        acc = self.data['acc']
        
        #eq 2.121 and 2.122 from jazar + wheelMass (last term)
        FvsF1 = 0.5 * ms * g_cons * (a2/ltot) * np.cos(X[2]) \
              - 0.5 * ms * g_cons * (h/ltot) * np.sin(X[2]) \
              - 0.5 * ms * acc * (h/ltot) \
              + m1 * g_cons
      
        FvsF2 = 0.5 * ms * g_cons * (a2/ltot) * np.cos(X[2]) \
              - 0.5 * ms * g_cons * (h/ltot) * np.sin(X[2]) \
              - 0.5 * ms * acc * (h/ltot) \
              + m1 * g_cons
              
        FvsR1 = 0.5 * ms * g_cons * (a1/ltot) * np.cos(X[2]) \
              + 0.5 * ms * g_cons * (h/ltot) * np.sin(X[2]) \
              + 0.5 * ms * acc * (h/ltot) \
              + m2 * g_cons
            
        FvsR2 = 0.5 * ms * g_cons * (a1/ltot) * np.cos(X[2]) \
              + 0.5 * ms * g_cons * (h/ltot) * np.sin(X[2]) \
              + 0.5 * ms * acc * (h/ltot) \
              + m2 * g_cons
              
        return [FvsF1, FvsF2, FvsR1, FvsR2]