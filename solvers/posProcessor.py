import matplotlib.pyplot as plt
import numpy as np

plt.rcParams['font.family'] = 'serif'

def plotDisp(solEl):
    #solEl is a solution element
    xdata = solEl.sol.t; ydata = solEl.sol.y
    
    if solEl.ndof == 7:
    # =============================================================================
    # Displacements - 7DOF
    # =============================================================================
        plt.figure()
        plt.suptitle("Displacements", fontsize=14)
        
        plt.subplot(4,2,(1,2))
        plt.title("Body mass")
        plt.plot(xdata,ydata[0])
        plt.ylabel("Displacement(m)")
        plt.xlabel("Time(s)")
        plt.grid(True)
    
        plt.subplot(4,2,3)
        plt.title("Body mass")
        plt.plot(xdata,ydata[1])
        plt.ylabel("Rotation(rad)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.subplot(4,2,4)
        plt.title("Body mass")
        plt.plot(xdata,ydata[2])
        plt.ylabel("Rotation(rad)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.subplot(4,2,5)
        plt.title("Front wheel")
        plt.plot(xdata,ydata[3])
        plt.ylabel("Displacement(m)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.subplot(4,2,6)
        plt.title("Front wheel")
        plt.plot(xdata,ydata[4])
        plt.ylabel("Displacement(m)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.subplot(4,2,7)
        plt.title("Rear wheel")
        plt.plot(xdata,ydata[5])
        plt.ylabel("Displacement(m)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.subplot(4,2,8)
        plt.title("Rear wheel")
        plt.plot(xdata,ydata[6])
        plt.ylabel("Displacement(m)")
        plt.xlabel("Time(s)")
        plt.grid(True)
                
        plt.get_current_fig_manager().window.showMaximized()
        plt.tight_layout(pad=0.8, w_pad=-1.0, h_pad=-2.0); plt.show()
                    
    if solEl.ndof == 4:
    # =============================================================================
    # Displacements - 4DOF
    # =============================================================================
        plt.figure()
        plt.suptitle("Displacements", fontsize=14)
        
        plt.subplot(2,2,1)
        plt.title("Body mass")
        plt.plot(xdata,ydata[0])
        plt.ylabel("Displacement(m)")
        plt.xlabel("Time(s)")
        plt.grid(True)
    
        plt.subplot(2,2,2)
        plt.title("Body mass")
        plt.plot(xdata,ydata[1])
        plt.ylabel("Rotation(rad)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.subplot(2,2,3)
        plt.title("Front wheel")
        plt.plot(xdata,ydata[2])
        plt.ylabel("Displacement(m)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.subplot(2,2,4)
        plt.title("Rear wheel")
        plt.plot(xdata,ydata[3])
        plt.ylabel("Displacement(m)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.get_current_fig_manager().window.showMaximized()
        plt.tight_layout(); plt.show()
                    
    if solEl.ndof == 2:
    # =============================================================================
    # Displacements - 2DOF
    # =============================================================================
        plt.figure() 
        plt.suptitle("Displacements", fontsize=14)
        
        plt.subplot(2,1,1)
        plt.title("Body mass")
        plt.plot(xdata,ydata[0])
        plt.ylabel("Displacement(m)")
        plt.xlabel("Time(s)")
        plt.grid(True)
    
        plt.subplot(2,1,2)
        plt.title("Wheel mass")
        plt.plot(xdata,ydata[1])
        plt.ylabel("Displacement(m)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.get_current_fig_manager().window.showMaximized()
        plt.tight_layout(); plt.show()
        

def plotVelo(solEl):
    #solEl is a solution element
    xdata = solEl.sol.t; ydata = solEl.sol.y
    
    if solEl.ndof == 7:            
    # =============================================================================
    # Velocity - 7DOF
    # =============================================================================
        plt.figure()
        plt.suptitle("Velocities", fontsize=14)
    
        plt.subplot(4,2,(1,2))
        plt.title("Body mass")
        plt.plot(xdata,ydata[7])
        plt.ylabel("Velocity (m/s)")
        plt.xlabel("Time(s)")
        plt.grid(True)
    
        plt.subplot(4,2,3)
        plt.title("Body mass")
        plt.plot(xdata,ydata[8])
        plt.ylabel("Angular velocity(rad/s)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.subplot(4,2,4)
        plt.title("Body mass")
        plt.plot(xdata,ydata[9])
        plt.ylabel("Angular velocity(rad/s)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.subplot(4,2,5)
        plt.title("Front wheel")
        plt.plot(xdata,ydata[10])
        plt.ylabel("Velocity (m/s)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.subplot(4,2,6)
        plt.title("Front wheel")
        plt.plot(xdata,ydata[11])
        plt.ylabel("Velocity (m/s)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.subplot(4,2,7)
        plt.title("Rear wheel")
        plt.plot(xdata,ydata[12])
        plt.ylabel("Velocity (m/s)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.subplot(4,2,8)
        plt.title("Rear wheel")
        plt.plot(xdata,ydata[13])
        plt.ylabel("Velocity (m/s)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.get_current_fig_manager().window.showMaximized()
        plt.tight_layout(pad=0.8, w_pad=-1.0, h_pad=-3.0); plt.show()
        
        
    if solEl.ndof == 4:            
    # =============================================================================
    # Velocity - 4DOF
    # =============================================================================
        plt.figure()
        plt.suptitle("Velocities", fontsize=14)
    
        plt.subplot(2,2,1)
        plt.title("Body mass")
        plt.plot(xdata,ydata[4])
        plt.ylabel("Velocity (m/s)")
        plt.xlabel("Time(s)")
        plt.grid(True)
    
        plt.subplot(2,2,2)
        plt.title("Body mass")
        plt.plot(xdata,ydata[5])
        plt.ylabel("Angular velocity(rad/s)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.subplot(2,2,3)
        plt.title("Front wheel")
        plt.plot(xdata,ydata[6])
        plt.ylabel("Velocity (m/s)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.subplot(2,2,4)
        plt.title("Rear wheel")
        plt.plot(xdata,ydata[7])
        plt.tight_layout()
        plt.ylabel("Velocity (m/s)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.get_current_fig_manager().window.showMaximized()
        plt.tight_layout(); plt.show()
        
    if solEl.ndof == 2:            
    # =============================================================================
    # Velocity - 2DOF
    # =============================================================================
        plt.figure()
        plt.suptitle("Velocities", fontsize=14)
    
        plt.subplot(2,1,1)
        plt.title("Body mass")
        plt.plot(xdata,ydata[2])
        plt.ylabel("Velocity (m/s)")
        plt.xlabel("Time(s)")
        plt.grid(True)
    
        plt.subplot(2,1,2)
        plt.title("Wheel mass")
        plt.plot(xdata,ydata[3])
        plt.ylabel("Velocity (m/s)")
        plt.xlabel("Time(s)")
        plt.grid(True)
        
        plt.get_current_fig_manager().window.showMaximized()
        plt.tight_layout(); plt.show()

        
def plotProfile(data, sbEl):    
    tf = data["conf"]["t_end"]; 
    v0 = data["carData"]["v0"]; acc = data["carData"]["acc"];
    xmax = v0*tf + 0.5*acc*tf**2;
    
    xsample = np.linspace(0,xmax, 10000);
    ydata = np.zeros( len(xsample) ); 
    for idx,i in enumerate(xsample):
        ydata[idx] = sbEl.profileFun(i)
    
    plt.figure()
    plt.title("Profile road with speed bump")
    plt.plot(xsample,ydata, label = "Profile")
    plt.ylim((0,1))
    plt.ylabel("Road profile (m)")
    plt.xlabel("Road length(m)")
    plt.legend()
    plt.grid(True)
    
    plt.get_current_fig_manager().window.showMaximized()
    plt.tight_layout(); plt.show()
    
def plotForces(solEl):
    #solEl is a solution element
    
    xcor = solEl.sol.t ; Fvstatic = solEl.Fvstatic; Fvtot = solEl.Fvtot; #Fvdyn = solEl.Fvdyn
    nEl = len(Fvtot)
    plt.figure()
    colors = plt.cm.rainbow(np.linspace(0,1,nEl))
    
    for idx in range(nEl):
        plt.plot(xcor,Fvstatic[idx],"--", c=colors[idx], linewidth = 0.75,  label = "Wheel#"+str(idx+1)+ "  - Static")
        plt.plot(xcor,Fvtot[idx], c=colors[idx], linewidth = 1.25, label = "Wheel#"+str(idx+1)+ " - Total")
    
    plt.legend()
    plt.grid(True)
    
    plt.get_current_fig_manager().window.showMaximized()
    plt.tight_layout(); plt.show()
    
    
def exportResults(fileName, solEl):
    #solEl is a solution element
    expMat = np.vstack(( solEl.sol.t,solEl.sol.y, solEl.Fvstatic, solEl.Fvtot)).T
    np.savetxt("output/" + fileName + ".txt", expMat)
