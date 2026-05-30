#kinematics visualiser
import matplotlib.pyplot as plt
import numpy as np

from matplotlib.widgets import Button, Slider

#input values
L1 = float(input("Enter the length of the first link: "))
L2 = float(input("Enter the length of the second link: "))
theta1 = float(input("Enter the angle of the first link (in degrees): "))
theta2 = float(input("Enter the angle of the second link (in degrees): ")) 

L1m = float(input("L1 mass: ")) 
L2m = float(input("L2 mass: ")) 
  
M1m = float(input("M1 mass: ")) 
M2m = float(input("M2 mass: "))  

t1 = np.radians(theta1)
t2 = np.radians(theta2)

x0, y0 = 0, 0

x1 = L1 * np.cos(t1)
y1 = L1 * np.sin(t1)

x2 = L1 * np.cos(t1) + L2 * np.cos(t2)
y2 = L1 * np.sin(t1) + L2 * np.sin(t2)

fig, ax = plt.subplots()

ax.set_aspect("equal")
ax.set_xlim(-0.25, 0.75)
ax.set_ylim(-0.25, 0.75)

shoulder, =ax.plot(x0, y0, "ko", markersize = 4, label = "Shoulder (0, 0)")
elbow, = ax.plot(x1, y1, "go", markersize = 4, label = f"Elbow ({x1:.5f}, {y1:.5f})")
wrist, = ax.plot(x2, y2, "ro", markersize = 4, label = f"Wrist ({x2:.5f}, {y2:.5f})")

line1, = ax.plot([x0, x1], [y0, y1])
line2, = ax.plot([x1, x2], [y1, y2])

fig.subplots_adjust(left = 0.5, bottom = 0.5)

axt = fig.add_axes([0.5, 0.1, 0.35, 0.03]) 
angle_slider1 = Slider( 
    ax = axt, 
    label = "angle 1", 
    valmin = np.radians(0), 
    valmax = np.radians(180), 
    valinit = t1, 
) 

axL = fig.add_axes([0.1, 0.3, 0.03, 0.5]) 
length_slider1 = Slider( 
    ax = axL, 
    label = "length 1", 
    valmin = 0, 
    valmax = 1, 
    valinit = L1,
    orientation = "vertical"
) 

axt = fig.add_axes([0.5, 0.2, 0.35, 0.03]) 
angle_slider2 = Slider( 
    ax = axt, 
    label = "angle 2", 
    valmin = np.radians(0), 
    valmax = np.radians(180), 
    valinit = t2, 
) 

axL = fig.add_axes([0.2, 0.3, 0.03, 0.5]) 
length_slider2 = Slider( 
    ax = axL, 
    label = "length 2", 
    valmin = 0,
    valmax = 1, 
    valinit = L2,
    orientation = "vertical"
) 

torque_text = ax.text(0.05, 0.95, "", transform = ax.transAxes, verticalalignment = "top")

def update(val):
    new_x1 = length_slider1.val * np.cos(angle_slider1.val)
    new_x2 = length_slider2.val * np.cos(angle_slider2.val) + new_x1
    new_y1 = length_slider1.val * np.sin(angle_slider1.val)
    new_y2 = length_slider2.val * np.sin(angle_slider2.val) + new_y1
    elbow.set_data([new_x1], [new_y1])
    wrist.set_data([new_x2], [new_y2])
    line1.set_data([x0, new_x1], [y0, new_y1])
    line2.set_data([new_x1, new_x2], [new_y1, new_y2])

    t_rel1 = angle_slider1.val
    t_rel2 = angle_slider2.val - angle_slider1.val

    L1CoM, L2CoM = length_slider1.val/2, length_slider2.val/2 
  
    L1CoMtot = L1CoM
    L2CoMtot = length_slider1.val + L2CoM
  
    g = 9.81 
  
    #Forces 
    Fl1 = L1m * g 
    Fm1 = M1m * g
    Fl2 = L2m * g
    Fm2 = M2m * g
  
    #Torque worst case (fully horizontal) 
    TtotS = round((Fl1 * L1CoMtot) + (Fm1 * length_slider1.val) + (Fl2 * L2CoMtot) + (Fm2 * length_slider1.val + length_slider2.val), 5)
    TtotE = round((Fl2 * L2CoMtot) + (Fm2 * (length_slider1.val + length_slider2.val)), 5)
  
    #Forward kinematics 
    cos1 = np.cos(t_rel1) 
    cos2 = np.cos(t_rel1 + t_rel2)
  
    Xjoint2 = round(length_slider1.val * cos1, 5) 
    Xjoint3 = round(Xjoint2 + length_slider2.val * cos2, 5)  
  
    XCoM1 = round(L1CoM * cos1, 5) 
    XCoM2 = round(Xjoint2 + L2CoM * cos2, 5)

    #Gravity torque at current position 
    Tgravity = round(Fl1 * XCoM1 + Fm1 * Xjoint2 + Fl2 * XCoM2 + Fm2 * Xjoint3, 5)
    TgravityE = round(Fl2 * XCoM2 + Fm2 * Xjoint3, 5)  
 
    sin1 = np.sin(t_rel1) 
    sin2 = np.sin(t_rel1 + t_rel2) 
  
    X_cord1 = 0 
    X_cord2 = Xjoint2 
    X_cord3 = Xjoint3  
  
    Y_cord1 = 0 
    Y_cord2 = round(length_slider1.val * sin1, 5) 
    Y_cord3 = round(Y_cord2 + length_slider2.val * sin2, 5) 
  
    #Moment of inertia (worst case - fully extended horizontal) 
    I_L1 = (1/3) * L1m * length_slider1.val**2 
    I_M1 = M1m * length_slider1.val**2 
    I_L2 = (1/3) * L2m * (length_slider1.val + length_slider2.val)**2 
    I_M2 = M2m * (length_slider1.val + length_slider2.val)**2
  
    I_totalS = I_L1 + I_M1 + I_L2 + I_M2 
    I_totalE = I_L2 + I_M2 
  
    #Motor torque (worst case) 
    acceleration = 3 
 
    TmoS = TtotS + (I_totalS * acceleration)
    TmoE = TtotE + (I_totalE * acceleration)

    torque_text.set_text(
        f"Shoulder motor torque (worst case): {TmoS:.5f} Nm\n"
        f"Elbow motor torque (worst case): {TmoE:.5f} Nm\n"
        f"Shoulder gravity torque (current position): {Tgravity:.5f} Nm\n"
        f"Elbow gravity torque (current position): {TgravityE:.5f} Nm\n"
    )
    fig.canvas.draw_idle()


angle_slider1.on_changed(update)
length_slider1.on_changed(update)

angle_slider2.on_changed(update)
length_slider2.on_changed(update)

#reset button
resetax1 = fig.add_axes([0.05, 0.1, 0.1, 0.04]) 
button1 = Button(resetax1, "reset lengths", hovercolor = "0.975")

resetax2 = fig.add_axes([0.15, 0.1, 0.1, 0.04]) 
button2 = Button(resetax2, "reset angles", hovercolor = "0.975")

def reset_lengths(event):
    length_slider1.reset()  
    length_slider2.reset()
button1.on_clicked(reset_lengths) 

def reset_angles(event):
    angle_slider1.reset()  
    angle_slider2.reset()
button2.on_clicked(reset_angles)

plt.show()