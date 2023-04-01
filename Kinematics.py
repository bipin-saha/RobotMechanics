import math

link_1 = 95         #mm
link_2 = 101.79     #mm
px = 196.79            #mm
py = 0             #mm


def forwardKinematics(link_1, link_2, theta_1, theta_2):
    px = link_1*math.cos(theta_1) + link_2*math.cos(theta_1+theta_2)
    py = link_1*math.sin(theta_1) + link_2*math.sin(theta_1+theta_2)
    return px, py

def inverseKinematics(link_1, link_2, px, py):
    theta2 = math.acos((px*px + py*py - link_1*link_1 - link_2*link_2)/(2*link_1*link_2))
    theta1 = math.atan2(py,px) - math.atan2((link_2*math.sin(theta2)),(link_1+link_2*math.cos(theta2)))

    theta1 = math.degrees(theta1)
    theta2 = math.degrees(theta2)
    """

    print(math.degrees(theta1))
    print(math.degrees(theta2))

    if theta1<0 and theta1<=90:
        theta1 = -1*theta1
    
    if theta1>=math.pi/2 or theta2>=math.pi/2:
        theta1 = theta1
        theta2 = theta2
    else:
        theta1 = -1*(theta1-math.pi/2)
        theta2 = -1*(theta2-math.pi/2)
    """

    #print(theta1, theta2)

    if theta2 >=-90 and theta2 <=90:
        theta1 = (theta1 + 90)

    #print(theta2)

    """
    if theta2 >=-90 and theta2 <=90:
        theta2 = (theta1 + 90)
    print(theta1, theta2)
    """
    

    return theta1, theta2


def motionPlanning():
    pass


"""
t1, t2 = inverseKinematics(link_1=link_1, link_2=link_2, px=100, py = 50)
print(math.degrees(t1), math.degrees(t2))
pxx, pyy = forwardKinematics(link_1=link_1, link_2=link_2, theta_1=t1, theta_2=t2)
print(pxx,pyy)
"""