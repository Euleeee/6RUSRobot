import math as m
import numpy as np
import time

def square(halfSideLength = 30, robotHeight = -90, n = 2):
    """Calculates coordinates for a square
        `halfSideLength`: half length of the edge
        `robotHeight`: z-Coordinate for 2D model (have to be a negative value)
        `n`: Number of rotations
        `retrun`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for linear moving  
        """
#  _______ 
# |       |
# |       |
# |_______|
#  
# | a | 
# a = halfSideLength
    i = range(n)
    posSquare = []
    for num in i:

        posSquare.append([halfSideLength,halfSideLength,robotHeight,0,0,0,'mov'])
        posSquare.append([-halfSideLength,halfSideLength,robotHeight,0,0,0,'lin'])
        posSquare.append([-halfSideLength,-halfSideLength,robotHeight,0,0,0,'lin'])
        posSquare.append([halfSideLength,-halfSideLength,robotHeight,0,0,0,'lin'])  

    posSquare.append([halfSideLength,halfSideLength,robotHeight,0,0,0,'lin'])
    posSquare.append([0,0,-127,0,0,0,'mov'])

    return posSquare


def triangle(halfSideLength = 15, robotHeight = -90, n = 2):
    """Calculates coordinates for a samesided triangle
        `halfSideLength`: half sidelength of the triangle
        `robotHeight`: z-Coordinate for 2D model (have to be a negative value)
        `retrun`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for linear moving  
        """
#     ^ 
#    / \ 
#   /   \  
#  /     \   
# /_______\
#  
# | a | 
# a = halfSideLength
    
    hHalf = (halfSideLength * m.sqrt(3)/2)/2
    posTriangle = []

    i = range(n)
    for num in i:
        
            posTriangle.append([-hHalf,halfSideLength,robotHeight,0,0,0,'mov'])
            posTriangle.append([-hHalf,-halfSideLength,robotHeight,0,0,0,'lin'])
            posTriangle.append([hHalf,0,robotHeight,0,0,0,'lin'])
            

    posTriangle.append([-hHalf,halfSideLength,robotHeight,0,0,0,'lin'])
    posTriangle.append([0,0,-127,0,0,0,'mov'])
    return posTriangle


def circle(radius = 15, resolution = 20, robotHeight = -90, n = 1, dir = 0):
    """Calculates coordinates for a 2D-circle
        `radius`: Radius of the circle
        `resolution`: Number of circlepoints
        `robotHeight`: z-Coordinate for 2D model (have to be a negative value)
        `n`: Number of rotations
        `dir`: Direction of the circle 
        `retrun`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for linear moving  
        """
    
    t = np.linspace(0, n*2*m.pi, resolution*n)
    circlePos = []
    for num in t:
        if dir == 0:
            x = m.cos(num)*radius
            y = m.sin(num)*radius
        else:
            x = m.cos(num)*radius
            y = m.sin(num-m.pi)*radius

        circlePos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

    circlePos.append([0,0,-127,0,0,0,'mov'])
    return circlePos


def eight(radius = 15, resolution = 20, robotHeight = -90, n = 1):
    """Calculates coordinates for a 2D-eight
        `radius`: Radius of the circle
        `resolution`: Number of circlepoints
        `robotHeight`: z-Coordinate for 2D model (have to be a negative value)
        `n`: Number of rotations
        `retrun`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for linear moving  
        """
    
    t = np.linspace(0, n*2*m.pi, resolution*n)
    eightPos = []
    for num in t:
            x = -m.sin(num)*radius
            y = m.cos(num)*radius - radius
            eightPos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

    eightPos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

    for num in t:
            x = -m.sin(num)*radius
            y = -m.cos(num)*radius + radius
            eightPos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

    eightPos.append([0,0,-127,0,0,0,'mov'])
    return eightPos


def pyramide(halfSideLength = 15, robotHeight = -90):
    """Calculates coordinates for a tetrahedron
        `halfSideLength`: half sidelength of the tetrahedron
        `robotHeight`: z-Coordinate for 2D Base (have to be a negative value)
        `retrun`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for linear moving  
        """

    hBaseHalf = (halfSideLength * m.sqrt(3)/2)/2
    hTetra = m.sqrt(6)*halfSideLength/3
    pyramidePos = [

            [0,0,robotHeight+hTetra,0,0,0,'mov'],
            [-hBaseHalf,-halfSideLength,robotHeight,0,0,0,'lin'],
            [hBaseHalf,0,robotHeight,0,0,0,'lin'],
            [0,0,robotHeight+hTetra,0,0,0,'mov'],

            [hBaseHalf,0,robotHeight,0,0,0,'lin'],
            [-hBaseHalf,halfSideLength,robotHeight,0,0,0,'lin'],
            [0,0,robotHeight+hTetra,0,0,0,'mov'],

            [-hBaseHalf,halfSideLength,robotHeight,0,0,0,'lin'],
            [-hBaseHalf,-halfSideLength,robotHeight,0,0,0,'lin'],
            [0,0,robotHeight+hTetra,0,0,0,'mov'],

            [0,0,-127,0,0,0,'mov']
        ]
        
    return pyramidePos


def pickPlace(distx = 10,disty = 10, midDist = 20,defaultHeight= -70,linHeight = 10, robotHeight = -90):
    """Calculates coordinates for a 3x2 palette
        `distx`: Distance between the palette places in x direction
        `disty`: Distance between the palette places in y direction
        `midDist`: Distance between mid and palette places 
        `defaulttHeight`: z-Coordinate for upper position of pick and place (have to be a negative value)
        `linHeight`: Linear distance to pick up/ place a piece 
        `robotHeight`: z-Coordinate for 2D Base (have to be a negative value)
        `retrun`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for linear moving  
        """
    pickPlacePos = []
    yCount = [0, 1]
    xCount = [-1,0,1]

    for numx in xCount:
            for numy in yCount:
                pickPlacePos.append([numx*distx, numy*disty-midDist, robotHeight+linHeight,0,0,0,'lin'])
                pickPlacePos.append([numx*distx, numy*disty-midDist, robotHeight,0,0,0,'mov'])
                pickPlacePos.append([numx*distx, numy*disty-midDist, robotHeight+linHeight,0,0,0,'lin'])

                pickPlacePos.append([numx*distx, 0, defaultHeight,0,0,0,'mov'])

                pickPlacePos.append([numx*distx, numy*disty+midDist, robotHeight+linHeight,0,0,0,'lin'])
                pickPlacePos.append([numx*distx, numy*disty+midDist, robotHeight,0,0,0,'mov'])
                pickPlacePos.append([numx*distx, numy*disty+midDist, robotHeight+linHeight,0,0,0,'lin'])

                pickPlacePos.append([numx*distx, 0, defaultHeight,0,0,0,'mov'])
                

    pickPlacePos.append([0,0,-127,0,0,0,'mov'])
    return pickPlacePos


def rectangleSignal(flankHeight = 30, flankWidth = 10, robotHeight = -90):
    """Calculates coordinates for rectangle Signal
    `flankHeight`: Flank height
    `flankWidth`: Flank width
    `robotHeight`: z-Coordinate for 2D Base (have to be a negative value)
    `retrun`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for linear moving  
    """
    rectanglePos = [
        [flankHeight/2,-2.5*flankWidth, robotHeight,0,0,0,'mov'],

        [-flankHeight/2,-2.5*flankWidth, robotHeight,0,0,0,'lin'],
        [-flankHeight/2,-1.5*flankWidth, robotHeight,0,0,0,'lin'],

        [flankHeight/2,-1.5*flankWidth, robotHeight,0,0,0,'lin'],
        [flankHeight/2,-0.5*flankWidth, robotHeight,0,0,0,'lin'],

        [-flankHeight/2,-0.5*flankWidth, robotHeight,0,0,0,'lin'],
        [-flankHeight/2,0.5*flankWidth, robotHeight,0,0,0,'lin'],

        [flankHeight/2,0.5*flankWidth, robotHeight,0,0,0,'lin'],
        [flankHeight/2,1.5*flankWidth, robotHeight,0,0,0,'lin'],

        [-flankHeight/2,1.5*flankWidth, robotHeight,0,0,0,'lin'],
        [-flankHeight/2,2.5*flankWidth, robotHeight,0,0,0,'lin'],

        [flankHeight/2,2.5*flankWidth, robotHeight,0,0,0,'mov']
    ]

    rectanglePos.append([0,0,-127,0,0,0,'mov'])
    return rectanglePos


def cylinder(downCirc = -120, upCirc = -70,radius = 15, resolution = 20):
    """Calculates coordinates for a cylinder
    `downCirc`: Lower circle of the cylinder
    `upCirc`: Upper circle of the cylinder
    `radius`: Radius of the cylinder
    `resolution`: Number of circlepoints
    `retrun`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for linear moving  
    """
    t = np.linspace(0, 2*m.pi, resolution)
    cylinderPos = []
    for num in t:
            x = -m.cos(num)*radius
            y = m.sin(num)*radius

            cylinderPos.append([x, y, downCirc, 0, 0, 0, 'mov'])

    for num in t:
            x = -m.cos(num)*radius
            y = m.sin(num)*radius

            cylinderPos.append([x, y, upCirc, 0, 0, 0, 'mov'])

    cylinderPos.append([0,0,-127,0,0,0,'mov'])
    return cylinderPos


def spiral(maxRadius = 25,resolution = 20, n = 5,robotHeight = -130):
    """Calculates coordinates for a spiral
    `maxRadius`: Max radius of the spiral
    `resolution`: Number of circlepoints of one circle
    `n`:Numer of circles
    `robotHeight`: Start height of the spiral
    `retrun`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for linear moving  
    """

    t = np.linspace(0,n*2*m.pi,n*resolution)
    r = np.linspace(2,maxRadius,n*resolution)
    spiralPos = []

    for i,num in enumerate(t):
        x = -m.cos(num)*r[i]
        y = m.sin(num)*r[i]
        z = robotHeight + 2*r[i]
        spiralPos.append([x,y,z,0,0,0,'mov'])

    return spiralPos


def elaboratedCurve(radius = 10, resolution = 22, robotHeight = -90,distx = 10,disty = 10, lines = 20):
    """Calculates coordinates for a 2D-Model
        `radius`: Radius of the circle
        `resolution`: Number of circlepoints, must be a multiple of 4
        `robotHeight`: z-Coordinate for 2D model (have to be a negative value)
        `distx`: x-distance between centerpoint of the circle and zero point of the coordinate system
        `disty`: y-distance between centerpoint of the circle and zero point of the coordinate system
        `lines`: Length of parallel lines
        `retrun`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for linear moving  
        """
    
    if resolution%4 != 0:
        while resolution%4 != 0:
            resolution += 1
    
    t = np.linspace(0, m.pi/2, int(resolution/4))
    elaboratedCurvePos = []
    for num in t:
            x = m.cos(num)*radius - distx
            y = -m.sin(num)*radius + disty
            elaboratedCurvePos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

    t = np.linspace(0, 2*m.pi, resolution)
    for num in t:
            x = -m.sin(num)*radius - distx
            y = m.cos(num)*radius - disty
            elaboratedCurvePos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

    t = np.linspace(0, 3*m.pi/2, int(3*resolution/4))
    for num in t:
            x = -m.sin(num)*radius - distx
            y = -m.cos(num)*radius + disty
            elaboratedCurvePos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

    
    elaboratedCurvePos.append([lines, disty, robotHeight, 0, 0, 0, 'lin'])

    z = np.linspace(0, m.pi, int(resolution/2))
    for num in z:
        x = m.sin(num)*radius + lines
        y = m.cos(num)*radius
        elaboratedCurvePos.append([x, y, robotHeight, 0, 0, 0, 'mov'])

    elaboratedCurvePos.append([0, -disty, robotHeight, 0, 0, 0, 'lin'])
    time.sleep(2) #TODO: Sleep wegmachen

    elaboratedCurvePos.append([0,0,-127,0,0,0,'mov'])
    #return elaboratedCurvePos
    return elaboratedCurvePos


if __name__ == '__main__':
    # Define return list values for demo sequences as this examples:
    # [x,y,z,a,b,c,'mov'] -> PTP
    # [x,y,z,a,b,c,'lin'] -> linear moving  
   ans = square()
   print(ans)