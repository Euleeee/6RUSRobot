import math as m
import numpy as np

def square(halfSideLength = 30, robotHeight = -90):
    """Calculate coordinates for a square
        `halfSideLength`: half length of the edge
        `robotHeight`: z-Coordinate for 2D model (have to be a negative value)
        `retrun`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for linear moving  
        """
#  _______ 
# |       |
# |       |
# |_______|
#  
# | a | 
# a = halfSideLength

    posSquare = [
            [0,0,-70,0,0,0,'mov'],
            [halfSideLength,halfSideLength,robotHeight,0,0,0,'mov'],
            [-halfSideLength,halfSideLength,robotHeight,0,0,0,'lin'],
            [-halfSideLength,-halfSideLength,robotHeight,0,0,0,'lin'],
            [halfSideLength,-halfSideLength,robotHeight,0,0,0,'lin'],
            [halfSideLength,halfSideLength,robotHeight,0,0,0,'lin'],
            [0,0,-127,0,0,0,'mov']
        ]

    return posSquare


def triangle(halfSideLength = 15, robotHeight = -90) :
    """Calculate coordinates for a samesided triangle
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

    posTriangle = [
            [0,0,-70,0,0,0,'mov'],
            [-hHalf,halfSideLength,robotHeight,0,0,0,'mov'],
            [-hHalf,-halfSideLength,robotHeight,0,0,0,'lin'],
            [hHalf,0,robotHeight,0,0,0,'lin'],
            [-hHalf,halfSideLength,robotHeight,0,0,0,'lin'],
            [0,0,-127,0,0,0,'mov']
        ]

    return posTriangle


def circle(radius = 15, resolution = 20, robotHeight = -90, n = 1, dir = 0):
    """Calculate coordinates for a 2D-circle
        `radius`: Radius of the circle
        `resolution`: Number of circlepoints
        `robotHeight`: z-Coordinate for 2D model (have to be a negative value)
        `n`: Number of rotations
        `dir`: Direction of the circle 
        `retrun`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for linear moving  
        """
    
    t = np.linspace(0, n*2*m.pi, resolution*n)
    circlePos = [[0,0,-70,0,0,0,'mov']]
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
    """Calculate coordinates for a 2D-eight
        `radius`: Radius of the circle
        `resolution`: Number of circlepoints
        `robotHeight`: z-Coordinate for 2D model (have to be a negative value)
        `n`: Number of rotations
        `retrun`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for linear moving  
        """
    
    t = np.linspace(0, n*2*m.pi, resolution*n)
    eightPos = [[0,0,-70,0,0,0,'mov']]
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
    """Calculate coordinates for a tetrahedron
        `halfSideLength`: half sidelength of the tetrahedron
        `robotHeight`: z-Coordinate for 2D Base (have to be a negative value)
        `retrun`: List of positions and driving mode. Exmaple: [x,y,z,a,b,c,'mov'] for PTP or [x,y,z,a,b,c,'lin'] for linear moving  
        """

    hBaseHalf = (halfSideLength * m.sqrt(3)/2)/2
    hTetra = m.sqrt(6)*halfSideLength/3
    pyramidePos = [
            [0,0,-70,0,0,0,'mov'],

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


def pickPlace():
    pickPlacePos = [[0,0,-70,0,0,0,'mov']]
    return pickPlacePos


def rectangleSignal():
    rectanglePos = [[0,0,-70,0,0,0,'mov']]
    return rectanglePos


def cylinder():
    cylinderPos = [[0,0,-70,0,0,0,'mov']]
    return cylinderPos


def spiral():
    spiralPos = [[0,0,-70,0,0,0,'mov']]
    return spiralPos


def elaboratedCurve():
    elaboratedCurvePos = [[0,0,-70,0,0,0,'mov']]
    return elaboratedCurvePos


if __name__ == '__main__':
    # Define return list values for demo sequences as this examples:
    # [x,y,z,a,b,c,'mov'] -> PTP
    # [x,y,z,a,b,c,'lin'] -> linear moving  
   ans = pyramide()
   print(ans)