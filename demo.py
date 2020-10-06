import math as m

def square(halfSideLength = 30, robotHeight = -90):
    """Calculate coordinates for a square
        `halfSideLength`: half length of the edge
        `robotHeight`: z-Coordinate for 2D model (have to be a negative value)
        `retrun`: List of positions and driving mode
        """
#  _______ 
# |       |
# |       |
# |_______|
#  
# | a | 

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
        `retrun`: List of positions and 
        """
#     ^ 
#    / \ 
#   /   \  
#  /     \   
# /_______\
#  
# | a | 

    hHalf = (halfSideLength * m.sqrt(3)/2)/2

    posTriangle = [
        [0,0,-70,0,0,0,'mov'],
        [-hHalf,halfSideLength,-robotHeight,0,0,0,'mov'],
        [-hHalf,-halfSideLength,robotHeight,0,0,0,'lin'],
        [hHalf,0,robotHeight,0,0,0,'lin'],
        [-hHalf,halfSideLength,robotHeight,0,0,0,'lin'],
        [0,0,-127,0,0,0,'mov']
        ]

    return posTriangle

if __name__ == '__main__':
   print('hi')