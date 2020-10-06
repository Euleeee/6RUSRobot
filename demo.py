import math as m

def square(halfSideLength = 30):
    """Robot drives a square
        `a`: half length of the edge
        `retrun`: List of positions and 
        """
#  _______ 
# |       |
# |       |
# |_______|
#  
# | a | 

    posSquare = [[0,0,-70,0,0,0,'mov'],
        [halfSideLength,halfSideLength,-90,0,0,0,'mov'],
        [-halfSideLength,halfSideLength,-90,0,0,0,'lin'],
        [-halfSideLength,-halfSideLength,-90,0,0,0,'lin'],
        [halfSideLength,-halfSideLength,-90,0,0,0,'lin'],
        [halfSideLength,halfSideLength,-90,0,0,0,'lin'],
        [0,0,-127,0,0,0,'mov']]

    return posSquare


def triangle(halfSideLength = 15):
    """Robot drives a samesided triangle
        `halfSideLength`: half sidelength of the triangle
        `retrun`: List of positions and 
        """
#     ^ 
#    / \ 
#   /   \  
#  /     \   
# /_______\
#  
# |   a    | 

    hHalf = (halfSideLength * m.sqrt(3)/2)/2

    posTriangle = [[0,0,-70,0,0,0,'mov'],
        [-hHalf,halfSideLength,-90,0,0,0,'mov'],
        [-hHalf,-halfSideLength,-90,0,0,0,'lin'],
        [hHalf,0,-90,0,0,0,'lin'],
        [-hHalf,halfSideLength,-90,0,0,0,'lin'],
        [0,0,-127,0,0,0,'mov']]

    return posTriangle

if __name__ == '__main__':
   print('hi')