#Tommy Truong

import numpy as np
import random as r
from queue import PriorityQueue

CONST_RADIUS = 50
INF = 0x3f3f3f
false = False
true = True

class Node:
    def __init__(self, x, y, z, ID):
        self.x = x
        self.y = y
        self.z = z
        self.ID = ID
        self.next = []
        self.dist = dist(x,y,z)

def main():

    with open('world2.txt') as f:
        line = f.read().split('\n')

        x = line[0].split(' ')
        x = [int(i) for i in x]

        y = line[1].split(' ')
        y = [int(i) for i in y]

        z = line[2].split(' ')
        z = [int(i) for i in z]

        cube1 = line[3].split(' ')
        cube1  = [int(i) for i in cube1]

        cube2 = line[4].split(' ')
        cube2  = [int(i) for i in cube2]

        sphere1 = line[5].split(' ')
        sphere1 = [int(i) for i in sphere1]

        sphere2 = line[6].split(' ')
        sphere2 = [int(i) for i in sphere2]

        start = line[7].split(' ')
        start = [int(i) for i in start]

        end = line[8].split(' ')
        end = [int(i) for i in end]

    diffX = x[1] - x[0]
    diffY = y[1] - y[0]
    diffZ = z[1] - z[0]

    world = np.zeros((diffX,diffY,diffZ))
    print('the world has been created')

    world[start[0], start[1], start[2]] = 1
    world[end[0], end[1], end[2]] = 2

    side = cube1[3]/2 
    for i in range(int(cube1[0] - side), int(cube1[0] + side)):
        for j in range(int(cube1[1] - side), int(cube1[1] + side)):
            for k in range(int(cube1[2] - side), int(cube1[2] + side)):
                world[i,j,k] = -1
    side = cube2[3]/2 
    for i in range(int(cube2[0] - side), int(cube2[0] + side)):
        for j in range(int(cube2[1] - side), int(cube2[1] + side)):
            for k in range(int(cube2[2] - side), int(cube2[2] + side)):
                world[i,j,k] = -1
    side = sphere1[3]
    for i in range(int(sphere1[0] - side), int(sphere1[0] + side)):
        for j in range(int(sphere1[1] - side), int(sphere1[1] + side)):
            for k in range(int(sphere1[2] - side), int(sphere1[2] + side)):
                if distance(i,j,k, sphere1[0], sphere1[1], sphere1[2]) < side :
                    world[i,j,k] = -1

    side = sphere2[3]
    for i in range(int(sphere2[0] - side), int(sphere2[0] + side)):
        for j in range(int(sphere2[1] - side), int(sphere2[1] + side)):
            for k in range(int(sphere2[2] - side), int(sphere2[2] + side)):
                if distance(i,j,k, sphere2[0], sphere2[1], sphere2[2]) < side :
                    world[i,j,k] = -1
    print('All obstacles have been place in the world')
    
    obstacles = [cube1, cube2, sphere1, sphere2]
    sphere = [sphere1,sphere2]
    cube = [cube1, cube2]

    
    points = []
    print('Filling up space with random points')
    for i in range(0, int(diffX*diffY/2)):
        randx = r.randint(0, int(diffX/2))
        randy = r.randint(0, int(diffY/2))
        randz = r.randint(0, int(diffZ/2))
        if world[randx, randy, randz] == -1 or world[randx, randy, randz] == 1 or world[randx, randy, randz] == 2:
            continue
        else:
            points.append([randx, randy, randz])
    for i in range(0, int(diffX*diffY/2)):
        randx = r.randint(int(diffX/2), int(diffX - 1))
        randy = r.randint(int(diffY/2), int(diffY - 1))
        randz = r.randint(int(diffZ/2), int(diffZ - 1))
        if world[randx, randy, randz] == -1 or world[randx, randy, randz] == 1 or world[randx, randy, randz] == 2:
            continue
        elif [randx, randy, randz] in points:
            continue
        else:
            points.append([randx, randy, randz])
    
    mergeSort(points)
    
    #for i in range (0, len(points)):
    #    print(points[i])
    
    
    nodes = []
    nodes.append(Node(start[0], start[1], start[2], 0))
    for i in range(1, len(points)):
        nodes.append(Node(points[i][0], points[i][1], points[i][2], i))
        #print(points[i][0], points[i][1], points[i][2])
    nodes.append(Node(end[0], end[1], end[2], len(nodes)))
    #print(nodes[len(nodes) - 1].x, nodes[len(nodes) - 1].y, nodes[len(nodes) - 1].z)
    print('Making an adjanceny list of the nodes')


    for i in range(0, len(nodes)):
        count = 10
        nodeFlag = false
        n1 = nodes[i]
        # goes through each node one at a time and compare it with all the nodes in the list
       
        if n1.ID == 0:
            print(n1.x, n1.y, n1.z)
        for j in range(0, len(nodes)):
            n2 = nodes[j]
            # if the neighbors are within a distance of each other then check if there is a obstacle obstructing the edge
            # if not then add it as a neighbor
            if n1.ID == 0:
                print(distance(n1.x,n1.y,n1.z,n2.x,n2.y,n2.z))
            if isNeighbor(n1, n2):
                cubeFlag = sphereFlag = false
                if n1.ID == 0:
                    print(true)
                for k in range(len(cube)):
                    if not validEdge(cube[k], cube[k][3]/2, n1, n2):
                        cubeFlag = true
                        break
                if cubeFlag:
                    continue
                for k in range(len(sphere)):
                    if not validEdge(sphere[k], sphere[k][3], n1, n2):
                        sphereFlag = True
                        break
                if sphereFlag:
                    continue
                n2.dist = distance(n1.x,n1.y,n1.z,n2.x,n2.y,n2.z)
                if n1.ID == 0:
                    print(n2.x, n2.y, n2.z)
                if count == 5:
                    break
                else:
                    count +=1
                nodes[i].next.append(n2)
    print(nodes[0].x, nodes[0].y, nodes[0].z)
    for x in nodes[0].next:
        print(x.x, x.y, x.z)
    print('----')
    print(nodes[len(nodes) - 1].x, nodes[len(nodes) - 1].y, nodes[len(nodes) - 1].z)
    for x in nodes[len(nodes) - 1].next:
        print(x.x, x.y, x.z)

    print('Finding the path....')

    dist_point = [INF]*len(nodes)
    dist_point[0] = 0.0
    prev = [Node]*len(nodes)
    pqueue = PriorityQueue()
    pqueue.put((nodes[0].dist, nodes[0]))
    queue = []
    queue.append(nodes[0])
    visited = [False]*len(nodes)
        
    while not pqueue.empty():
        #nodeSort(queue)
        #queue.reverse()
        node = pqueue.get()[1]
        if visited[node.ID]:
            continue
        if node.ID == len(nodes) - 1:
            print(node.x, node.y, node.z)
        for i in range(0, len(node.next)):
            if dist_point[node.next[i].ID] > dist_point[node.ID] + node.next[i].dist:
                dist_point[node.next[i].ID] = dist_point[node.ID] + node.next[i].dist
                #print(node.x, node.y, node.z)
                pqueue.put((-node.next[i].ID, node.next[i]))
                #queue.append(node.next[i])
                prev[node.next[i].ID] = node
                #print(node.x, node.y, node.z, 'going into node: ', node.next[i].x,node.next[i].y,node.next[i].z)
        visited[node.ID] = True

    path = []
    stuff = []
    sequence = []

    sequence.append(nodes[len(nodes) - 1])
    ID = nodes[len(nodes) - 1].ID
    while ID != 0:
        sequence.append(prev[ID])
        ID = prev[ID].ID
        

    sequence.append(nodes[0])
    while len(sequence) != 0:
        stuff.append(sequence.pop())

    tmpArr = []

    for i in range(0, len(stuff)):
        tmpArr.append([stuff[i].x, stuff[i].y, stuff[i].z])
        #path.append([stuff[i].x, stuff[i].y, stuff[i].z])
        

    print('Filling in the gaps')
    #for i in range(0, len(tmpArr) - 1):
    for i in range(0, len(tmpArr) - 1):
        x_start = tmpArr[i][0]
        y_start = tmpArr[i][1]
        z_start = tmpArr[i][2]
        x_end = tmpArr[i+1][0]
        y_end = tmpArr[i+1][1]
        z_end = tmpArr[i+1][2]
        path.append([x_start, y_start, z_start])
        while (x_start != x_end) or (y_start != y_end) or (z_start != z_end):
            if (x_start == x_end):
                x_start = x_start + 0
            elif x_start > x_end:
                x_start -= 1
            else:
                x_start = x_start +  1
            if (y_start == y_end):
                y_start = y_start +  0
            elif y_start > y_end:
                y_start -= 1
            else:
                y_start = y_start + 1
            if (z_start == z_end):
                z_start = z_start + 0
            elif z_start > z_end:
                z_start -= 1
            else:
                z_start = z_start + 1

            p1 = x_start
            p2 = y_start
            p3 = z_start

            path.append([p1,p2,p3])
    
    


    print('the path has been found')
    print('Outputting the path into PRM_output.txt')
    file = open('PRM_output.txt', 'w')
    for i in range(0, len(path)):
        #file.write('%d,%d,%d\n' %(path[i].x, path[i].y, path[i].z))
         file.write('%d %d %d\n' %(path[i][0], path[i][1], path[i][2]))
    file.close()
    print('Finish')

                
    


def validEdge(obstacle, side, n1, n2):
    flagx = flagy = flagz = true
    if int(n1.x) in range(int(obstacle[0] - side), int(obstacle[0] + side + 1)) or int(n2.x) in range(int(obstacle[0] - side), int(obstacle[0] + side + 1)):
    #if int(obstacle[0] - side) in range(int(n1.x), int(n2.x)) or int(obstacle[0] + side + 1) in range(int(n1.x), int(n2.x)):
        #print('x: ', n1.x, n2.x, obstacle[0] - side, obstacle[0] + side)
        flagx = false
    if int(n1.y) in range(int(obstacle[1] - side), int(obstacle[1] + side + 1)) or int(n2.y) in range(int(obstacle[1] - side), int(obstacle[1] + side + 1)):
        #print('y: ',n1.y, n2.y, obstacle[1] - side, obstacle[1] + side)
        flagy = false
    if int(n1.z) in range(int(obstacle[2] - side), int(obstacle[2] + side + 1)) or int(n2.z) in range(int(obstacle[2] - side), int(obstacle[2] + side + 1)):
        #print('z: ',n1.z, n2.z, obstacle[2] - side, obstacle[2] + side)
        flagz = false
    
    return flagx or flagy or flagz

def isNeighbor(n1, n2):
    if n1.x == n2.x and n2.y == n1.y and n1.z == n2.z:
        return false
    else:
        return distance(n1.x, n1.y, n1.z, n2.x, n2.y, n2.z) < CONST_RADIUS



#compues the distance from the edge to the centerpoint
def distance(x1,y1,z1,x0,y0,z0):
    x = np.square(float(x1) - x0)
    y = np.square(float(y1) - y0)
    z = np.square(float(z1) - z0)
    
    return np.sqrt(x+y+z)

def mergeSort(vert):
    if len(vert) >1:
        mid = len(vert)//2
        l = vert[:mid]
        r = vert[mid:]

        mergeSort(l)
        mergeSort(r)

        i = j = k = 0
        while i < len(l) and j < len(r):
            if dist(l[i][0],l[i][1],l[i][2]) < dist(r[j][0],r[j][1],r[j][2]):
                vert[k] = l[i]
                i+=1
            else:
                vert[k] = r[j]
                j+=1
            k+=1
        while i < len(l):
            vert[k] = l[i]
            i+=1
            k+=1
        while j < len(r):
            vert[k] = r[j]
            j +=1
            k +=1

def dist(x1,y1,z1):
    x = np.square(float(x1))
    y = np.square(float(y1))
    z = np.square(float(z1))
    
    return np.sqrt(x+y+z)

main()
