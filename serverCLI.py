import sys
import argparse
import math
from adjacencygraph import AdjacencyGraph
import queue

myfile = open("edmonton-roads-2.0.1.txt","r")
def breadth_first_search(g, v):
    '''Discovers all vertices in graph g reachable from vertex v
    and returns the search graph. Paths on the search graph
    are guaranteed to follow shortest paths from v.

    Args:
        g (graph): Graph to search in.
        v (vertex of g): Where the search starts from.

    Returns:
        dict: Dictionary whose keys are nodes u discovered with value being
        the node from where the node described by the key was discovered.
        By definition, v is discovered from v.
    '''
    todolist = queue.deque([v])  # todolist also stores "from where"
    reached = {v: v}
    while todolist:
        u = todolist.popleft()
        for w in g.neighbours(u):
            if w not in reached:
                reached[w] = u  # w has just been discovered
                todolist.append(w)
    return reached

class MinHeap:

    def __init__(self):
        self._array = []

    def add(self, key, value):
        self._array.append((key, value))
        self.fix_heap_up(len(self._array)-1)

    def pop_min(self):
        if not self._array:
            raise RuntimeError("Attempt to call pop_min on empty heap")
        retval = self._array[0]
        self._array[0] = self._array[-1]
        del self._array[-1]
        if self._array:
            self.fix_heap_down(0)
        return retval

    def fix_heap_up(self, i):
        if self.isroot(i):
            return
        p = self.parent(i)
        if self._array[i][0] < self._array[p][0]:
            self.swap(i, p)
            self.fix_heap_up(p)

    def swap(self, i, j):
        self._array[i], self._array[j] = \
            self._array[j], self._array[i]

    def isroot(self, i):
        return i == 0

    def isleaf(self, i):
        return self.lchild(i) >= len(self._array)

    def lchild(self, i):
        return 2*i+1

    def rchild(self, i):
        return 2*i+2

    def parent(self, i):
        return (i-1)//2

    def min_child_index(self, i):
        l = self.lchild(i)
        r = self.rchild(i)
        retval = l
        if r < len(self._array) and self._array[r][0] < self._array[l][0]:
            retval = r
        return retval

    def isempty(self):
        return len(self._array) == 0

    def length(self):
        return len(self._array)

    def fix_heap_down(self, i):
        if self.isleaf(i):
            return

        j = self.min_child_index(i)
        if self._array[i][0] > self._array[j][0]:
            self.swap(i, j)
            self.fix_heap_down(j)

    # Some stnadard collection interfaces

    # So the len() function will work.
    def __len__(self):
        return len(self._array)

    # Iterator
    def __iter__(self):
        return iter(self._array)

    def __next__(self):
        return (self._array).__next__



class Location:
    def __init__(self, lat, lon):
        # We want the values as integers so we will multiply by 100 000
        self.lat = int(float(lat)*100000) # .lat is what is being stored in the class and lat is what value is being passed into the class
        self.lon = int(float(lon)*100000)
        # Options:
        # vertex class
        # simple dictionary
        #   key id, value w
    def __str__():
        return str(self.lat)

location_lookup = {}

def read_city_graph(file): #file for simple graph
    g = AdjacencyGraph()  # should this be directed?
    for line in file: #file 
        if line[0] == 'V':
            _, ID, lat, lon = line.split(',') # Creating variables ID , lat, lon using line split for each line in the graph
            g.add_vertex(int(ID)) # adding a new vertex using the vertex ID
            location_lookup[int(ID)] =(lat, lon) # warning: not returning this
        elif line[0] == 'E':
            _, ID1, ID2, street_name = line.split(',')
            g.add_edge((int(ID1), int(ID2)), True) # SHOULD THE DEFAULT BE SET TO TRUE????
    return g


def cost_distance(u,v):

    '''Computes and returns the strait-line distance between the two verticies u and v.
        Args:
            u,v: The ids for two verticies that are the start and end of a valid edge in the graph.
        Returns:
            Numeric Value: The distance between the two verticies.
    '''
    # u is node one, v is node two
    latU = int(location_lookup[u][0])
    lonU = int(location_lookup[u][1])
    latV = int(location_lookup[v][0])
    lonV = int(location_lookup[v][1])

    distance = math.sqrt(((latU-latV)**2)+((lonU-lonV)**2)) # remove square root for efficiency and save time
    return distance


def least_cost_path(graph, start, dest, cost):
    reached = dict()
    runners = MinHeap()
    runners.add(0, (0, start, start))

    while len(runners):
        (key, (t_arrive, v_from, v_to)) = runners.pop_min()
        if v_to in reached and reached[v_to][0] < t_arrive:
            continue
        reached[v_to] = (t_arrive, v_from)

        for v_next in graph.neighbours(v_to):
            if v_next in reached:
                continue
            runners.add(t_arrive + cost(v_to, v_next), (t_arrive + cost(v_to, v_next), v_to, v_next))

    path = []
    path.append(dest)

    while start not in path:
         if set(path) < set(reached.keys()):
           path.append(reached[path[-1]][1])
         else:
           break

    path.reverse()
    if len(path) < 2:
        return []
    return path



g = read_city_graph(myfile.readlines()) # Reading in Edmonton Roads as an input file myfile.split('\n')
#print(g._vertices)
#print(location_lookup)
'''
valid = 'R'

for line in sys.stdin:
    if line[0] == 'R':
        inputInfo = line.split(',')
    # else:
        # do anything? or do nothing??? *************
'''
# the above code will be uncommented when we run it from Command line and the below example case will be removed
inputInfo =['R','5353683','-11350844','5352487','-11345827']
strtLat = inputInfo[1]
strtLon = inputInfo[2]
endLat  = inputInfo[3]
endLon  = inputInfo[4]
strtCoord=(strtLat,strtLon)
print(strtCoord)

strtID = [key for key, value in location_lookup.values() if value == strtCoord] #how can we do an inverse dictionary lookup? this isnt working
print(strtID)
#print(least_cost_path(g,1,9, cost_distance))


