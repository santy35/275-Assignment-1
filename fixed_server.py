'''
CMPUT 275 Assignment #1 Part 1
Stefan Vidakovic - 1458690
Tyler Santos - 1509651
'''

'''
Feedback
Open Tests Passed = 2/2
Closed Tests Passed = 43/56
cost_distance function = 10/1
find closest vertex = 10/1
Guidelines = 10/1
Comments = 1/1
- When start == end, it reports that there is no possible path (N 0)
- Prints 'E' when there is no route
- Cannot handle multiple requests
'''


import sys
import argparse
import math
from adjacencygraph import AdjacencyGraph
import queue
from cs_message import *

myfile = open("edmonton-roads-2.0.1.txt", "r")

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
        self.fix_heap_up(len(self._array) - 1)

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
        return 2 * i + 1

    def rchild(self, i):
        return 2 * i + 2

    def parent(self, i):
        return (i - 1) // 2

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

location_lookup = {}

def read_city_graph(myfile):  # file for simple graph
    g = AdjacencyGraph()
    for line in myfile:
        if line[0] == 'V':
            # Creating variables ID , lat, lon using line split for each line in the graph
            line = line.strip('\n')
            _, ID, lat, lon = line.split(',')
            g.add_vertex(int(ID))  # adding a new vertex using the vertex ID
            location_lookup[int(ID)] = (
                int(float(lat) * 100000), int(float(lon) * 100000))

        elif line[0] == 'E':
            _, ID1, ID2, street_name = line.split(',')
            g.add_edge((int(ID1), int(ID2)), True)
    return g


def cost_distance(u, v):
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

    # We could remove the square root here for efficiency and to save time, but we kept it for accuracy
    distance = math.sqrt(((latU - latV)**2) + ((lonU - lonV)**2))
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
            runners.add(t_arrive + cost(v_to, v_next),
                        (t_arrive + cost(v_to, v_next), v_to, v_next))

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


# Finding the closest distance and node to the provided coordinates
def find_closest(lat, lon):
    y = []

    for i in location_lookup.keys():
        hpot = (
            int(math.hypot(lat - location_lookup[i][0], lon - location_lookup[i][1])))
        y.append([hpot, i])

    return min(y)[1]


def handle_input():
    # Reading in provided coordinates from standard input
    try:
        line = input()
    except EOFError:
        return False
    if line == '' or line == '\n':
        return False
    elif line[0] == 'R':
        inputInfo = line.split()
    else:
        return True

    strtLat = int(float(inputInfo[1]))
    strtLon = int(float(inputInfo[2]))
    endLat = int(float(inputInfo[3]))
    endLon = int(float(inputInfo[4]))
    strtCoord = (strtLat, strtLon)
    endCoord = (endLat, endLon)

    if strtCoord != endCoord:
        strtID = find_closest(strtLat, strtLon)
        endID = find_closest(endLat, endLon)
        path = least_cost_path(g, strtID, endID, cost_distance)

        print('N', len(path))
        for i in path:
            temp = location_lookup[i]
            temp1 = str(temp[0])
            temp2 = str(temp[1])
            print('W', temp1, temp2)

        if 0 < len(path):
            print('E')
    else:
        print('N', 1)
        print('W',strtCoord[0], strtCoord[1])
        print('E')

    return True



# Reading in Edmonton Roads as an input file myfile.split('\n')
g = read_city_graph(myfile.readlines())


def protocol(serial_in, serial_out):
    # simple echo protocol
    while True:
        while True:
            msg = receive_msg_from_client(serial_in)
            log_msg(msg)
            if msg[0] == "R":
                break

        # Hope that it's a properly formatted R message
        coords = msg[2:].split()

        if len(coords) != 4:
            continue

        (lat_s, lon_s, lat_e, lon_e) = coords
        lat_s = int(float(lat_s) * 100000)
        lon_s = int(float(lon_s) * 100000)
        lat_e = int(float(lat_e) * 100000)
        lon_e = int(float(lon_e) * 100000)
        start = find_closest(lat_s, lon_s)
        dest = find_closest(lat_e, lon_e)
        if start != dest:
            path = least_cost_path(g,start,dest,cost_distance)
            n = len(path)
            send_msg_to_client(serial_out, "N {}" .format(n))
            for i in path:
                temp = location_lookup[i]
                temp1 = str(temp[0])
                temp2 = str(temp[1])
                send_msg_to_client(serial_out, "W {} {}" .format(temp1,temp2))

                msg = receive_msg_from_client(serial_in)
                log_msg(msg)

            send_msg_to_client(serial_out, "E")
        else:
            send_msg_to_client(serial_out, "E")

def main():

    import argparse
    parser = argparse.ArgumentParser(
        description='Client-server message test.',
        formatter_class=argparse.RawTextHelpFormatter,
        )

    parser.add_argument("-d0",
        help="Debug off",
        action="store_false",
        dest="debug")

    parser.add_argument("-s",
        help="Set serial port for protocol",
        nargs="?",
        type=str,
        dest="serial_port_name",
        default="/dev/ttyACM0")

    args = parser.parse_args()

    debug = args.debug

    set_logging(debug)

    # this imports serial, and provides a useful wrapper around it
    import textserial

    serial_port_name = args.serial_port_name;
    log_msg("Opening serial port: {}".format(serial_port_name))

    # Open up the connection
    baudrate = 9600  # [bit/seconds] 115200 also works

    # Run the server protocol forever

    # The with statment ensures that if things go bad, then ser
    # will still be closed properly.
    # errors='ignore' allows any 1 byte character, not just the usual
    # ascii range [0,127]

    with textserial.TextSerial(
        serial_port_name, baudrate, errors='ignore', newline=None) as ser:
        protocol(ser, ser)

if __name__ == "__main__":
    main()
