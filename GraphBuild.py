'''
Tyler Santos Comput 274 Exercise 2 1509651
'''
import sys  # contains commands we need to run multiple command line arguments
import argparse
from adjacencygraph import AdjacencyGraph
from bfs import breadth_first_search

parser = argparse.ArgumentParser(
    description='Text Frequency Analysis',
    formatter_class=argparse.RawTextHelpFormatter,
)

parser.add_argument("infile",
                    help="file to be sorted, stdin if omitted",
                    nargs="?",
                    type=argparse.FileType('r'),
                    default=sys.stdin
                    )

args = parser.parse_args()


'''
V,1497333445,53.500376,-113.572036
V,1497333460,53.495822,-113.565706
V,1497333464,53.492020,-113.565725
V,1497333466,53.499024,-113.566567
V,1497333470,53.497904,-113.564528
E,251245715,225247824,119 Street NW
E,225247824,251245715,119 Street NW
E,251245716,251245715,119 Street NW
E,251245715,251245716,119 Street NW
'''
'''
V,ID,LATITUDE,LONGITUDE
E,VERTEXID,VERTEXID,STREETINFO
'''
'''
V,1,1,-1
V,2,3,4
V,3,2,-6
V,4,-4,-2
V,5,-3,5
V,6,5,8
V,7,1,7
V,8,-2,-3
V,9,-5,7
E,1,2,ONETOTWOPATH
E,1,3,ONETOTHREEPATH
E,3,8,THREETOEIGHTPATH
E,1,7,ONETOSEVENPATH
E,2,6,TWOTOSIXPATH
E,7,6,SEVENTOSIXPATH
E,7,5,SEVENTOFIVEPATH
E,6,9,SIXTONINEPATH
E,9,5,NINETOFIVEPATH
E,5,9,FIVETONINEPATH
E,5,4,FIVETOFOURPATH
E,4,9,FOURTONINEPATH


'''

def count_components(g):
    vertices = list(g.vertices())
    node_groups = []
    count = 0

    while (len(vertices) != 0):  # all nodes have been counted when the queue is empty
        # reached is a set of everynode we have visited in our search
        reached = breadth_first_search(g, vertices[0])
        for vertex in range(len(vertices)):
            if vertices[vertex] in reached:  # does it still exist?
                node_groups.append(vertices[vertex])
        vertices = [z for z in vertices if z not in node_groups]
        count += 1
        node_groups = []

    print(count)


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

def read_city_graph(file):
    g = AdjacencyGraph()  # should this be directed?
    for line in file:
        if line[0] == 'V':
            _, ID, lat, lon = line.split(',') # Creating variables ID , lat, lon using line split for each line in the graph
            g.add_vertex(int(ID)) # adding a new vertex using the vertex ID
            location_lookup[int(ID)] =(lat, lon) # warning: not returning this
        elif line[0] == 'E':
            _, ID1, ID2, street_name = line.split(',')
            g.add_edge((int(ID1), int(ID2)), True) 
    return g

# number of components in Edmonton graph = 648


myfile = '''V,1,1,-1
V,2,3,4
V,3,2,-6
V,4,-4,-2
V,5,-3,5
V,6,5,8
V,7,1,7
V,8,-2,-3
V,9,-5,7
E,1,2,ONETOTWOPATH
E,1,3,ONETOTHREEPATH
E,3,8,THREETOEIGHTPATH
E,1,7,ONETOSEVENPATH
E,2,6,TWOTOSIXPATH
E,7,6,SEVENTOSIXPATH
E,7,5,SEVENTOFIVEPATH
E,6,9,SIXTONINEPATH
E,9,5,NINETOFIVEPATH
E,5,9,FIVETONINEPATH
E,5,4,FIVETOFOURPATH
E,4,9,FOURTONINEPATH'''

g = read_city_graph(myfile.split('\n'))
print(g._vertices)
print(location_lookup)

#cost_distance()
