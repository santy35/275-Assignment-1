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
    print(distance)
