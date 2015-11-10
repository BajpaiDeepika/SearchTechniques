#References -->
#https://docs.python.org/2/tutorial/datastructures.html#using-lists-as-queues
#https://docs.python.org/2/tutorial/inputoutput.html
#http://stackoverflow.com/questions/4913349/haversine-formula-in-python-bearing-and-distance-between-two-gps-points
#http://andrew.hedges.name/experiments/haversine/
# Stack Overflow

'''Implemented bfs,dfs and astar with routing option as 'time' and 'distance'
Heuristic function for distance is determined by calculating linear distance between two cities using latitude and longitude.
Heuristic function for time is determined by calculating average speed limit for a city which has all connected highways.

Challenges faced -
Heuristic for calculating minimum segments was thought to be based on getting maximum number of common highways between two cities.
'''

from __future__ import division
from decimal import Decimal
from math import radians, cos, sin, asin, sqrt
import operator
from sys import argv


city_gps_file_location = "city-gps.txt"
city_gps_file = open(city_gps_file_location)

road_segments_file_location = "road-segments.txt"
roadSegments_file = open(road_segments_file_location)

# This will hold all the visited cities in sequence and the first two elements would be distance and time taken to travel to these cities
#route = []
optimalRoute = []
def isValidCity(name):
    city_gps_file = open(city_gps_file_location)
    for cityDetails in city_gps_file:
        cityInfo = cityDetails.split()
        if(cityInfo[0].lower() == name.lower()):
            return True
    return False

def isCityVisited(city,route):
    if city in route:
        return True
    return False

def getOptimalRouteFromTraversedCities(traversedRoute, cityToConnectedCitiesMappings):
    optimalRoute = []
    # back track the cities in our visited list (route list) to get the the optimal path
    destCity = traversedRoute[len(traversedRoute) - 1]
    optimalRouteFound = False
    optimalRoute.append(destCity)  
    while not optimalRouteFound:  
        for parentCity, childCities in cityToConnectedCitiesMappings.iteritems():
            if destCity in childCities:
                optimalRoute.append(parentCity)  
                destCity = parentCity
            
            if destCity == traversedRoute[0]:
                optimalRouteFound = True
    optimalRoute.reverse()   
    return optimalRoute

def getRouteViaBFS(source,dest):
    # Fringe will have road segments
    route = []
    route.append(sourceCity)
    fringe = []
    startCity = source
    cityToConnectedCitiesMapping = {}
    if not isValidCity(source) or not isValidCity(dest):
        # return empty route and display message that city is not valid
        route.append("Either source or destination city is not valid. Please enter again")
        return route
    routeNotFound = True
    while routeNotFound:
        roadSegments_file = open(road_segments_file_location)
        connectedCities = []
        for roadSegmentDetails in roadSegments_file:
            roadSegment = roadSegmentDetails.split()
            city1 = roadSegment[0]
            city2 = roadSegment[1]
            flag1 = city1 == source and not isCityVisited(city2,route) and isValidCity(city2)
            flag2 = city2 == source and not isCityVisited(city1,route) and isValidCity(city1)
            #if (city1 == source and not isCityVisited(city1)) or (city2 == source and not isCityVisited(city2)):
            if flag1 or flag2:
                #this is the first route we should take in BFS
                if not roadSegment in fringe:
                    fringe.append(roadSegment)
                    
                if city1 == source:
                    connectedCities.append(city2)
                else:
                    connectedCities.append(city1)
                # check if we have reached destination
                if city1 == dest or city2 == dest:
                    
                    route.append(dest)
                    routeNotFound = False
        
        cityToConnectedCitiesMapping[source] = connectedCities
        
        # after the above loop, remove first element from fringe and now this will be our new source
        if routeNotFound:
            if fringe:
                
                nextRoadSegmentToTravel = fringe.pop(0)
                
                city1 = nextRoadSegmentToTravel[0]
                city2 = nextRoadSegmentToTravel[1]
                
                if city1 in route:
                    route.append(city2)
                    source = city2
                else:
                    route.append(city1)
                    source = city1
            else:
                route.append("No route exists for your desired destination")
        # here we know that some route exists between source and destination
        else:
            optimalRoute = getOptimalRouteFromTraversedCities(route,cityToConnectedCitiesMapping)    
            return optimalRoute      
    return route

def getTimeAndDistance(finalRoute):
    timeAndDistance = []
    timeAndDistance.append(0)
    timeAndDistance.append(0)
    startCity = finalRoute[0]
    nextCityToVisit = finalRoute[1]
    destCity = finalRoute[len(finalRoute) - 1]
    areAllCitiesVisitedInRoute = False
    i = 1
    while not areAllCitiesVisitedInRoute:
        roadSegments_file = open(road_segments_file_location)
        for roadSegmentDetails in roadSegments_file:
            roadSegment = roadSegmentDetails.split()
            city1 = roadSegment[0]
            city2 = roadSegment[1]
            flag1 = city1 == startCity and city2 == nextCityToVisit
            flag2 = city1 == nextCityToVisit and city2 == startCity
            if flag1 or flag2:
                timeAndDistance[0] += int(roadSegment[2])
                timeAndDistance[1] += int(roadSegment[2])/int(roadSegment[3])
                startCity = finalRoute[i]   
                if(i < len(finalRoute) - 1):
                    nextCityToVisit = finalRoute[i+1]
                    i += 1
                break
        if destCity == startCity:
            areAllCitiesVisitedInRoute = True
    return timeAndDistance

# currentCity -- recently visited city / current city
#nextCityToVisit -- Next city to visit.Heuristic is being calculated for this city
# dest -- Goal node
def getHeuristicForDistance(currentCity,nextCityToVisit,dest):
    cityDistances = []
    city_gps_file = open(city_gps_file_location)
    for cityDetails in city_gps_file:
        cityInfo = cityDetails.split()
        if(cityInfo[0].lower() == nextCityToVisit.lower()):
            cityDistances.append(cityInfo)
        if(cityInfo[0].lower() == dest.lower()):
            cityDistances.append(cityInfo)
        if len(cityDistances) == 2:
            break
    
    #linearDistance = haversine(cityDistances[0][1], cityDistances[0][2], cityDistances[1][1], cityDistances[1][2])
    
    distanceFromCurrentCityToNextCity = 0
    if not currentCity == nextCityToVisit:
        roadSegments_file = open(road_segments_file_location)
        for roadSegmentDetails in roadSegments_file:
            roadSegment = roadSegmentDetails.split()
            city1 = roadSegment[0]
            city2 = roadSegment[1]
            flag1 = city1 == currentCity and city2 == nextCityToVisit
            flag2 = city1 == nextCityToVisit and city2 == currentCity
            if flag1 or flag2:
                 distanceFromCurrentCityToNextCity = int(roadSegment[2])
                 break
    return getLinearDistance(float(cityDistances[0][1]), float(cityDistances[1][1]), float(cityDistances[0][2]), float(cityDistances[1][2]))
    #return (linearDistance + distanceFromCurrentCityToNextCity)

def getHeuristicForTime(source,dest):
    cityDistances = []
    city_gps_file = open(city_gps_file_location)
    for cityDetails in city_gps_file:
        cityInfo = cityDetails.split()
        if(cityInfo[0].lower() == source.lower()):
            cityDistances.append(cityInfo)
        if(cityInfo[0].lower() == dest.lower()):
            cityDistances.append(cityInfo)
        if len(cityDistances) == 2:
            break
    distance = haversine(cityDistances[0][1], cityDistances[1][1], cityDistances[0][2], cityDistances[1][2])
    #distance = getLinearDistance(float(cityDistances[0][1]), float(cityDistances[1][1]), float(cityDistances[0][2]), float(cityDistances[1][2]))
    averageSpeed = (estimateSpeedForCity(source) + estimateSpeedForCity(dest) ) / 2
    estimatedTime = distance / averageSpeed
    return estimatedTime
    
def estimateSpeedForCity(currentCity):
    averageSpeed = 1
    speedsList = []
    roadSegments_file = open(road_segments_file_location)
    for roadSegmentDetails in roadSegments_file:
        roadSegment = roadSegmentDetails.split()
        city1 = roadSegment[0]
        city2 = roadSegment[1]
        if currentCity == city1 or currentCity == city2:
            speedsList.append(int(roadSegment[3]))
    
    averageSpeed = sum(speedsList)/len(speedsList)
    return averageSpeed

def getLinearDistance(lat1,lat2,lon1,lon2):
    distance = sqrt((lat2-lat1)**2 + (lon2-lon1)**2)
    return distance
    
def haversine(lat1, lat2, lon1, lon2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(radians, [float(lon1), float(lat1), float(lon2), float(lat2)])
    

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371 # Radius of earth in kilometers. Use 3956 for miles
    return c * r

def getHeuristicsByOption(currentCity,nextCityToVisit,dest,option):
    if option.lower() == "distance":
        return getHeuristicForDistance(currentCity,nextCityToVisit,dest)
    elif option.lower() == "time":
        return getHeuristicForTime(nextCityToVisit,dest)
    elif option.lower() == "segment":
        return None


def getRouteViaAStar(source, dest , option):
    # we will changes this path cost depending upon search option - segments, distance or time
    # For now, let's us start with distance
    pathCost = 0
     # Fringe will have road segments
    route = []
    fringe = []
    cityToHeuristicsMappings = {}
    if not isValidCity(source) or not isValidCity(dest):
        # return empty route and display message that city is not valid
        route.append("Either source or destination city is not valid. Please enter again")
        return route
    routeNotFound = True
    while routeNotFound:
        fringe = []
        while not fringe:
            sortedCityToHeuristicMappings = sorted(cityToHeuristicsMappings.items(), key=operator.itemgetter(1))
            roadSegments_file = open(road_segments_file_location)
            for roadSegmentDetails in roadSegments_file:
                roadSegment = roadSegmentDetails.split()
                city1 = roadSegment[0]
                city2 = roadSegment[1]
                flag1 = city1 == source  and isValidCity(city2) and not isCityVisited(city2,route)
                flag2 = city2 == source  and isValidCity(city1) and not isCityVisited(city1,route)
                
                if flag1 or flag2:
                    fringe.append(roadSegment)
             
            # get second best heuristic value and set this city as source
            if not fringe:
                for i in sortedCityToHeuristicMappings:
                    if not i[0] == source:
                        source = i[0]
                        # remove this city from your heuristic mappings
                        del cityToHeuristicsMappings[i[0]] 
                        break       
        if routeNotFound:
            cityToHeuristicsMappings = {}
            if fringe:
                roadSegmentToTake = []
                city1 = fringe[0][0]
                city2 = fringe[0][1]
                
                if city1 == source:
                    minFunctionCost = pathCost + getHeuristicsByOption(city1,city2,dest,option)
                    cityToHeuristicsMappings[city2] = getHeuristicsByOption(city1,city2,dest,option)
                else:
                    minFunctionCost = pathCost + getHeuristicsByOption(city2,city1,dest,option)
                    cityToHeuristicsMappings[city1] = getHeuristicsByOption(city1,city2,dest,option)
                i = 0
                while fringe:
                    city1 = fringe[i][0]
                    city2 = fringe[i][1]
                    if city1 == source:
                        functionCost = pathCost + getHeuristicsByOption(city1,city2,dest,option)
                        cityToHeuristicsMappings[city2] = getHeuristicsByOption(city1,city2,dest,option)
                    else:
                        functionCost = pathCost + getHeuristicsByOption(city2,city1,dest,option)
                        cityToHeuristicsMappings[city2] = getHeuristicsByOption(city1,city2,dest,option)
                    if(minFunctionCost >= functionCost):
                        minFunctionCost = functionCost
                        roadSegmentToTake = fringe[i]
                    i += 1
                    if i == len(fringe):
                        break
                #print " Road Segment chosen is :"
                #print roadSegmentToTake
                city1 = roadSegmentToTake[0]
                city2 = roadSegmentToTake[1]
                if option.lower() == "distance":
                    pathCost += int(roadSegmentToTake[2])
                elif option.lower() == "time":
                    pathCost += int(roadSegment[2])/int(roadSegment[3])
                elif option.lower() == "segments":
                    route.append("Please give option as time or distance")
                    return route
                if city1 == source:
                    source = city2
                    route.append(city1)
                else:
                    source = city1
                    route.append(city2)
                 
                 # check if we have reached destination
                if source == dest:    
                    route.append(dest)
                    routeNotFound = False
            else:
                route.append("No route exists for your desired destination")
                return route
    return route
#-------------------------------------------------------------------------------------------
##dfs algorithm implementation as a stack
nodecity = roadSegments_file.readlines() # to read all the nodes from file
for i, _ in enumerate(nodecity):
    nodecity[i] = nodecity[i].split() # will split lines as per space and make a matrix of rpows and columns
    
def getRouteViaDFS(nodecity1, city_start1, city_destination1):
    stack = []
    stack.append([city_start1])
    while stack:
        path = stack.pop()
        node = path[-1]
        if node == city_destination1:
            return path
        for i, _ in enumerate(nodecity1):
            if nodecity1[i][0] == node:
                new_path = list(path)
                new_path.append(nodecity1[i][1])
                if nodecity1[i][1] == city_destination1:
                    return new_path
                else:
                    stack.append(new_path)


script, sourceCity ,destinationCity, option , algo = argv

#sourceCity = "Dexter,_Minnesota"#"Dexter,_Minnesota" #"Austin,_Texas"#"Cape_Girardeau,_Missouri"# "Fort_Meade,_Maryland" #"Bloomington,_Indiana"
#destinationCity = "Winona,_Minnesota"  #"Winona,_Minnesota" #"Windcrest,_Texas" #"Sikeston,_Missouri" "Arbutus,_Maryland"#"Indianapolis,_Indiana"

if algo == "bfs":
    bfsRoute = getRouteViaBFS(sourceCity,destinationCity)
    print "*************************BFS Route*******************************"
    print bfsRoute
    lastStringInRoute = bfsRoute[(len(bfsRoute) - 1)]
    if isValidCity(lastStringInRoute):
        print getTimeAndDistance(bfsRoute)
    else:
        print lastStringInRoute
elif algo == "dfs":
    dfsRoute= getRouteViaDFS(nodecity,sourceCity,destinationCity)
    print "*************************DFS Route*******************************"
    print dfsRoute
    lastStringInRoute = dfsRoute[(len(dfsRoute) - 1)]
    if dfsRoute:
        if isValidCity(lastStringInRoute):
            print getTimeAndDistance(dfsRoute)
        else:
            print lastStringInRoute
elif algo == "astar":
    astarRoute = getRouteViaAStar(sourceCity,destinationCity,option)
    print "=========================A* Route (Time)=============================="
    print astarRoute
    lastStringInRoute =  astarRoute[(len(astarRoute) - 1)]
    if isValidCity(lastStringInRoute):
        print getTimeAndDistance(astarRoute)
    else:
        print lastStringInRoute
        

    

