# libs
import os
import lanelet2
import lanelet2.core as lncore

# load the map, for example autonomoustuff
osm_path = os.path.join(os.path.dirname(os.path.abspath('')), "Town01.osm")
print("using OSM: %s (exists? %s)" % (osm_path, os.path.exists(osm_path)))

# load map from origin
lorigin = lanelet2.io.Origin(0.0, 0.0, 0.0) #node 7550 37.9970032767 125.001162728768
lmap = lanelet2.io.load(osm_path, lorigin)

# ... and traffic rules (Germany is the sole location, for now)
trafficRules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany, lanelet2.traffic_rules.Participants.Vehicle)
# print("start")
graph = lanelet2.routing.RoutingGraph(lmap, trafficRules)
# print("end")

# create routing graph, and select start lanelet and end lanelet for the shortest Path 
# start_lane_num = input("start: ")
# end_lane_num = input("end: ")
start_lane_num = 8000
end_lane_num = 13525
startLane = lmap.laneletLayer[start_lane_num] # lanelet IDs 8000     13663
endLane = lmap.laneletLayer[end_lane_num] #13525       14011
route = graph.getRoute(startLane, endLane)

if route is None:
    print("error: no route was calculated")
else:
    sp = route.shortestPath()
    print(sp)
    if sp is None:
        print ("error: no shortest path was calculated")
    else:
        print [l.id for l in sp.getRemainingLane(startLane)] if sp else None

# save the path in another OSM map with a special tag to highlight it
# if sp:
#     for llet in sp.getRemainingLane(startLane):
#         lmap.laneletLayer[llet.id].attributes["shortestPath"] = "true"
#     projector = lanelet2.projection.MercatorProjector(lorigin)
#     sp_path = os.path.join(os.path.dirname(osm_path), os.path.basename(osm_path).split(".")[0] + "_shortestpath.osm")
#     lanelet2.io.write(sp_path, lmap, projector)

# now display in JOSM both, and you can see the path generated over the JOSM map 
# Ctrl+F -->  type:relation "type"="lanelet" "shortestPath"="True"
# and the path will be highlighted as the image below