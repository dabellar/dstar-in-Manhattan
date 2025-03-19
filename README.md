# Pathfinding in Manhattan

### About 
In the winter term of 2024-2025, I took CS 133B, the second part of Caltech's robotics courses. For the final project, we needed to use the pathfinding algorithms we had learned in class and expand upon them. I chose to use A* and D* to drive from a start and goal node in a map simulating Manhattan. 

#### Source/Credits
The python package I used to create the map is from [OSMnx](https://geoffboeing.com/publications/osmnx-paper/). It uses Open Street Map data for modeling, analyzing, and visualizing urban networks. I also used the [Open Street Map Wiki](https://wiki.openstreetmap.org/wiki/Main_Page) as guide to read the data I was getting.

The D* Lite algorithm was adapted from *Fast Replanning for Navigation in Unknown Terrain* by Sven Koenig and Maxim Likhacev (Link to paper [here](https://ieeexplore.ieee.org/document/1435479)). 

### How to Run the Project

#### Requirements 
For development, you will need [OSMnx](https://osmnx.readthedocs.io/en/stable/installation.html) installed in your environment. Before running, you will need to activate for the visualization to work.

#### Running the Project  
After cloning the repository, use `python3 main.py` to run the visualization. The start and goal nodes are hardcoded in (start: E 92nd St and 5th Ave, goal: W 106th St and Central Park West, map focused on 97th Transverse). To find specific intersections, add the names of the streets/avenues to get_intersection.py and run. The file includes some other examples of graph views, starts, and goals.

#### Tips:
If you want to use the bounding box feature to get a specific portion of the map, use https://norbertrenner.de/osm/bbox.html to get latitude and longitude.  
