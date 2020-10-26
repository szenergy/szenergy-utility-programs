
#!/usr/bin/env python
#############################################################
## from http://code.activestate.com/recipes/534109-xml-to-python-data-structure/

import re
import xml.sax.handler

def xml2obj(src):
    """
    A simple function to converts XML data into native Python object.
    """

    non_id_char = re.compile('[^_0-9a-zA-Z]')
    def _name_mangle(name):
        return non_id_char.sub('_', name)

    class DataNode(object):
        def __init__(self):
            self._attrs = {}    # XML attributes and child elements
            self.data = None    # child text data
        def __len__(self):
            # treat single element as a list of 1
            return 1
        def __getitem__(self, key):
            if isinstance(key, str):  # basestring python 2 version
                return self._attrs.get(key,None)
            else:
                return [self][key]
        def __contains__(self, name):
            return self._attrs.has_key(name)
        def __nonzero__(self):
            return bool(self._attrs or self.data)
        def __getattr__(self, name):
            if name.startswith('__'):
                # need to do this for Python special methods???
                raise AttributeError(name)
            return self._attrs.get(name,None)
        def _add_xml_attr(self, name, value):
            if name in self._attrs:
                # multiple attribute of the same name are represented by a list
                children = self._attrs[name]
                if not isinstance(children, list):
                    children = [children]
                    self._attrs[name] = children
                children.append(value)
            else:
                self._attrs[name] = value
        def __str__(self):
            return self.data or ''
        def __repr__(self):
            items = sorted(self._attrs.items())
            if self.data:
                items.append(('data', self.data))
            return u'{%s}' % ', '.join([u'%s:%s' % (k,repr(v)) for k,v in items])

    class TreeBuilder(xml.sax.handler.ContentHandler):
        def __init__(self):
            self.stack = []
            self.root = DataNode()
            self.current = self.root
            self.text_parts = []
        def startElement(self, name, attrs):
            self.stack.append((self.current, self.text_parts))
            self.current = DataNode()
            self.text_parts = []
            # xml attributes --> python attributes
            for k, v in attrs.items():
                self.current._add_xml_attr(_name_mangle(k), v)
        def endElement(self, name):
            text = ''.join(self.text_parts).strip()
            if text:
                self.current.data = text
            if self.current._attrs:
                obj = self.current
            else:
                # a text only node is simply represented by the string
                obj = text or ''
            self.current, self.text_parts = self.stack.pop()
            self.current._add_xml_attr(_name_mangle(name), obj)
        def characters(self, content):
            self.text_parts.append(content)

    builder = TreeBuilder()
    if isinstance(src,str): # basestring python 2 version
        xml.sax.parseString(src, builder)
    else:
        xml.sax.parse(src, builder)
    values_view = builder.root._attrs.values()
    value_iterator = iter(values_view)
    first_value = next(value_iterator)
    # return builder.root._attrs.values()[0] # python 2 version
    return first_value



#############################################################



def main():



    # make dictionary of node IDs
    nodes = {}
    for node in myMap['node']:
        nodes[node['id']] = node
        
    ways = {}
    for way in myMap['way']:
        ways[way['id']]=way

        
    print(len(nodes))
    print(len(ways))

    # draw the map

    import pylab as p

    ##p.ion()
    ##
    ##p.figure()
    ##for way in myMap['way']:
    ##    lastPoint=None
    ##    for nd in way['nd']:
    ##        newPointLat = nodes[nd['ref']]['lat']
    ##        newPointLong= nodes[nd['ref']]['lon']
    ##        newPoint = (float(newPointLat),float(newPointLong))
    ##        if not lastPoint==None:
    ##            lastX = lastPoint[0]
    ##            lastY = lastPoint[1]
    ##            newX  = newPoint[0]
    ##            newY  = newPoint[1]
    ##            p.plot([lastX,newX],[lastY,newY],'-b')
    ##        lastPoint = newPoint
    ##      
    ##p.ioff()
    ##p.show()

    ###p.ion()
    ##
    ##p.figure()
    ##for idx,nodeID in enumerate(nodes.keys()):
    ##    y = float(nodes[nodeID]['lat'])
    ##    x = float(nodes[nodeID]['lon'])
    ##    p.plot([x],[y],'.b')
    ##    if idx%1000 == 0:
    ##        print(idx)
    ##      
    ##p.ioff()
    ##p.show()


    # try to find a list of all intersections

    # build matrix of first and last node ids for each way

    print('Enumerating end nodes.')
    endNodes = {}
    wayType  = {}
    for wayID in ways.keys():
        endNodes[wayID] = [ ways[wayID]['nd'][0], ways[wayID]['nd'][-1]]    
        wayTags         = ways[wayID]['tag']
        if not wayTags==None:
            hwyTypeList  = [d['v'] for d in wayTags if d['k']=='highway']
            if len(hwyTypeList)>0:
                    wayType[wayID] = hwyTypeList[0]

    print('Done enumerating end nodes')
    print(wayType)
    print(set([wayType[k] for k in wayType.keys()]))
    roadTypes = set([wayType[k] for k in wayType.keys()])

    #p.ion()

    renderingRules = {
        'primary': dict(
                marker          = 'D',
                markeredgecolor = 'b',
                markeredgewidth = 1,
                markerfacecolor = 'b', 
                markersize      = 2,                   
                linestyle       = '-',
                linewidth       = 10,
                color           = (0.7,0.7,1.0,0.1),
                alpha           = 1.0,
                solid_capstyle  = 'round',
                solid_joinstyle = 'round',
                zorder          = -1,
                markerzorder    = 0,
                _firstmarker          = 'x',
                _firstmarkeredgecolor = 'g',
                _firstmarkeredgewidth = 1,
                _firstmarkerfacecolor = 'g', 
                _firstmarkersize      = 8,
                _firstzorder          = 1,   
                _lastmarker           = 'o',
                _lastmarkeredgecolor  = 'r',
                _lastmarkeredgewidth  = 1,
                _lastmarkerfacecolor  = 'r', 
                _lastmarkersize       = 8,
                _lastzorder           = 0,   
                ),
        'primary_link': dict(
                marker          = 'D',
                markeredgecolor = 'b',
                markeredgewidth = 1,
                markerfacecolor = 'b', 
                markersize      = 2,                   
                linestyle       = '-',
                linewidth       = 8,
                color           = (0.7,0.7,1.0,0.1),
                alpha           = 1.0,
                solid_capstyle  = 'round',
                solid_joinstyle = 'round',
                zorder          = -1,            
                markerzorder    = 0,
                _firstmarker          = 'x',
                _firstmarkeredgecolor = 'g',
                _firstmarkeredgewidth = 1,
                _firstmarkerfacecolor = 'g', 
                _firstmarkersize      = 12,   
                _firstzorder          = 1,            
                _lastmarker           = 'o',
                _lastmarkeredgecolor  = 'r',
                _lastmarkeredgewidth  = 1,
                _lastmarkerfacecolor  = 'r', 
                _lastmarkersize       = 8,   
                _lastzorder           = 0, 
                ),
        'secondary': dict(
                marker          = 'D',
                markeredgecolor = 'b',
                markeredgewidth = 1,
                markerfacecolor = 'b', 
                markersize      = 2,                   
                linestyle       = '-',
                linewidth       = 6,
                color           = (0.2,0.2,0.7,0.1),
                alpha           = 0.5,
                solid_capstyle  = 'round',
                solid_joinstyle = 'round',
                zorder          = -2,            
                markerzorder    = 0,
                _firstmarker          = 'x',
                _firstmarkeredgecolor = 'g',
                _firstmarkeredgewidth = 1,
                _firstmarkerfacecolor = 'g', 
                _firstmarkersize      = 10,   
                _firstzorder          = 1,            
                _lastmarker           = 'o',
                _lastmarkeredgecolor  = 'r',
                _lastmarkeredgewidth  = 1,
                _lastmarkerfacecolor  = 'r', 
                _lastmarkersize       = 8,   
                _lastzorder           = 0, 
                ),
        'secondary_link': dict(
                marker          = 'D',
                markeredgecolor = 'b',
                markeredgewidth = 1,
                markerfacecolor = 'b', 
                markersize      = 2,                   
                linestyle       = '-',
                linewidth       = 6,
                color           = (0.2,0.2,0.7,0.1),
                alpha           = 0.5,
                solid_capstyle  = 'round',
                solid_joinstyle = 'round',
                zorder          = -2,            
                markerzorder    = 0,
                _firstmarker          = 'x',
                _firstmarkeredgecolor = 'g',
                _firstmarkeredgewidth = 1,
                _firstmarkerfacecolor = 'g', 
                _firstmarkersize      = 10,   
                _firstzorder          = 1,            
                _lastmarker           = 'o',
                _lastmarkeredgecolor  = 'r',
                _lastmarkeredgewidth  = 1,
                _lastmarkerfacecolor  = 'r', 
                _lastmarkersize       = 8,   
                _lastzorder           = 0, 
                ),
        'tertiary': dict(
                marker          = 'D',
                markeredgecolor = 'b',
                markeredgewidth = 1,
                markerfacecolor = 'b', 
                markersize      = 2,                   
                linestyle       = '-',
                linewidth       = 4,
                color           = (0.0,0.0,0.7,0.1),
                alpha           = 0.5,
                solid_capstyle  = 'round',
                solid_joinstyle = 'round',
                zorder          = -3,            
                markerzorder    = 0,
                _firstmarker          = 'x',
                _firstmarkeredgecolor = 'g',
                _firstmarkeredgewidth = 1,
                _firstmarkerfacecolor = 'g', 
                _firstmarkersize      = 10,   
                _firstzorder          = 1,            
                _lastmarker           = 'o',
                _lastmarkeredgecolor  = 'r',
                _lastmarkeredgewidth  = 1,
                _lastmarkerfacecolor  = 'r', 
                _lastmarkersize       = 8,   
                _lastzorder           = 0, 
                ),
        'tertiary_link': dict(
                marker          = 'D',
                markeredgecolor = 'b',
                markeredgewidth = 1,
                markerfacecolor = 'b', 
                markersize      = 2,                   
                linestyle       = '-',
                linewidth       = 4,
                color           = (0.0,0.0,0.7,0.1),
                alpha           = 0.5,
                solid_capstyle  = 'round',
                solid_joinstyle = 'round',
                zorder          = -3,            
                markerzorder    = 0,
                _firstmarker          = 'x',
                _firstmarkeredgecolor = 'g',
                _firstmarkeredgewidth = 1,
                _firstmarkerfacecolor = 'g', 
                _firstmarkersize      = 10,   
                _firstzorder          = 1,            
                _lastmarker           = 'o',
                _lastmarkeredgecolor  = 'r',
                _lastmarkeredgewidth  = 1,
                _lastmarkerfacecolor  = 'r', 
                _lastmarkersize       = 8,   
                _lastzorder           = 0, 
                ),
        'residential': dict(
                marker          = 'D',
                markeredgecolor = 'b',
                markeredgewidth = 1,
                markerfacecolor = 'b', 
                markersize      = 2,                   
                linestyle       = '-',
                linewidth       = 1,
                color           = (0.1,0.1,0.1,1.0),
                alpha           = 1.0,
                solid_capstyle  = 'round',
                solid_joinstyle = 'round',
                zorder          = -99,            
                markerzorder    = 0,
                _firstmarker          = 'x',
                _firstmarkeredgecolor = 'g',
                _firstmarkeredgewidth = 1,
                _firstmarkerfacecolor = 'g', 
                _firstmarkersize      = 10,   
                _firstzorder          = 1,            
                _lastmarker           = 'o',
                _lastmarkeredgecolor  = 'r',
                _lastmarkeredgewidth  = 1,
                _lastmarkerfacecolor  = 'r', 
                _lastmarkersize       = 8,   
                _lastzorder           = 0, 
                ),            
        'unclassified': dict(
                marker          = 'D',
                markeredgecolor = (0.5,0.5,0.5),
                markeredgewidth = 1,
                markerfacecolor = (0.5,0.5,0.5), 
                markersize      = 2,                   
                linestyle       = ':',
                linewidth       = 1,
                color           = (0.5,0.5,0.5),
                alpha           = 0.5,
                solid_capstyle  = 'round',
                solid_joinstyle = 'round',
                zorder          = -1,            
                markerzorder    = 0,
                _firstmarker          = 'x',
                _firstmarkeredgecolor = 'g',
                _firstmarkeredgewidth = 1,
                _firstmarkerfacecolor = 'g', 
                _firstmarkersize      = 6,   
                _firstzorder          = 1,            
                _lastmarker           = 'o',
                _lastmarkeredgecolor  = 'r',
                _lastmarkeredgewidth  = 1,
                _lastmarkerfacecolor  = 'r', 
                _lastmarkersize       = 6,   
                _lastzorder           = 0,             
                ),
        'default': dict(
                marker          = 'D',
                markeredgecolor = 'b',
                markeredgewidth = 1,
                markerfacecolor = 'b', 
                markersize      = 2,                   
                linestyle       = '-',
                linewidth       = 3,
                color           = 'b',
                alpha           = 0.5,
                solid_capstyle  = 'round',
                solid_joinstyle = 'round',
                zorder          = -1,            
                markerzorder    = 0,
                _firstmarker          = 'x',
                _firstmarkeredgecolor = 'b',
                _firstmarkeredgewidth = 1,
                _firstmarkerfacecolor = 'b', 
                _firstmarkersize      = 6,   
                _firstzorder          = 1,              
                _lastmarker     = 'o',
                _lastmarkeredgecolor = 'b',
                _lastmarkeredgewidth = 1,
                _lastmarkerfacecolor = 'b', 
                _lastmarkersize      = 6,  
                _lastzorder           = 0,                     
                ),
        }
                        




    for idx,nodeID in enumerate(nodes.keys()):
        if idx==0:
            minX = float(nodes[nodeID]['lon'])
            maxX = float(nodes[nodeID]['lon'])
            minY = float(nodes[nodeID]['lat'])
            maxY = float(nodes[nodeID]['lat'])
        else:
            minX = min(minX,float(nodes[nodeID]['lon']))
            maxX = max(maxX,float(nodes[nodeID]['lon']))
            minY = min(minY,float(nodes[nodeID]['lat']))
            maxY = max(maxY,float(nodes[nodeID]['lat']))
            
    minX = float(myMap['bounds']['minlon'])
    maxX = float(myMap['bounds']['maxlon'])
    minY = float(myMap['bounds']['minlat'])
    maxY = float(myMap['bounds']['maxlat'])



    fig = p.figure()
    ax = fig.add_subplot(111,autoscale_on=False,xlim=(minX,maxX),ylim=(minY,maxY))
    for idx,nodeID in enumerate(wayType.keys()):
##        if idx>100:
##            break
        try:
            if wayType[nodeID] in ['primary','primary_link','unclassified',
                            'secondary','secondary_link',
                            'tertiary','tertiary_link',
                            'residential',
                            ]:
            
                zz=['primary_link','primary',
                            'secondary','secondary_link',
                            'tertiary','tertiary_link',
                            'motorway','motorway_link',
                            'service',
                            'residential'
                            'unclassified',
                            ]
                oldX = None
                oldY = None
                thisRoadType = wayType[nodeID]
                
                if thisRoadType in renderingRules.keys():
                    thisRendering = renderingRules[thisRoadType]
                else:
                    thisRendering = renderingRules['default']
                    
                for nCnt,nID in enumerate(ways[nodeID]['nd']):
                    y = float(nodes[nID['ref']]['lat'])
                    x = float(nodes[nID['ref']]['lon'])
                    if oldX == None:
                        # first point in road way
                        p.plot([x],[y],
                            marker          = thisRendering['_firstmarker'],
                            markeredgecolor = thisRendering['_firstmarkeredgecolor'],
                            markeredgewidth = thisRendering['_firstmarkeredgewidth'],
                            markerfacecolor = thisRendering['_firstmarkerfacecolor'],
                            markersize      = thisRendering['_firstmarkersize'],    
                            zorder          = thisRendering['_firstzorder'],               
                            )
                    elif nCnt<(len(ways[nodeID]['nd'])-1):
                        p.plot([oldX,x],[oldY,y],
                            marker          = '',
                            linestyle       = thisRendering['linestyle'],
                            linewidth       = thisRendering['linewidth'],
                            color           = thisRendering['color'],
                            alpha           = thisRendering['alpha'],
                            solid_capstyle  = thisRendering['solid_capstyle'],
                            solid_joinstyle = thisRendering['solid_joinstyle'],
                            zorder          = thisRendering['zorder'],
                            )
                        p.plot([x],[y],
                            marker          = thisRendering['marker'],
                            markeredgecolor = thisRendering['markeredgecolor'],
                            markeredgewidth = thisRendering['markeredgewidth'],
                            markerfacecolor = thisRendering['markerfacecolor'],
                            markersize      = thisRendering['markersize'],                   
                            color           = thisRendering['color'],
                            alpha           = thisRendering['alpha'],
                            zorder          = thisRendering['markerzorder'],
                            )
                    else:
                        # last segment in road way
                        p.plot([oldX,x],[oldY,y],
                            marker          = '',
                            linestyle       = thisRendering['linestyle'],
                            linewidth       = thisRendering['linewidth'],
                            color           = thisRendering['color'],
                            alpha           = thisRendering['alpha'],
                            solid_capstyle  = thisRendering['solid_capstyle'],
                            solid_joinstyle = thisRendering['solid_joinstyle'],
                            zorder          = thisRendering['zorder'],                        
                            )                    
                    oldX = x
                    oldY = y
                # last point in road way
                p.plot([x],[y],
                            marker          = thisRendering['_lastmarker'],
                            markeredgecolor = thisRendering['_lastmarkeredgecolor'],
                            markeredgewidth = thisRendering['_lastmarkeredgewidth'],
                            markerfacecolor = thisRendering['_lastmarkerfacecolor'],
                            markersize      = thisRendering['_lastmarkersize'],
                            zorder          = thisRendering['_lastzorder'],                        
                        )
                       
        except KeyError:
            pass
        if idx%100 == 0:
            print(idx)
          
    #p.ioff()
    print('Done Plotting')
    p.show()



def nodeDist(nodes,oldNodeID,newNodeID):
    y0 = float(nodes[oldNodeID]['lat'])
    x0 = float(nodes[oldNodeID]['lon'])
    y1 = float(nodes[newNodeID]['lat'])
    x1 = float(nodes[newNodeID]['lon'])

    return ((x0-x1)**2+(y0-y1)**2)**(0.5)

from scipy import sparse

def linkNodes():
    
    # make dictionary of node IDs
    nodes = {}
    for node in myMap['node']:
        nodes[node['id']] = node
        
    ways = {}
    for way in myMap['way']:
        ways[way['id']]=way
        
        
    neighborDict = {}

        
    print(len(nodes))
    print(len(ways))
    
    M = sparse.lil_matrix((len(nodes),len(nodes)))
    
    nodeID_List = sorted(nodes.keys())
    
    # walk each way and enter distance for nodes in way node list
    
    for wayID in ways.keys():
        way = ways[wayID]
        wayTags         = way['tag']
        if not wayTags==None:
            hwyTypeList  = [d['v'] for d in wayTags if d['k']=='highway']
            if len(hwyTypeList)>0:
                    wayType = hwyTypeList[0]        
        else:
            wayType = None
        if wayType in ['primary_link','primary',
                        'secondary','secondary_link',
                        'tertiary','tertiary_link',
                        'motorway','motorway_link',
                        'service',
                        'residential',
                        'unclassified',
                        'multipolygon'
                        ]:
            # this way is a highway of some sort
            for idx,node in enumerate(way['nd']):
                if idx==0:
                    oldNodeID = node['ref']
                    oldNodeIdx = nodeID_List.index(oldNodeID)
                else:
                    newNodeID = node['ref']
                    distance = nodeDist(nodes,oldNodeID,newNodeID)
                    newNodeIdx = nodeID_List.index(newNodeID)
                    M[oldNodeIdx,newNodeIdx]=distance
                    M[newNodeIdx,oldNodeIdx]=distance
                    
                    try:
                        neighborDict[oldNodeID].append(newNodeID)
                    except KeyError:
                        neighborDict[oldNodeID] = [newNodeID]
                    try:
                        neighborDict[newNodeID].append(oldNodeID)
                    except KeyError:
                        neighborDict[newNodeID] = [oldNodeID] 
                                           
                    oldNodeID  = newNodeID
                    oldNodeIdx = newNodeIdx
                
    return (M,nodeID_List, neighborDict)
                        
        




##        
##    print('Linking end nodes')
##
##    wayLinks = {}
##    for firstWayID in wayType.keys():
##        firstNodeSet = set([id['ref'] for id in ways[firstWayID]['nd']])
##        for secondWayID in wayType.keys():
##            secondNodeSet = set([id['ref'] for id in ways[secondWayID]['nd']])
##            if any(firstNodeSet.intersection(secondNodeSet)):
##                if firstWayID <> secondWayID:
##                    try:
##                        wayLinks[firstWayID].append(secondWayID)
##                    except KeyError:
##                        wayLinks[firstWayID]=[secondWayID]
##            
##    ##        if ((endNodes[firstWayID][0] in ways[secondWayID]['nd']) or 
##    ##            (endNodes[firstWayID][1] in ways[secondWayID]['nd'])):
##    ##                if firstWayID <> secondWayID:
##    ##                    try:
##    ##                        wayLinks[firstWayID].append(secondWayID)
##    ##                    except KeyError:
##    ##                        wayLinks[firstWayID]=[secondWayID]
##                    
##      
##    print(len(wayLinks.keys()))

##            
##import cProfile
##cProfile.run('main()','multiWeightingRouting_002.prof')




#src = file("map_campus_gyor01.osm")  # basestring python 2 version
src = open("map_campus_gyor01.osm")
myMap = xml2obj(src)
main()
(M,nodeID_List,neighborDict)=linkNodes()

##from matplotlib.pyplot import figure, show
##import numpy
##
##fig = figure()
##ax1 = fig.add_subplot(111)
##
##ax1.spy(M, precision=0.00001)
##
##show()

""""

from AStarSearch import *

        
def neighbor_nodes(x):
    rowIdx = nodeID_List.index(x)
    row = M[rowIdx,:]
    rowValues = []
    for i in range(0,row.get_shape()[1]):
        rowValues.append(row[0,i])
    nodeList = [nodeID_List[colIdx] for colIdx,val in enumerate(rowValues) if val>0]
    return nodeList

##def build_neighbor_Dict():
##    neighborDict = {}
##    for rowIdx,nodeID in enumerate(nodeID_List):
##        row = M[rowIdx,:]
##        rowValues = []
##        for i in range(0,row.get_shape()[1]):
##            rowValues.append(row[0,i])
##        nodeList = [nodeID_List[colIdx] for colIdx,val in enumerate(rowValues) if val>0]
##        neighborDict[nodeID]=nodeList
##        
##        if rowIdx%100 == 0: print '... finished row #%i' % rowIdx
##    return neighborDict
    
def neighbor_node_lookup(x):
    return neighborDict[x]

def distance_between(x,y):
    rowIdx = nodeID_List.index(x)
    colIdx = nodeID_List.index(y)
    return M[rowIdx,colIdx]


##print 'Building Neighbor Node Dictionary'
##neighborDict = build_neighbor_Dict()
print 'Routing...'
print Astar("61916608","820007070",distance_between,neighbor_node_lookup,distance_between)    
print 'Done'



"""